"""Task-space reach-and-dwell success monitor -- the MEASURING instrument.

Scores the locked success definition of docs/p1_g4_reach_dwell.md §A1:

    position    : ||p_tool - p_cmd||        <  5 mm
    orientation : angle(a_tool, a_cmd)      <  5 deg   (approach axis, not roll)
    dwell       : 2.0 s CONTINUOUS, sampled >= 10 Hz

One out-of-tolerance sample RESETS the dwell window -- not an average, not "90% of
samples". The claim being tested is that the pose is HELD, and an average cannot
tell "held" from "flown through and came back".

N-arm concurrent success = there exists ONE window of `dwell` seconds in which ALL
armed arms satisfy their own tolerance SIMULTANEOUSLY. Not "each arm succeeded
somewhere in this episode" -- that is satisfiable by taking turns, and taking turns
is exactly what the prior work does (p1_plan.md §2c, Harada 2015 "either... or").

WHY THIS IS A SEPARATE NODE from gantry_reach_executor: whatever scores success
must not also be what chooses candidates, otherwise the criterion can drift toward
the result. This node is a pure observer -- it commands nothing, and adds no
traffic to the Modbus or Kortex buses (TF is already broadcast; it only reads the
buffer). That matters: the G3 session measured the table poll loop running at
7.4 Hz, not 10 Hz, purely from lock-free RS-485 contention.

This does NOT replace gantry_reach_executor's joint-space `reach_tol` check -- that
one stays useful as a controller-convergence guard. But it is NOT the task success
definition and must not be quoted as one: its 0.03 is radians for arm joints and
METRES for the gantry linear axis, i.e. it passes a 3 cm gantry miss.

    ros2 run reachability_gng reach_dwell_monitor
    ros2 topic pub --once /reach_dwell/target/arm_1 geometry_msgs/PoseStamped ...
    ros2 topic pub --once /reach_dwell/clear std_msgs/String "{data: 'all'}"
"""
from __future__ import annotations

import csv
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)


def approach_axis(qx, qy, qz, qw):
    """The frame's local +Z expressed in the parent frame -- the approach/tool
    direction for the Kinova tool_frame. Cheaper and better conditioned than
    building the full rotation matrix, and +Z is the only axis the success
    definition constrains (roll about the tool axis is deliberately free)."""
    return np.array([2.0 * (qx * qz + qw * qy),
                     2.0 * (qy * qz - qw * qx),
                     1.0 - 2.0 * (qx * qx + qy * qy)])


class Task:
    """One arm's active reach-and-dwell target and its running verdict."""

    def __init__(self, arm, pose, t_set):
        self.arm = arm
        self.p_cmd = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.a_cmd = approach_axis(pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w)
        self.t_set = t_set
        self.in_tol_since = None     # None = not currently inside tolerance
        self.samples = []            # (t, pos_err, ori_err) inside current window
        self.n_samples = 0           # every sample, for the effective-rate check
        self.n_tf_fail = 0
        self.peak_effort = {}        # joint name -> max |tau| since the target was set
        self.done_at = None
        self.result = None           # dict, filled on success
        # Reporting flag, NOT a gate on evaluation. An arm that has met its dwell
        # is still HOLDING its target and must keep counting toward the common
        # window -- dropping it the moment it succeeds makes N-arm concurrency
        # undetectable unless every arm happens to arrive inside the same window.
        self.reported = False


class ReachDwellMonitor(Node):
    def __init__(self):
        super().__init__('reach_dwell_monitor')
        self.declare_parameter('arms', ['arm_1', 'arm_2', 'arm_3', 'arm_4'])
        self.declare_parameter('tool_frames',
                               ['t1_a1_tool_frame', 't1_a2_tool_frame',
                                't2_a1_tool_frame', 't2_a2_tool_frame'])
        self.declare_parameter('world_frame', 'world')
        # The locked thresholds (docs/p1_g4_reach_dwell.md §A1). Exposed as
        # parameters so a run can be REPORTED at a different tolerance, not so the
        # bar can be moved after seeing the data -- every CSV row records the
        # thresholds it was scored against.
        self.declare_parameter('pos_tol', 0.005)        # m
        self.declare_parameter('ori_tol_deg', 5.0)      # deg
        self.declare_parameter('dwell', 2.0)            # s
        self.declare_parameter('rate', 20.0)            # Hz
        self.declare_parameter('min_rate', 10.0)        # §A7.4 validity floor
        self.declare_parameter('csv_log', '')           # base path; '' = off

        g = self.get_parameter
        arms = [str(a) for a in g('arms').value]
        frames = [str(f) for f in g('tool_frames').value]
        if len(arms) != len(frames):
            raise ValueError(
                f'arms ({len(arms)}) and tool_frames ({len(frames)}) must be the '
                'same length -- they are positionally paired')
        self.tool_frame = dict(zip(arms, frames))
        self.world_frame = str(g('world_frame').value)
        self.pos_tol = float(g('pos_tol').value)
        self.ori_tol = math.radians(float(g('ori_tol_deg').value))
        self.dwell = float(g('dwell').value)
        self.rate = float(g('rate').value)
        self.min_rate = float(g('min_rate').value)

        self._lock = threading.Lock()
        self.tasks = {}                 # arm -> Task (active or just-finished)
        self.concurrent_since = None    # start of the current all-arms-in-tol window
        self.concurrent_reported = False
        self.best_concurrent = 0        # largest N that has held a full window
        self._effort = {}               # joint name -> latest |tau|

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        for a in arms:
            self.create_subscription(
                PoseStamped, f'/reach_dwell/target/{a}',
                lambda msg, arm=a: self._on_target(arm, msg), 10)
        self.create_subscription(String, '/reach_dwell/clear', self._on_clear, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.status_pub = self.create_publisher(String, '/reach_dwell/status', 10)

        self._csv_base = str(g('csv_log').value)
        self._samples_csv = None
        self._samples_w = None
        if self._csv_base:
            self._open_csv()

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'reach_dwell_monitor up: pos<{self.pos_tol * 1000:.1f}mm '
            f'ori<{math.degrees(self.ori_tol):.1f}deg dwell={self.dwell:.1f}s '
            f'@{self.rate:.0f}Hz over {arms}. Observer only -- commands nothing.')

    # ---------------------------------------------------------------- inputs

    def _on_target(self, arm, msg):
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().error(
                f'{arm}: target frame_id "{msg.header.frame_id}" != world_frame '
                f'"{self.world_frame}" -- REJECTED. This node does not transform '
                'targets; a silently reframed target would corrupt every error.')
            return
        with self._lock:
            self.tasks[arm] = Task(arm, msg.pose, time.time())
            self.concurrent_since = None
            self.concurrent_reported = False
        p = self.tasks[arm].p_cmd
        self.get_logger().info(
            f'{arm}: target set ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) in '
            f'{self.world_frame}')

    def _on_clear(self, msg):
        who = msg.data.strip()
        with self._lock:
            if who in ('', 'all'):
                self.tasks.clear()
            else:
                self.tasks.pop(who, None)
            self.concurrent_since = None
            self.concurrent_reported = False
        self.get_logger().info(f'cleared: {who or "all"}')

    def _on_joints(self, msg):
        if not msg.effort:
            return
        n = min(len(msg.name), len(msg.effort))
        with self._lock:
            for i in range(n):
                self._effort[msg.name[i]] = abs(float(msg.effort[i]))

    # ---------------------------------------------------------------- scoring

    def _measure(self, task):
        """(pos_err_m, ori_err_rad) or None when TF is unavailable."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.world_frame, self.tool_frame[task.arm], rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        p = np.array([t.x, t.y, t.z])
        a = approach_axis(r.x, r.y, r.z, r.w)
        pos_err = float(np.linalg.norm(p - task.p_cmd))
        # Both axes are unit by construction; clip guards the arccos against
        # 1+1e-16 from floating point, which would return NaN and silently poison
        # the verdict rather than failing loudly.
        cos = float(np.clip(np.dot(a, task.a_cmd), -1.0, 1.0))
        return pos_err, math.acos(cos)

    def _tick(self):
        now = time.time()
        with self._lock:
            # Every task with a live target, INCLUDING ones that already met their
            # dwell -- they are still holding, and concurrency is about holding.
            active = list(self.tasks.values())
            if not active:
                return
            all_in_tol = True
            for task in active:
                m = self._measure(task)
                if m is None:
                    task.n_tf_fail += 1
                    task.in_tol_since = None
                    all_in_tol = False
                    continue
                pos_err, ori_err = m
                task.n_samples += 1
                for jn, tau in self._effort.items():
                    if tau > task.peak_effort.get(jn, 0.0):
                        task.peak_effort[jn] = tau
                in_tol = pos_err < self.pos_tol and ori_err < self.ori_tol
                if in_tol:
                    if task.in_tol_since is None:
                        task.in_tol_since = now
                        task.samples = []
                    task.samples.append((now, pos_err, ori_err))
                else:
                    task.in_tol_since = None
                    task.samples = []
                    all_in_tol = False
                self._write_sample(now, task, pos_err, ori_err, in_tol)
                if (task.in_tol_since is not None and not task.reported
                        and now - task.in_tol_since >= self.dwell):
                    self._succeed(task, now)

            # Concurrency: the window is COMMON, so it starts when the LAST arm
            # entered tolerance and dies the moment any one of them leaves.
            if all_in_tol and len(active) > 1:
                start = max(t.in_tol_since for t in active)
                if self.concurrent_since is None or start > self.concurrent_since:
                    self.concurrent_since = start
                if (now - self.concurrent_since >= self.dwell
                        and not self.concurrent_reported):
                    self.concurrent_reported = True
                    self.best_concurrent = max(self.best_concurrent, len(active))
                    self.get_logger().info(
                        f'>>> {len(active)}-ARM CONCURRENT dwell held {self.dwell:.1f}s '
                        f'(common window) -- arms {[t.arm for t in active]}')
                    self._publish({'event': 'concurrent',
                                   'n_arms': len(active),
                                   'arms': [t.arm for t in active],
                                   'dwell_s': self.dwell})
            else:
                # The common window died: the next one is a NEW window and gets
                # reported on its own merits.
                self.concurrent_since = None
                self.concurrent_reported = False

    def _succeed(self, task, now):
        span = now - task.t_set
        pos = [s[1] for s in task.samples]
        ori = [s[2] for s in task.samples]
        eff_rate = task.n_samples / span if span > 0 else 0.0
        task.done_at = now
        task.reported = True
        task.result = {
            'event': 'success',
            'arm': task.arm,
            'time_to_dwell_s': round(span, 3),
            'pos_err_max_mm': round(max(pos) * 1000.0, 3),
            'pos_err_mean_mm': round(float(np.mean(pos)) * 1000.0, 3),
            'ori_err_max_deg': round(math.degrees(max(ori)), 3),
            'ori_err_mean_deg': round(math.degrees(float(np.mean(ori))), 3),
            'n_window_samples': len(task.samples),
            'sample_rate_hz': round(eff_rate, 2),
            'n_tf_fail': task.n_tf_fail,
            'peak_effort_nm': {k: round(v, 3) for k, v in task.peak_effort.items()},
            'pos_tol_mm': self.pos_tol * 1000.0,
            'ori_tol_deg': math.degrees(self.ori_tol),
            'dwell_s': self.dwell,
        }
        # §A7.3/§A7.4: a run that fails these is an INSTRUMENT failure, not a task
        # result, and must be discarded rather than quietly averaged in.
        bad = []
        if eff_rate < self.min_rate:
            bad.append(f'sample rate {eff_rate:.1f} Hz < {self.min_rate:.0f} Hz')
        if task.n_tf_fail:
            bad.append(f'{task.n_tf_fail} TF lookup failures')
        task.result['instrument_ok'] = not bad
        if bad:
            task.result['instrument_fault'] = '; '.join(bad)
            self.get_logger().error(
                f'{task.arm}: dwell held BUT RUN IS INVALID (§A7) -- '
                f'{task.result["instrument_fault"]}. Discard this run.')
        self.get_logger().info(
            f'>>> {task.arm}: SUCCESS -- held {self.dwell:.1f}s, '
            f'pos max {task.result["pos_err_max_mm"]:.2f}mm '
            f'(tol {self.pos_tol * 1000:.1f}), ori max '
            f'{task.result["ori_err_max_deg"]:.2f}deg '
            f'(tol {math.degrees(self.ori_tol):.1f}), '
            f'{len(task.samples)} samples @ {eff_rate:.1f} Hz')
        self._publish(task.result)
        self._write_summary(task.result)

    # ---------------------------------------------------------------- outputs

    def _publish(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _open_csv(self):
        base = self._csv_base[:-4] if self._csv_base.endswith('.csv') else self._csv_base
        d = os.path.dirname(base)
        if d:
            os.makedirs(d, exist_ok=True)
        self._samples_path = base + '_samples.csv'
        self._summary_path = base + '_summary.csv'
        self._samples_csv = open(self._samples_path, 'w', newline='')
        self._samples_w = csv.writer(self._samples_csv)
        self._samples_w.writerow(
            ['t', 'arm', 'pos_err_mm', 'ori_err_deg', 'in_tol',
             'pos_tol_mm', 'ori_tol_deg'])
        self.get_logger().info(
            f'logging samples -> {self._samples_path}, summary -> {self._summary_path}')

    def _write_sample(self, t, task, pos_err, ori_err, in_tol):
        if self._samples_w is None:
            return
        self._samples_w.writerow(
            [f'{t:.4f}', task.arm, f'{pos_err * 1000.0:.4f}',
             f'{math.degrees(ori_err):.4f}', int(in_tol),
             self.pos_tol * 1000.0, math.degrees(self.ori_tol)])
        self._samples_csv.flush()

    def _write_summary(self, result):
        if not self._csv_base:
            return
        new = not os.path.exists(self._summary_path)
        with open(self._summary_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(list(result.keys()))
            w.writerow([json.dumps(v) if isinstance(v, (dict, list)) else v
                        for v in result.values()])

    def destroy_node(self):
        if self._samples_csv is not None:
            self._samples_csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ReachDwellMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
