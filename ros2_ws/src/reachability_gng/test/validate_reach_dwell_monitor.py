"""Validate reach_dwell_monitor against a SYNTHETIC tool frame with known error.

The monitor scores the paper's headline success numbers, so it gets checked
against ground truth it cannot influence: we drive the TF ourselves, so the true
position/orientation error is known exactly, and we assert the monitor reports it.

Cases:
  A  exactly on target            -> SUCCESS, pos_err ~0
  B  6 mm off (tol is 5 mm)       -> NO success
  C  in tol, one 1-sample excursion at t=1.0 -> window RESETS (success late)
  D  6 deg off in approach axis   -> NO success (position alone is not enough)
  E  two arms, staggered entry    -> concurrent window starts at the LATER arm
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

TARGET = (1.0, 0.5, 1.2)


class Driver(Node):
    def __init__(self):
        super().__init__('validate_driver')
        self.br = TransformBroadcaster(self)
        self.pub = {a: self.create_publisher(PoseStamped, f'/reach_dwell/target/{a}', 10)
                    for a in ('arm_1', 'arm_2')}
        self.clear = self.create_publisher(String, '/reach_dwell/clear', 10)
        self.events = []
        self.create_subscription(
            String, '/reach_dwell/status',
            lambda m: self.events.append(dict(json.loads(m.data), _t=time.time())), 10)
        self.offset = {'t1_a1_tool_frame': (0.0, 0.0, 0.0),
                       't1_a2_tool_frame': (0.0, 0.0, 0.0)}
        self.quat = {'t1_a1_tool_frame': (0.0, 0.0, 0.0, 1.0),
                     't1_a2_tool_frame': (0.0, 0.0, 0.0, 1.0)}
        self.live = set()
        self.create_timer(0.02, self._tf)

    def _tf(self):
        now = self.get_clock().now().to_msg()
        for frame in self.live:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = frame
            o = self.offset[frame]
            t.transform.translation.x = TARGET[0] + o[0]
            t.transform.translation.y = TARGET[1] + o[1]
            t.transform.translation.z = TARGET[2] + o[2]
            q = self.quat[frame]
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            self.br.sendTransform(t)

    def set_target(self, arm, quat=(0.0, 0.0, 0.0, 1.0)):
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = TARGET
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        self.pub[arm].publish(m)

    def reset(self):
        self.events.clear()
        m = String()
        m.data = 'all'
        self.clear.publish(m)


def spin(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def successes(d, arm=None):
    return [e for e in d.events if e.get('event') == 'success'
            and (arm is None or e['arm'] == arm)]


def main():
    mon = subprocess.Popen(
        ['ros2', 'run', 'reachability_gng', 'reach_dwell_monitor'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    d = Driver()
    fails = []
    try:
        spin(d, 2.0)   # let the monitor come up and discover topics

        # ---- A: exactly on target -------------------------------------
        d.reset()
        d.live = {'t1_a1_tool_frame'}
        d.offset['t1_a1_tool_frame'] = (0.0, 0.0, 0.0)
        spin(d, 0.5)
        d.set_target('arm_1')
        spin(d, 3.5)
        s = successes(d, 'arm_1')
        if not s:
            fails.append('A: on-target did NOT succeed')
        elif s[0]['pos_err_max_mm'] > 0.1:
            fails.append(f'A: pos_err {s[0]["pos_err_max_mm"]} mm, expected ~0')
        else:
            print(f'A PASS  on-target -> SUCCESS, pos_err_max '
                  f'{s[0]["pos_err_max_mm"]} mm, {s[0]["n_window_samples"]} samples '
                  f'@ {s[0]["sample_rate_hz"]} Hz, instrument_ok={s[0]["instrument_ok"]}')

        # ---- B: 6 mm off, tol is 5 mm ---------------------------------
        d.reset()
        d.offset['t1_a1_tool_frame'] = (0.006, 0.0, 0.0)
        spin(d, 0.5)
        d.set_target('arm_1')
        spin(d, 3.5)
        if successes(d, 'arm_1'):
            fails.append('B: 6 mm off was accepted -- threshold not enforced')
        else:
            print('B PASS  6 mm off (tol 5) -> correctly NO success')

        # ---- C: continuity -- one bad sample must reset the window -----
        d.reset()
        d.offset['t1_a1_tool_frame'] = (0.0, 0.0, 0.0)
        spin(d, 0.5)
        d.set_target('arm_1')
        t0 = time.time()
        spin(d, 1.0)
        d.offset['t1_a1_tool_frame'] = (0.05, 0.0, 0.0)   # brief 50 mm excursion
        spin(d, 0.1)
        d.offset['t1_a1_tool_frame'] = (0.0, 0.0, 0.0)
        spin(d, 3.0)
        s = successes(d, 'arm_1')
        if not s:
            fails.append('C: never succeeded after the excursion')
        else:
            held = s[0]['time_to_dwell_s']
            # Without a reset it would have finished ~2.0 s after the target was
            # set; the excursion at 1.0 s must push it to ~3.1 s.
            if held < 2.9:
                fails.append(f'C: succeeded at {held}s -- window did NOT reset '
                             '(an averaging criterion would do this)')
            else:
                print(f'C PASS  excursion at 1.0s reset the window: dwell '
                      f'completed at {held}s, not ~2.0s')

        # ---- D: orientation alone must be able to fail the task --------
        d.reset()
        d.offset['t1_a1_tool_frame'] = (0.0, 0.0, 0.0)
        # rotate the tool 6 deg about X: position perfect, approach axis 6 deg off
        a = math.radians(6.0) / 2.0
        d.quat['t1_a1_tool_frame'] = (math.sin(a), 0.0, 0.0, math.cos(a))
        spin(d, 0.5)
        d.set_target('arm_1')          # commanded orientation = identity
        spin(d, 3.5)
        if successes(d, 'arm_1'):
            fails.append('D: 6 deg approach error accepted -- orientation not scored')
        else:
            print('D PASS  6 deg approach error (tol 5) -> correctly NO success')
        d.quat['t1_a1_tool_frame'] = (0.0, 0.0, 0.0, 1.0)

        # ---- E: concurrency uses the COMMON window --------------------
        d.reset()
        d.live = {'t1_a1_tool_frame', 't1_a2_tool_frame'}
        d.offset['t1_a1_tool_frame'] = (0.0, 0.0, 0.0)
        d.offset['t1_a2_tool_frame'] = (0.05, 0.0, 0.0)   # arm_2 starts OUT of tol
        spin(d, 0.5)
        d.set_target('arm_1')
        d.set_target('arm_2')
        spin(d, 1.5)
        d.offset['t1_a2_tool_frame'] = (0.0, 0.0, 0.0)    # arm_2 arrives late
        t_late = time.time()
        spin(d, 3.5)
        conc = [e for e in d.events if e.get('event') == 'concurrent']
        if not conc:
            fails.append('E: no concurrent event')
        elif conc[0]['n_arms'] != 2:
            fails.append(f'E: n_arms={conc[0]["n_arms"]}, expected 2')
        else:
            # The COMMON window must start when the LATE arm entered tolerance,
            # not when the early one did. arm_1 was in tolerance ~1.5 s before
            # arm_2, so a window anchored on arm_1 would fire ~1.5 s too early --
            # and would credit concurrency that never happened.
            lag = conc[0]['_t'] - t_late
            if lag < 1.9:
                fails.append(f'E: concurrent fired {lag:.2f}s after the late arm '
                             'arrived -- window anchored on the EARLY arm')
            else:
                print(f'E PASS  2-arm common window: fired {lag:.2f}s after the '
                      f'late arm entered tolerance (must be >=2.0s), '
                      f'arms={conc[0]["arms"]}')
    finally:
        d.destroy_node()
        rclpy.try_shutdown()
        mon.terminate()
        mon.wait(timeout=5)

    print()
    if fails:
        print('FAILURES:')
        for f in fails:
            print('  -', f)
        return 1
    print('ALL CASES PASS -- instrument validated against known synthetic error')
    return 0


if __name__ == '__main__':
    sys.exit(main())
