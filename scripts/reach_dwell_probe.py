#!/usr/bin/env python3
"""p1_state.md 8c STEP 2: one arm, reach-and-dwell to a PERCEIVED target.

docs/p1_g16_hw.md A8. This is the COMMANDER. It is deliberately a separate
process from reach_dwell_monitor, which is the SCORER, because whatever scores
success must not also choose the candidates -- otherwise the criterion drifts
toward the result. The monitor commands nothing; this commands and scores
nothing.

The contract between them, and the reason the ordering matters:

    1. perceive          -> p_perceived        (from /target_object, or --target)
    2. choose            -> p_cmd              (perceived + approach offset)
    3. PUBLISH p_cmd to /reach_dwell/target/<arm>   <-- BEFORE any motion
    4. command MoveIt to p_cmd
    5. the monitor independently scores  ||p_tool - p_cmd|| and the 2.0 s dwell

Step 3 must precede step 4 so the monitor scores the pose that was actually
COMMANDED. Publishing the PERCEIVED pose instead would silently fold perception
error (L3, 3-5 cm) into the execution criterion (L2, < 5 mm) and make the
success number meaningless -- that is exactly the confusion p1_g4 A2 exists to
prevent. L3 is reported by this script, separately, and is NOT part of success.

SAFETY (docs/p1_g16_hw.md A6). Read-only unless --move is typed on purpose:
  * default is a DRY RUN: it perceives, computes p_cmd, publishes the target,
    and prints the plan -- but sends no trajectory.
  * it REFUSES to command the tuck pose [0, 2.6, 2.6, 0, 0, 0]; that draws
    12.78 of 14 N.m and is a fake-system artefact, not a real rest pose.
  * it aborts on any joint effort above --tau-max.
  * the rest pose is HANGING.

NOTE ON THE PERCEPTION SOURCE. /detected_object_pose does NOT exist in this
checkout: it comes from lidar_filter.py, which needs livox_ros_driver2, and that
source tree is empty (there is no .gitmodules). What exists is the RGBD chain
ending in /target_object. And because the locked criterion scores COMMANDED ->
ACHIEVED, a hand-given --target measures exactly the same thing with the cameras
and the segmentation model taken out of the failure surface -- which is what day
one should do, so that a failure can actually be attributed.

    # day one: no perception at all, measures the same L2
    python3 scripts/reach_dwell_probe.py --arm arm_1 --target 0.9,0.3,1.2
    # once the RGBD chain is up
    python3 scripts/reach_dwell_probe.py --arm arm_1 --trials 10 --move
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

TOOL_FRAME = {'arm_1': 't1_a1_tool_frame', 'arm_2': 't1_a2_tool_frame',
              'arm_3': 't2_a1_tool_frame', 'arm_4': 't2_a2_tool_frame'}
JOINT_PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_',
                'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
# NOT /detected_object_pose: that is published by workcell_description's
# lidar_filter.py, which needs livox_ros_driver2 -- and that source tree is
# EMPTY in this checkout (there is no .gitmodules either), so the LIDAR driver
# cannot run and the topic never appears. The RGBD chain is what exists:
#   rgbd_perception.launch.py -> instance segmentation -> object_localizer
#                             -> /target_object
PERCEPT_TOPIC = '/target_object'
# A6/S2. Never commanded. Kept here so the refusal is explicit and greppable.
FORBIDDEN_TUCK = [0.0, 2.6, 2.6, 0.0, 0.0, 0.0]
# Per-joint effort ratings from Kinova's own gen3_lite_macro.xacro. joint_2's 14
# is the one that binds when reaching outward from the ceiling; the wrist joints
# never came near their 7 in G16 (peak 0.6).
JOINT_EFFORT_LIMIT = {1: 10.0, 2: 14.0, 3: 10.0, 4: 7.0, 5: 7.0, 6: 7.0}
# RNEA underestimates measured peak effort by a roughly CONSTANT amount --
# unmodelled joint friction and strain-wave gear loss, plus Kortex reporting
# motor-current-derived effort rather than pure joint torque. Calibrated on 7
# pairs from G16 (docs/p1_g16_hw.md B5.4): mean +6.59, sd 1.86, and the offset
# does not scale with the predicted value.
#
# Screening on (predicted + offset) against the ratings above reproduces the
# real safe/unsafe split on 6 of those 7 -- the miss is trial 3, predicted
# 7.60 -> 14.20 so REFUSED, while it actually measured 10.86 and was safe. That
# error is a false REFUSAL, i.e. it errs toward not moving, which is the correct
# direction for a safety screen. A raw-prediction threshold would separate all
# 7, but only inside a 0.17 N.m gap (7.60..7.77) -- far too thin to trust.
TORQUE_MODEL_OFFSET_NM = 6.6


class Probe(Node):
    def __init__(self, arm, approach, tau_max, move, topic=PERCEPT_TOPIC,
                 fixed=None):
        super().__init__('reach_dwell_probe')
        self.arm, self.approach, self.tau_max, self.move = \
            arm, approach, tau_max, move
        self.fixed = fixed
        self.perceived = None
        self.status = []
        self.tau_peak = 0.0
        self.tau_worst_joint = ''

        self.create_subscription(PoseStamped, topic, self._on_percept, 10)
        self.create_subscription(String, '/reach_dwell/status',
                                 self._on_status, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.target_pub = self.create_publisher(
            PoseStamped, f'/reach_dwell/target/{arm}', 10)
        self.clear_pub = self.create_publisher(String, '/reach_dwell/clear', 10)

    # ------------------------------------------------------------- inputs
    def _on_percept(self, msg):
        self.perceived = msg

    def _on_status(self, msg):
        try:
            self.status.append(json.loads(msg.data))
        except (ValueError, TypeError):
            self.status.append({'raw': msg.data})

    def _on_js(self, msg):
        # A6/S3. Real N.m at ~96 Hz. Once ARMSTATE_IN_FAULT latches, the red LED
        # needs a PHYSICAL reset -- fault_controller is not spawned -- so the
        # only useful guard is one that trips below the firmware threshold.
        for name, eff in zip(msg.name, msg.effort or []):
            if abs(eff) > self.tau_peak:
                self.tau_peak, self.tau_worst_joint = abs(eff), name

    # ------------------------------------------------------------- helpers
    def wait_percept(self, timeout):
        """A fixed target is a legitimate source, and on day one the better one.

        The locked success criterion (A1) scores COMMANDED -> ACHIEVED (L2).
        Perception error (L3) is explicitly NOT part of it. So where p_cmd comes
        from does not change what is being measured -- it only changes which
        pose is measured. Running L2 against a hand-given pose therefore
        measures exactly the same quantity while removing the cameras, the
        segmentation model and the extrinsics from the failure surface. Couple
        them on day one and a failure cannot be attributed.
        """
        if self.fixed is not None:
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = self.fixed
            p.pose.orientation.w = 1.0
            return p
        t0 = time.time()
        while time.time() - t0 < timeout and self.perceived is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.perceived

    def command_pose(self, p):
        """p_cmd = perceived + a straight-up approach offset."""
        out = PoseStamped()
        out.header.frame_id = 'world'          # the monitor rejects other frames
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = p.pose.position.x
        out.pose.position.y = p.pose.position.y
        out.pose.position.z = p.pose.position.z + self.approach
        # Approach axis pointing down; roll is free by the locked definition.
        out.pose.orientation.x = 1.0
        out.pose.orientation.y = 0.0
        out.pose.orientation.z = 0.0
        out.pose.orientation.w = 0.0
        return out

    def _await_match(self, pub, timeout):
        """Block until DDS has actually matched a subscriber to `pub`.

        Publishing into a publisher DDS has not yet matched is dropped SILENTLY.
        Measured in G16: the old 0.1 s of spinning lost 10 of 11 publishes, and
        the monitor went on scoring against the target it still held -- which
        produces plausible-looking numbers rather than an obvious break. That is
        far more dangerous than a loud failure, so this waits and then reports
        whether it succeeded instead of assuming.
        """
        t0 = time.time()
        while pub.get_subscription_count() == 0 and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return pub.get_subscription_count() > 0

    def publish_target(self, p_cmd, timeout=15.0):
        """MUST happen before motion -- see the module docstring."""
        if not self._await_match(self.target_pub, timeout):
            return False
        self.target_pub.publish(p_cmd)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def clear(self, timeout=15.0):
        if not self._await_match(self.clear_pub, timeout):
            return False
        self.clear_pub.publish(String(data=self.arm))
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def collect(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tau_peak > self.tau_max:
                self.get_logger().error(
                    f'ABORT: torsi {self.tau_peak:.2f} N.m pada '
                    f'{self.tau_worst_joint} > batas {self.tau_max:.2f}')
                return 'TORQUE-ABORT'
            for s in self.status:
                if s.get('arm') == self.arm and s.get('event') == 'success':
                    return 'SUCCESS'
        return None

    def verdict(self, raw, mv=None, reached=None):
        """A3's THREE-WAY split. Never collapse these into one bucket.

        NO-PLAN now comes from MoveIt's own answer (`mv`), not from "the monitor
        never saw tolerance". Those are different claims, and A3 defines NO-PLAN
        as the first one. Inferring it from the second would file every
        executed-but-inaccurate reach as a reachability failure and blame the
        capability map for what is actually tracking error.

        A6/S2 refusals and execution faults are reported under their own names
        rather than being folded into a task verdict -- they are machine events
        in the A5 sense, and A5 says those are not method failures.
        """
        if raw in ('SUCCESS', 'TORQUE-ABORT'):
            return raw
        if mv in ('NO-PLAN', 'TUCK-REFUSED', 'EXEC-FAIL',
                  'TORQUE-UNSAFE', 'TORQUE-UNSCREENED'):
            return mv
        if reached is None:
            reached = any(s.get('arm') == self.arm and s.get('in_tol')
                          for s in self.status)
        # See docs/p1_g16_hw.md B3: A3 assumed the only failures were NO-PLAN and
        # REACHED-NOT-HELD. Real hardware has a third -- planned, executed, and
        # settled OUTSIDE the 5 mm bar, never touching tolerance at all. Folding
        # that into either existing bucket would misattribute it, so it is
        # reported under its own name.
        return 'REACHED-NOT-HELD' if reached else 'EXEC-MISS'


def _rowcount(path):
    """Where the monitor's sample log stands right now.

    The monitor publishes on /reach_dwell/status only when a dwell SUCCEEDS --
    it emits no per-sample messages -- so `in_tol` never appears on the topic
    and REACHED-NOT-HELD cannot be detected from it at all. The per-sample data
    does exist, flushed every sample to <csv_log>_samples.csv. Reading that file
    keeps the scorer untouched: this reads an artefact the monitor already
    writes, and still commands and scores nothing.
    """
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        return sum(1 for _ in f)


def _reached_since(path, arm, start_row):
    """Did any sample this arm produced since `start_row` land inside tolerance?

    Row offsets rather than timestamps, so no clock has to be reconciled
    between the two processes.
    """
    if start_row is None or not path or not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.reader(f))
    for row in rows[max(start_row, 1):]:
        if len(row) >= 5 and row[1] == arm and row[4] == '1':
            return True
    return False


def _goal_constraints(link, p_cmd, pos_tol, ori_tol_deg):
    """A pose goal on the tool frame, with roll about the tool axis left FREE.

    The locked definition (p1_g4 A1) constrains the approach axis and explicitly
    not roll, so the z-axis tolerance is opened to a full turn rather than
    quietly constraining something the criterion does not score.

    Planner tolerance is deliberately TIGHTER than the 5 mm / 5 deg scoring bar:
    the plan should aim well inside the bar, so that what the run measures is
    execution tracking rather than how close the goal region let the planner
    stop. Loosening these to make trials pass would be moving the bar.
    """
    c = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = 'world'
    pc.link_name = link
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [pos_tol]
    bv = BoundingVolume()
    bv.primitives = [sphere]
    bv.primitive_poses = [p_cmd.pose]
    pc.constraint_region = bv
    pc.weight = 1.0
    c.position_constraints.append(pc)

    oc = OrientationConstraint()
    oc.header.frame_id = 'world'
    oc.link_name = link
    oc.orientation = p_cmd.pose.orientation
    oc.absolute_x_axis_tolerance = math.radians(ori_tol_deg)
    oc.absolute_y_axis_tolerance = math.radians(ori_tol_deg)
    oc.absolute_z_axis_tolerance = 2.0 * math.pi        # roll FREE, by A1
    oc.weight = 1.0
    c.orientation_constraints.append(oc)
    return c


def _violates_tuck(traj, arm, tol_deg=10.0):
    """A6/S2, enforced where the joint command actually EXISTS.

    A pose goal never names joints, so there is nothing to check until MoveIt
    has produced a trajectory. That is why this runs between plan and execute
    rather than before planning: planning is free, executing is not. Every
    waypoint is checked, not just the last -- the tuck being dangerous partway
    through is exactly as bad as ending there.
    """
    jt = traj.joint_trajectory
    order = [f'{JOINT_PREFIX[arm]}joint_{i}' for i in range(1, 7)]
    try:
        idx = [jt.joint_names.index(n) for n in order]
    except ValueError:
        return None            # not the 6 arm joints -- caller decides
    for k, pt in enumerate(jt.points):
        v = [pt.positions[i] for i in idx]
        if max(abs(a - b) for a, b in zip(v, FORBIDDEN_TUCK)) < math.radians(tol_deg):
            return k
    return -1


def _urdf_path(node, cache='/tmp/reach_dwell_live.urdf'):
    """The URDF the running system is actually using, not a file on disk."""
    if os.path.exists(cache):
        return cache
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters,
                             '/robot_state_publisher/get_parameters')
    if not cli.wait_for_service(timeout_sec=15.0):
        return None
    req = GetParameters.Request()
    req.names = ['robot_description']
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    if f.result() is None or not f.result().values:
        return None
    with open(cache, 'w') as fh:
        fh.write(f.result().values[0].string_value)
    return cache


def predict_peak_torque(traj, arm, node):
    """Peak |tau| each arm joint will see, per waypoint, BEFORE anything moves.

    Full RNEA -- not just gravity -- because the planned trajectory carries
    velocities and accelerations, and G16 measured a 17.61 N.m transient whose
    static component was only ~9. Screening on gravity alone would have passed
    exactly the motion that needed screening.

    Validated against hardware 2026-08-17: at the pose where the arm was
    measured holding 9.597 N.m on joint_2, this predicts 9.253 -- 3.6 % low.

    Returns {joint: peak_abs_Nm} or None if the model cannot be built, in which
    case the caller must REFUSE to move rather than proceed unscreened.
    """
    try:
        import numpy as np
        import pinocchio as pin
    except ImportError:
        return None
    path = _urdf_path(node)
    if not path:
        return None
    if not hasattr(node, '_pin'):
        m = pin.buildModelFromUrdf(path)
        node._pin = (m, m.createData())
    m, d = node._pin

    jt = traj.joint_trajectory
    names = [n for n in jt.joint_names if n.startswith(JOINT_PREFIX[arm])]
    try:
        idx = {n: (m.joints[m.getJointId(n)].idx_q,
                   m.joints[m.getJointId(n)].idx_v) for n in names}
    except Exception:                                        # noqa: BLE001
        return None
    col = {n: jt.joint_names.index(n) for n in names}

    peak = {n: 0.0 for n in names}
    for pt in jt.points:
        q = pin.neutral(m)
        v = np.zeros(m.nv)
        a = np.zeros(m.nv)
        for n in names:
            iq, iv = idx[n]
            k = col[n]
            q[iq] = pt.positions[k]
            if pt.velocities:
                v[iv] = pt.velocities[k]
            if pt.accelerations:
                a[iv] = pt.accelerations[k]
        tau = pin.rnea(m, d, q, v, a)
        for n in names:
            peak[n] = max(peak[n], abs(tau[idx[n][1]]))
    return peak


def move_to(arm, p_cmd, node, pos_tol=0.002, ori_tol_deg=2.0,
            vel_scale=0.15, plan_time=15.0, plan_only=False, tau_max=None):
    """Plan to p_cmd, CHECK the plan against A6/S2, then execute it.

    Reuses the MoveIt path hardware_check.py --arms drives (MoveGroup action,
    15 % velocity scaling); no new motion driver is written. The one departure
    is that this plans and executes in two steps instead of one, and the reason
    is A6/S2: a pose goal names no joints, so the tuck refusal has nothing to
    inspect until a trajectory exists. plan_only gives us that trajectory while
    the arm is still stationary.

    Returns one of 'NO-PLAN' | 'TUCK-REFUSED' | 'MOVED' | 'EXEC-FAIL'. The
    NO-PLAN verdict comes from MoveIt itself rather than being inferred from
    "the monitor never saw tolerance", which is what A3 needs it to mean.
    """
    if not hasattr(node, '_plan_ac'):
        node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
        node._exec_ac = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
    if not node._plan_ac.wait_for_server(timeout_sec=15.0):
        return 'EXEC-FAIL'

    goal = MoveGroup.Goal()
    goal.request.group_name = arm
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = plan_time
    goal.request.max_velocity_scaling_factor = vel_scale
    goal.request.max_acceleration_scaling_factor = vel_scale
    goal.request.goal_constraints.append(
        _goal_constraints(TOOL_FRAME[arm], p_cmd, pos_tol, ori_tol_deg))
    goal.planning_options.plan_only = True

    fut = node._plan_ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return 'NO-PLAN'
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=plan_time + 20.0)
    res = rf.result()
    if res is None or res.result.error_code.val != MoveItErrorCodes.SUCCESS:
        return 'NO-PLAN'

    traj = res.result.planned_trajectory
    if not traj.joint_trajectory.points:
        return 'NO-PLAN'

    hit = _violates_tuck(traj, arm)
    if hit is not None and hit >= 0:
        node.get_logger().error(
            f'A6/S2: rencana DITOLAK -- titik {hit} berada dalam 10 deg dari '
            f'FORBIDDEN_TUCK {FORBIDDEN_TUCK}. TIDAK dieksekusi.')
        return 'TUCK-REFUSED'

    # A6/S3, enforced as PREVENTION rather than detection. The --tau-max guard
    # in collect() only WATCHES: by the time it fires the trajectory has already
    # run and the torque has already been applied -- that is how trial 4 reached
    # 17.61 N.m on a joint rated 14. Screening the plan is the only point at
    # which an over-torque motion can still be refused instead of merely noticed.
    if tau_max is not None:
        peak = predict_peak_torque(traj, arm, node)
        if peak is None:
            node.get_logger().error(
                'TIDAK BISA memprediksi torsi (pinocchio/URDF tidak tersedia) '
                '-- MENOLAK bergerak tanpa penyaringan. Ini disengaja.')
            return 'TORQUE-UNSCREENED'
        # Compare each joint against ITS OWN rating, corrected by the measured
        # model offset -- not against one global number. joint_2 is rated 14 and
        # the wrist only 7, so a single bar would be simultaneously too loose
        # for the wrist and too tight for the shoulder.
        worst, worst_frac = None, 0.0
        parts = []
        for k, v in sorted(peak.items()):
            j = int(k.split('joint_')[-1])
            lim = min(JOINT_EFFORT_LIMIT.get(j, 7.0), tau_max) \
                if j != 2 else min(JOINT_EFFORT_LIMIT[2], max(tau_max, 14.0))
            corr = v + TORQUE_MODEL_OFFSET_NM
            parts.append(f'{j}={v:.2f}->{corr:.2f}/{lim:.0f}')
            if corr / lim > worst_frac:
                worst, worst_frac, worst_corr, worst_lim = k, corr / lim, corr, lim
        node.get_logger().info(
            f'torsi RNEA+{TORQUE_MODEL_OFFSET_NM} vs rating: ' + ', '.join(parts)
            + f'  (terketat {worst} {worst_frac * 100:.0f} %)')
        if worst_frac > 1.0:
            node.get_logger().error(
                f'A6/S3: rencana DITOLAK SEBELUM GERAK -- {worst} diperkirakan '
                f'{worst_corr:.2f} N.m > rating {worst_lim:.0f}. Pose ini '
                'menuntut torsi di atas rating sendi; itu sifat POSE, bukan '
                'sifat penjaga. (Penyaring sengaja konservatif: pada kalibrasi '
                'G16 ia menolak 1 dari 4 pose yang ternyata aman.)')
            return 'TORQUE-UNSAFE'

    # A6/S7: --move not typed means the arm does not move, but the plan was
    # still worth asking for -- it separates NO-PLAN from reachable-but-untried
    # without commanding anything.
    if plan_only:
        return 'PLANNED'

    if not node._exec_ac.wait_for_server(timeout_sec=10.0):
        return 'EXEC-FAIL'
    eg = ExecuteTrajectory.Goal()
    eg.trajectory = traj
    ef = node._exec_ac.send_goal_async(eg)
    rclpy.spin_until_future_complete(node, ef, timeout_sec=20.0)
    egh = ef.result()
    if egh is None or not egh.accepted:
        return 'EXEC-FAIL'
    erf = egh.get_result_async()
    rclpy.spin_until_future_complete(node, erf, timeout_sec=90.0)
    eres = erf.result()
    if eres is None or eres.result.error_code.val != MoveItErrorCodes.SUCCESS:
        return 'EXEC-FAIL'
    return 'MOVED'


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', default='arm_1', choices=sorted(TOOL_FRAME))
    ap.add_argument('--trials', type=int, default=10,
                    help='A2: dikunci 10; ubah hanya kalau A2 diubah dulu')
    ap.add_argument('--approach', type=float, default=0.10,
                    help='offset approach di atas objek terpersepsi (m)')
    ap.add_argument('--tau-max', type=float, default=10.0,
                    help='A6/S3 batas torsi abort (N.m); tuck = 12.78/14')
    ap.add_argument('--settle', type=float, default=8.0,
                    help='detik menunggu monitor per percobaan')
    ap.add_argument('--topic', default=PERCEPT_TOPIC,
                    help=f'topik pose terpersepsi (default {PERCEPT_TOPIC}; '
                         '/detected_object_pose BUTUH livox yang TIDAK ADA)')
    ap.add_argument('--target', metavar='X,Y,Z',
                    help='pose tetap di frame world, lewati persepsi sama '
                         'sekali. Mengukur L2 yang SAMA; pakai ini hari pertama')
    ap.add_argument('--move', action='store_true',
                    help='A6/S7: BENAR-BENAR menggerakkan lengan. '
                         'Tanpa ini skrip DRY RUN.')
    ap.add_argument('--plan-check', action='store_true',
                    help='saat DRY RUN, tetap MINTA rencana ke MoveIt (tanpa '
                         'mengeksekusi) supaya NO-PLAN terpisah dari '
                         'terjangkau-tapi-belum-dicoba')
    ap.add_argument('--monitor-csv', default='',
                    help='<csv_log>_samples.csv milik reach_dwell_monitor. '
                         'Dibutuhkan untuk membedakan REACHED-NOT-HELD dari '
                         'EXEC-MISS: monitor TIDAK menerbitkan status per-sampel')
    a = ap.parse_args()

    fixed = None
    if a.target:
        try:
            fixed = tuple(float(v) for v in a.target.split(','))
            if len(fixed) != 3:
                raise ValueError
        except ValueError:
            ap.error('--target harus X,Y,Z (meter, frame world)')

    if not a.move:
        print('\033[33mDRY RUN\033[0m -- tidak ada gerak. '
              'Tambahkan --move untuk menggerakkan (A6/S7).\n')

    rclpy.init()
    node = Probe(a.arm, a.approach, a.tau_max, a.move, a.topic, fixed)
    src = f'TETAP {fixed} (persepsi dilewati)' if fixed else f'topik {a.topic}'
    print(f'probe: {a.arm} ({TOOL_FRAME[a.arm]}), {a.trials} percobaan, '
          f'approach {a.approach*1000:.0f} mm, abort torsi {a.tau_max} N.m')
    print(f'sumber target: {src}')
    print('kriteria A1 TERKUNCI: pos < 5 mm, ori < 5 deg, dwell 2.0 s KONTINU\n')

    tally = {'SUCCESS': 0, 'REACHED-NOT-HELD': 0, 'NO-PLAN': 0,
             'TORQUE-ABORT': 0, 'INVALID': 0}
    rows = []
    try:
        for i in range(1, a.trials + 1):
            node.status.clear()
            node.clear()
            p = node.wait_percept(timeout=10.0)
            if p is None:
                print(f'  [{i:2d}] TIDAK VALID (mesin): tidak ada pose di '
                      f'{a.topic} -- A5, diulang, tidak masuk penyebut. '
                      'Pakai --target X,Y,Z untuk melewati persepsi.')
                tally['INVALID'] += 1
                continue

            p_cmd = node.command_pose(p)
            # BEFORE motion. Always -- and VERIFIED, not assumed: an unmatched
            # publish is dropped silently and the monitor would then score this
            # trial against a stale target (docs/p1_g16_hw.md B3.4 conflict 6).
            if not node.publish_target(p_cmd):
                print(f'  [{i:2d}] TIDAK VALID (mesin): tidak ada pelanggan di '
                      f'/reach_dwell/target/{a.arm} -- reach_dwell_monitor '
                      'tidak jalan? A5, diulang, tidak masuk penyebut.')
                tally['INVALID'] += 1
                continue

            if not a.move:
                mv = move_to(a.arm, p_cmd, node, plan_only=True,
                             tau_max=a.tau_max) if a.plan_check else 'DRY'
                print(f'  [{i:2d}] DRY RUN  p_cmd = '
                      f'({p_cmd.pose.position.x:+.3f}, '
                      f'{p_cmd.pose.position.y:+.3f}, '
                      f'{p_cmd.pose.position.z:+.3f}) world'
                      + (f'   rencana: {mv}' if a.plan_check else ''))
                if a.plan_check:
                    rows.append(dict(trial=i, verdict=f'DRY:{mv}'))
                continue

            row0 = _rowcount(a.monitor_csv)
            node.tau_peak, node.tau_worst_joint = 0.0, ''   # per-trial peak
            mv = move_to(a.arm, p_cmd, node, tau_max=a.tau_max)
            raw = node.collect(a.settle) if mv == 'MOVED' else None

            v = node.verdict(raw, mv, _reached_since(a.monitor_csv, a.arm, row0))
            tally[v] = tally.get(v, 0) + 1
            rows.append(dict(trial=i, verdict=v, move_status=mv,
                             tau_peak=round(node.tau_peak, 3),
                             target=[round(p_cmd.pose.position.x, 4),
                                     round(p_cmd.pose.position.y, 4),
                                     round(p_cmd.pose.position.z, 4)]))
            print(f'  [{i:2d}] {v:17s} (moveit: {mv:12s}) '
                  f'torsi puncak {node.tau_peak:5.2f} N.m')
            if v == 'TORQUE-ABORT':
                print('\033[31m  BERHENTI: batas torsi. A6/S4 -- red LED butuh '
                      'reset FISIK.\033[0m')
                break
    except KeyboardInterrupt:
        print('\ndihentikan pengguna')
    finally:
        # A5: machine-mode events (TUCK-REFUSED / EXEC-FAIL / TORQUE-ABORT /
        # INVALID) are NOT method failures and stay OUT of the denominator.
        valid = sum(tally.get(k, 0) for k in
                    ('SUCCESS', 'REACHED-NOT-HELD', 'EXEC-MISS', 'NO-PLAN'))
        if valid:
            print(f'\n  A2: {tally["SUCCESS"]} / {valid} sukses  '
                  f'(LULUS butuh >= 8/10)')
            print(f'  A3: SUCCESS {tally["SUCCESS"]}, '
                  f'REACHED-NOT-HELD {tally["REACHED-NOT-HELD"]}, '
                  f'EXEC-MISS {tally.get("EXEC-MISS", 0)}, '
                  f'NO-PLAN {tally["NO-PLAN"]}')
            mesin = {k: v for k, v in tally.items()
                     if k in ('TUCK-REFUSED', 'EXEC-FAIL', 'TORQUE-ABORT',
                              'TORQUE-UNSAFE', 'TORQUE-UNSCREENED',
                              'INVALID') and v}
            print(f'  TIDAK VALID (mesin, A5, diulang, di LUAR penyebut): '
                  f'{mesin or "tidak ada"}')
        if rows:
            json.dump(rows, open('/tmp/g16_step2.json', 'w'), indent=1)
            print('  -> /tmp/g16_step2.json')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
