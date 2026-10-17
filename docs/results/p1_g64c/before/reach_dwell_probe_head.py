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
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint,
                             PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

TOOL_FRAME = {'arm_1': 't1_a1_tool_frame', 'arm_2': 't1_a2_tool_frame',
              'arm_3': 't2_a1_tool_frame', 'arm_4': 't2_a2_tool_frame'}
JOINT_PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_',
                'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
GANTRY_OF = {'arm_1': 'gantry_1', 'arm_2': 'gantry_1',
             'arm_3': 'gantry_2', 'arm_4': 'gantry_2'}
RAIL_JOINT = {'gantry_1': 't1_linear_joint', 'gantry_2': 't2_linear_joint'}
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
#
# PER ACTUATOR FAMILY, not one number (docs/p1_g17_hw.md B1.2). All seven
# calibration pairs above are joint_2 peaks, i.e. the offset was only ever
# measured on a KA-75+. Applying 6.6 to a KA-58 wrist rated 7.0 spends 94.3 % of
# the rating before any torque is predicted, which refused 9 of 18 step-3 plans
# with joint_4 binding 72 % of the time -- and G16's own measurements rule that
# out: wrist peak 0.499 N.m under load (B4.2) and -0.007 N.m at rest (B1.5),
# neither of which is possible if a 6.6 N.m friction term existed there.
# Scaled by nominal actuator torque: KA-75+ 12.0 -> 6.6, KA-58 3.6 -> 1.98.
# Still 3.3x above the largest wrist torque G16 ever measured, so it stays
# conservative; it is not a threshold move, it is the same measured constant
# applied only where it was measured to apply.
#
# G22 (docs/p1_g22_hw.md A8-1, operator decision): joint_2 raised 6.6 -> 7.7.
# g21 B8 over 81 executed plans (g18-g20): measured - RNEA reached +7.15 and
# grows with the prediction; g20 trial 8 passed at 13.72 and measured 14.27.
# +7.7 = max offset 7.15 + largest overshoot 0.55. joint_1/joint_3 unchanged.
JOINT_TORQUE_OFFSET_NM = {1: 6.6, 2: 7.7, 3: 6.6, 4: 1.98, 5: 1.98, 6: 1.98}
# STEP 3. Minimum distance allowed between the two arms that share a gantry,
# anywhere along a planned trajectory. MoveIt cannot supply this: 112 of the 121
# geometry-bearing t1_a1_* <-> t1_a2_* pairs are disabled in the SRDF with
# reason="Never" while the mounts are 0.800 m apart and each arm reaches 1.005 m
# (scripts/interarm_collision.py). 5 cm is chosen to absorb the discrete
# waypoint sampling, not because contact at 4 cm would be acceptable.
INTERARM_MARGIN_M = 0.05
# move_to() asks for another OMPL sample after these refusals (see there).
# G36: + RETRACT-BLOCKED (the plan ends where the arm cannot go home).
# G42: + GOAL-MISMATCH (a joint-goal plan ending > 3 mm / 2 deg off the target).
# G44: + START-JUMP (MoveIt sees the start in collision; see start_valid).
RETRYABLE = ('NO-PLAN', 'TUCK-REFUSED', 'TORQUE-UNSAFE', 'INTERARM-COLLIDE',
             'RETRACT-BLOCKED', 'GOAL-MISMATCH', 'START-JUMP')
# G42 (docs/p1_g42_joint_runner.md A1): a joint goal names no pose, so the
# plan's end is checked against the target the monitor will score.
GOAL_POS_TOL_M = 0.003
GOAL_AXIS_TOL_DEG = 2.0
# G47: joint tolerance of a TASK joint goal (one with a scored target). A
# sample anywhere in the +-0.2 deg box can land > 3 mm from the target
# (GOAL-MISMATCH 6-8 %, G42/G47); retract goals to REST keep 0.2.
TASK_GOAL_TOL_DEG = 0.2
RESERVE_DIR = None          # G54: parallel runner reservations (docs/p1_g54_parallel.md A1); None = old behaviour


# G44 (docs/p1_g44_start_jump.md B1/B2): 3 x p99.9 of 853 archived mock plans
# (d01 0.005, d_in 0.048 rad -- TOTG output resampled every 0.1 s). A guard for
# a trajectory that was NOT resampled; the start-validity check is the real one.
START_JUMP_T01 = 0.015
START_JUMP_TIN = 0.144


def start_valid(node, start_joints=None):
    """G44 B2: is the start valid in MoveIt's OWN scene? (valid, contacts), or
    (None, []) when /check_state_validity does not answer. group_name '' = the
    whole robot (a group name checks only that group's links, G40). The state
    is move_group's monitored state + the `start_joints` diff -- exactly what
    the planner starts from. An invalid start is the trigger of
    FixStartStateCollision: it jiggles the start, plans from the shifted copy
    and returns SUCCESS for a path whose first part MoveIt never checked."""
    from moveit_msgs.srv import GetStateValidity
    cli = getattr(node, '_csv_cli', None)
    if cli is None:
        cli = node._csv_cli = node.create_client(GetStateValidity,
                                                 '/check_state_validity')
    if not cli.wait_for_service(timeout_sec=10.0):
        return None, []
    req = GetStateValidity.Request()
    req.group_name = ''
    req.robot_state.is_diff = True
    if start_joints:
        req.robot_state.joint_state.name = list(start_joints)
        req.robot_state.joint_state.position = [float(v) for v in start_joints.values()]
    # G47 B1: a fresh process can lose the FIRST response ("failed to send
    # response ... client will not receive response" in move_group: the
    # client's response reader not yet matched). Re-send up to 3 times; still
    # nothing = None = the caller refuses (START-UNSCREENED).
    for _ in range(3):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        res = fut.result()
        if res is not None:
            return bool(res.valid), sorted({f'{c.contact_body_1}|{c.contact_body_2}'
                                            for c in res.contacts})
    return None, []


def start_jump(pts, start=None):
    """G44 (docs/p1_g44_start_jump.md): (d01, d_in, d0s) in rad, each the max over
    the planned joints -- |p1 - p0|, the largest later waypoint step, and |p0 -
    start|. pts = trajectory positions (one list per waypoint), start = the
    start positions in the same joint order or None. FixStartStateCollision
    prepends the original start as waypoint 0 and its shifted copy follows, so
    a start the planner found "in collision" shows up as a jump that no
    waypoint screen ever looks inside."""
    import numpy as np
    p = np.asarray(pts, float)
    d = np.abs(np.diff(p, axis=0)).max(axis=1) if len(p) > 1 else np.zeros(1)
    d0s = 0.0 if start is None else float(np.abs(p[0] - np.asarray(start, float)).max())
    return float(d[0]), float(d[1:].max()) if len(d) > 1 else 0.0, d0s


class Probe(Node):
    def __init__(self, arm, approach, tau_max, move, topic=PERCEPT_TOPIC,
                 fixed=None, arms=None):
        super().__init__('reach_dwell_probe')
        self.arm, self.approach, self.tau_max, self.move = \
            arm, approach, tau_max, move
        self.fixed = fixed
        self.perceived = None
        self.status = []
        self.tau_peak = 0.0
        self.tau_worst_joint = ''
        # /joint_states has TWO publishers here and they send SEPARATE messages
        # carrying different joints, so no single message is the whole robot --
        # `--once` picks one of them and silently misses the rest. Merging by
        # name across messages is the only way to hold a complete state.
        self.joint_pos = {}

        self.create_subscription(PoseStamped, topic, self._on_percept, 10)
        self.create_subscription(String, '/reach_dwell/status',
                                 self._on_status, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.target_pub = {
            a: self.create_publisher(PoseStamped, f'/reach_dwell/target/{a}', 10)
            for a in (arms or [arm])}
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
        for name, pos in zip(msg.name, msg.position or []):
            self.joint_pos[name] = float(pos)

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

    def publish_target(self, p_cmd, arm=None, timeout=15.0):
        """MUST happen before motion -- see the module docstring.

        In dual-arm mode BOTH targets go out before EITHER arm moves. That is
        not merely tidy: reach_dwell_monitor resets its common-window clock
        whenever a target arrives, so publishing arm_2's target after arm_1 was
        already holding would discard the very window being measured.
        """
        pub = self.target_pub[arm or self.arm]
        if not self._await_match(pub, timeout):
            return False
        pub.publish(p_cmd)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def clear(self, arm=None, timeout=15.0):
        if not self._await_match(self.clear_pub, timeout):
            return False
        self.clear_pub.publish(String(data=arm or self.arm))
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def wait_joints(self, names, seconds=1.0):
        """Merge /joint_states by name until every `names` entry is present.

        See __init__: two publishers, separate messages. Measured in G16 to make
        a single-message read fail silently ~90 % of the time -- and here a
        missing joint would place the OTHER arm at its URDF neutral pose inside
        the collision screen, which is exactly the wrong direction to be wrong.
        """
        t0 = time.time()
        while time.time() - t0 < seconds or not all(n in self.joint_pos
                                                    for n in names):
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > seconds + 5.0:
                break
        return {n: self.joint_pos[n] for n in names if n in self.joint_pos}

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

    def collect_concurrent(self, seconds, arms):
        """Watch for the COMMON window -- the quantity step 3 exists to measure.

        docs/p1_g4_reach_dwell.md A1: N-arm success is ONE window of 2.0 s in
        which ALL arms are inside their own tolerance SIMULTANEOUSLY. Two arms
        that each succeed at different moments is exactly what the prior work
        already does, so per-arm successes are counted but they are NOT the
        result. `STAGGERED` names that outcome instead of letting it pass as a
        success, which is the whole reason the distinction was locked.
        """
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tau_peak > self.tau_max:
                self.get_logger().error(
                    f'ABORT: torsi {self.tau_peak:.2f} N.m pada '
                    f'{self.tau_worst_joint} > batas {self.tau_max:.2f}')
                return 'TORQUE-ABORT', set()
            for s in self.status:
                if s.get('event') == 'concurrent' and s.get('n_arms', 0) >= len(arms):
                    return 'CONCURRENT', set(arms)
                # G22 (docs/p1_g22_hw.md B0): the monitor never publishes
                # `concurrent` for ONE arm (g20 B1.2), so N = 1 used to sit out
                # the whole --settle after its own success -- dead time that a
                # timed schedule would book as makespan. N = 1 ends on success.
                if len(arms) == 1 and s.get('event') == 'success' \
                        and s.get('arm') == arms[0]:
                    return 'SUCCESS', set(arms)
        won = {s.get('arm') for s in self.status if s.get('event') == 'success'}
        won &= set(arms)
        if won == set(arms):
            return 'STAGGERED', won
        return ('PARTIAL' if won else 'NEITHER'), won

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


def _joint_goal(joints, tol=math.radians(0.2)):
    """A joint-space goal {joint: pos} (G34: env-aware retract plans to REST)."""
    c = Constraints()
    for n, v in joints.items():
        jc = JointConstraint()
        jc.joint_name = n
        jc.position = float(v)
        jc.tolerance_above = jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
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


def goal_pose_error(traj, arm, node, target, start_joints=None):
    """G42 GOAL-MISMATCH: FK of the plan's LAST point (tool frame, live URDF;
    rail + rotation from `start_joints` or /joint_states) vs `target`.
    (pos_err_m, approach_axis_err_deg) or None when it cannot be computed."""
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
    g = GANTRY_OF[arm]
    base = [RAIL_JOINT[g], f't{g[-1]}_rotation_joint']
    known = dict(start_joints) if start_joints else node.wait_joints(base)
    jt = traj.joint_trajectory
    known.update(zip(jt.joint_names, jt.points[-1].positions))
    q = pin.neutral(m)
    for n in base + [f'{JOINT_PREFIX[arm]}joint_{i}' for i in range(1, 7)]:
        if n not in known or not m.existJointName(n):
            return None
        jm = m.joints[m.getJointId(n)]
        if jm.nq != 1:
            return None
        q[jm.idx_q] = known[n]
    if not m.existFrame(TOOL_FRAME[arm]):
        return None
    pin.framesForwardKinematics(m, d, q)
    T = d.oMf[m.getFrameId(TOOL_FRAME[arm])]
    p, o = target.pose.position, target.pose.orientation
    pos = float(np.linalg.norm(np.array(T.translation) - [p.x, p.y, p.z]))
    # +Z of the target quaternion = the approach axis the monitor scores
    a_cmd = np.array([2 * (o.x * o.z + o.w * o.y), 2 * (o.y * o.z - o.w * o.x),
                      1 - 2 * (o.x * o.x + o.y * o.y)])
    a_cmd /= np.linalg.norm(a_cmd)
    c = float(np.clip(np.dot(np.array(T.rotation)[:, 2], a_cmd), -1.0, 1.0))
    return pos, math.degrees(math.acos(c))


def screen_interarm(traj, arm, node, other_arm, margin=INTERARM_MARGIN_M,
                    other_joints=None):
    """Screen a plan against the OTHER arm on the same gantry, before executing.

    STEP 3 only. Step 2 moved one arm and needed nothing like this.

    `other_joints` PLACES the other arm (and the rail) instead of reading them
    off /joint_states -- for SELECTION only, docs/p1_g19_hw.md A4: arm_2 must be
    screened with arm_1 already at its paired target (g18 B2.2), and that has to
    be screened without moving arm_1 there.

    This exists because MoveIt here cannot answer the question: the SRDF marks
    112 of the 121 geometry-bearing cross-arm pairs `reason="Never"`, including
    gripper-vs-gripper, so a plan that drives arm_1 through arm_2 comes back
    valid. Verified offline: driving both tools to (0.55, 0.36, 1.40) puts the
    two left finger links in contact, and that exact pair is disabled at
    trailer_workcell.srdf:339.

    Returns None when the screen cannot run -- the caller must then REFUSE,
    the same way an unavailable torque model refuses.
    """
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from interarm_collision import CrossGantryChecker, InterArmChecker
    except ImportError:
        return None
    # docs/p1_g20_hw.md: `other_arm` may be ONE arm (steps 3-4, same gantry,
    # unchanged) or a LIST (step 5): every arm in it is HELD still, the ones on
    # this arm's gantry go to the mesh checker validated in g17-g19, the ones on
    # the other gantry to the hull checker -- MoveIt screens neither, 112/121
    # same-gantry and 283 cross-gantry pairs are disabled in the SRDF.
    others = [other_arm] if isinstance(other_arm, str) else list(other_arm)
    g = GANTRY_OF[arm]
    same = [o for o in others if GANTRY_OF[o] == g]
    cross = [o for o in others if GANTRY_OF[o] != g]
    if not hasattr(node, '_interarm'):
        node._interarm = {}
    try:
        if same and g not in node._interarm:
            node._interarm[g] = InterArmChecker(gantry=g, margin=margin)
        if cross and 'cross' not in node._interarm:
            node._interarm['cross'] = CrossGantryChecker(margin=margin)
    except Exception:                                        # noqa: BLE001
        return None
    # The other arms are HELD at their measured configuration while this moves.
    want = [RAIL_JOINT[g]]
    for o in others:
        want += [f'{JOINT_PREFIX[o]}joint_{i}' for i in range(1, 7)]
        if RAIL_JOINT[GANTRY_OF[o]] not in want:
            want.append(RAIL_JOINT[GANTRY_OF[o]])
    # docs/p1_g31_rot_sched.md A2 (G29 C-2): BOTH gantry rotations, always.
    # Without them q_from starts from neutral and S18 silently judged rot = 0.
    want += ['t1_rotation_joint', 't2_rotation_joint']
    # Strict names: q_from drops a name its model does not know, without a word.
    used = ([node._interarm[g]] if same else []) + \
        ([node._interarm['cross']] if cross else [])
    unknown = sorted({n for n in want + list(traj.joint_trajectory.joint_names)
                      for c in used if not c.model.existJointName(n)})
    if unknown:
        node.get_logger().error(
            f'penyaring tabrakan: sendi tidak dikenal model {unknown} -- '
            'MENOLAK menyaring dengan nama yang akan dibuang diam-diam.')
        return None
    other = {n: other_joints[n] for n in want if n in other_joints} \
        if other_joints is not None else node.wait_joints(want)
    if len(other) < len(want):
        node.get_logger().error(
            f'penyaring tabrakan: hanya {len(other)}/{len(want)} sendi untuk '
            'lengan lain -- MENOLAK menyaring dengan keadaan yang tidak '
            f'lengkap (hilang {sorted(set(want) - set(other))}).')
        return None
    jt = traj.joint_trajectory
    pts = [list(p.positions) for p in jt.points]
    res = []
    if same:
        res.append(node._interarm[g].screen_trajectory(
            list(jt.joint_names), pts, other))
    if cross:
        res.append(node._interarm['cross'].screen_trajectory(
            list(jt.joint_names), pts, other, only=JOINT_PREFIX[arm]))
    return min(res, key=lambda r: r[1])


def screen_env(traj, node, start_joints=None):
    """G33: the plan against the frozen camera map (scripts/env_collision.py),
    with every other joint HELD at its measured value (or `start_joints`).

    Until G33 nothing here saw the environment and G32-HW R10 ev 8 drove arm_3
    into a rack after passing every screen (docs/p1_g32_hw.md B3). Also asks
    move_group whether `env_static_map` is really in its planning scene: a plan
    made without it was planned blind, whatever this screen says afterwards.

    Returns None when the screen cannot run -- the caller must REFUSE."""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from env_collision import CONFIG_JOINTS, EnvChecker
        from env_static_map_pub import OBJECT_ID
        if not hasattr(node, '_env'):
            node._env = EnvChecker()
    except Exception as e:                                    # noqa: BLE001
        node.get_logger().error(f'penyaring lingkungan tidak bisa dimuat: {e}')
        return None
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    if not hasattr(node, '_gps'):
        node._gps = node.create_client(GetPlanningScene, '/get_planning_scene')
    if not node._gps.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('/get_planning_scene tidak ada')
        return None
    rq = GetPlanningScene.Request()
    rq.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    f = node._gps.call_async(rq)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    ids = [o.id for o in f.result().scene.world.collision_objects] \
        if f.result() is not None else []
    if OBJECT_ID not in ids:
        node.get_logger().error(
            f'{OBJECT_ID} TIDAK ada di planning scene ({ids}) -- MoveIt merencana '
            'buta lingkungan. Jalankan scripts/env_static_map_pub.py.')
        return None
    base = node.wait_joints(CONFIG_JOINTS)
    if start_joints:
        base.update({k: v for k, v in start_joints.items() if k in CONFIG_JOINTS})
    if len(base) < len(CONFIG_JOINTS):
        node.get_logger().error(
            f'penyaring lingkungan: hilang {sorted(set(CONFIG_JOINTS) - set(base))}')
        return None
    jt = traj.joint_trajectory
    try:
        return node._env.screen_trajectory(
            list(jt.joint_names), [list(p.positions) for p in jt.points], base)
    except KeyError as e:
        node.get_logger().error(f'penyaring lingkungan: {e}')
        return None


def move_to(arm, p_cmd, node, pos_tol=0.002, ori_tol_deg=2.0,
            vel_scale=0.15, plan_time=15.0, plan_only=False, tau_max=None,
            other_arm=None, attempts=1, retract_check=False, start_joints=None,
            target=None):
    """Plan to p_cmd, CHECK the plan against A6/S2, then execute it.

    G42: p_cmd may be a {joint: pos} goal; `target` (the PoseStamped the
    monitor scores) then feeds the GOAL-MISMATCH screen. `start_joints` plans
    from a PLACED state and is allowed with plan_only ONLY. A PLANNED plan's
    last point is left in node._plan_end, the attempt count in node._plan_tries.

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
    if start_joints and not plan_only:
        raise ValueError('start_joints hanya untuk plan_only: rencana akan '
                         'mulai di tempat lengan TIDAK berada')
    if not hasattr(node, '_plan_ac'):
        node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
        node._exec_ac = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
    if not node._plan_ac.wait_for_server(timeout_sec=15.0):
        return 'EXEC-FAIL'

    def _plan_once():
        """Ask for one plan and run every screen on it. (verdict, traj)."""
        return _plan_and_screen(arm, p_cmd, node, pos_tol, ori_tol_deg,
                                vel_scale, plan_time, tau_max, other_arm,
                                start_joints=start_joints,
                                retract_check=retract_check, target=target)

    # RETRY, and this is a feasibility change rather than a safety one.
    # docs/p1_g17_hw.md B1.9 measured the same target from the same rest pose
    # coming back TORQUE-UNSAFE on one attempt and PLANNED on the next: 8 of 10
    # for (1.286, 0.459, 1.160), while other targets were stable 10/10 either
    # way. The refusal is a property of the trajectory OMPL happened to sample,
    # not of the pose. Taking one sample therefore discards good targets on
    # planner luck and files the loss as a method failure, which is exactly the
    # misattribution A3 exists to prevent.
    #
    # EVERY plan is still screened in full, and only a plan that passes every
    # screen is ever executed -- so the guard is untouched. What changes is the
    # MEANING of a refusal verdict: "no safe trajectory found in `attempts`
    # tries" instead of "this one sample was bad".
    #
    # Not retried: TORQUE-UNSCREENED / INTERARM-UNSCREENED / EXEC-FAIL. Those
    # say the screen or the machinery is unavailable, and repeating a broken
    # tool just hides that it is broken.
    verdict, traj = _plan_once()
    node._plan_tries = 1
    node._plan_verdicts = [verdict]          # G46: every attempt's verdict (G42 B1 logging gap)
    for k in range(2, max(1, attempts) + 1):
        if verdict not in RETRYABLE:
            break
        node.get_logger().info(
            f'percobaan rencana {k}/{attempts}: yang sebelumnya {verdict} '
            '-- meminta lintasan LAIN (penyaring tidak dilonggarkan)')
        verdict, traj = _plan_once()
        node._plan_tries = k
        node._plan_verdicts.append(verdict)
    if verdict != 'PLANNED':
        return verdict
    jt = traj.joint_trajectory
    node._plan_end = dict(zip(jt.joint_names, map(float, jt.points[-1].positions)))

    # A6/S7: --move not typed means the arm does not move, but the plan was
    # still worth asking for -- it separates NO-PLAN from reachable-but-untried
    # without commanding anything.
    if plan_only:
        return 'PLANNED'

    if RESERVE_DIR is not None:              # G54: screen vs the other gantry's reservation, then reserve, THEN move
        rv = _reserve(traj, arm, node)
        if rv != 'CLEAR':
            return rv
        return _execute_direct(node, traj, arm)
    return _execute(node, traj)


def _reserve(traj, arm, node):
    """G54 (docs/p1_g54_parallel.md A1). Under the runner's flock: the plan (this arm's waypoints, everything else at
    its measured value) against EVERY state the other gantry has reserved; CLEAR -> res_own = measured + plan, then the
    caller executes. res_own shrinks when the RUNNER sees the event end. 'CLEAR' | 'RESERVE-BLOCKED' |
    'RESERVE-UNSCREENED' (no motion on either refusal)."""
    sys.path.insert(0, '/home/user1/Documents/ceiling_arm/docs/results/p1_g54')
    import reserve as RSV
    g = int(GANTRY_OF[arm][-1])
    go = 2 if g == 1 else 1
    jt = traj.joint_trajectory
    names = list(jt.joint_names)
    pts = [list(map(float, p.positions)) for p in jt.points]
    try:
        node._interarm = getattr(node, '_interarm', {})
        if 'cross' not in node._interarm:
            from interarm_collision import CrossGantryChecker
            node._interarm['cross'] = CrossGantryChecker(margin=INTERARM_MARGIN_M)
        with RSV.Lock(RESERVE_DIR) as L:
            want = RSV.GANTRY_JOINTS[1] + RSV.GANTRY_JOINTS[2]
            base = node.wait_joints(want)
            other = L.read(go)
            if base is None or len(base) < len(want) or other is None:
                node.get_logger().error('G54 reservasi: keadaan/reservasi tidak lengkap -- MENOLAK.')
                return 'RESERVE-UNSCREENED'
            v, d, pair, _ = RSV.screen(node._interarm['cross'], names, pts, base, other, go)
            node.get_logger().info(f'G54 reservasi g{go}: {len(other)} keadaan, minimum {d * 1000:.1f} mm '
                                   f'({pair}) -> {v}')
            if v != 'CLEAR':
                return 'RESERVE-BLOCKED'
            states = [dict(base)]
            for q in pts:
                st = dict(base)
                st.update(zip(names, q))
                states.append(st)
            L.write(g, states, f'probe {arm} {os.getpid()}')
    except Exception as e:                                   # noqa: BLE001
        node.get_logger().error(f'G54 reservasi gagal: {e} -- MENOLAK.')
        return 'RESERVE-UNSCREENED'
    return 'CLEAR'


def _execute_direct(node, traj, arm):
    """G54 B1: move_group SERIALISES ExecuteTrajectory goals (measured, g54_concurrent_exec), so with --reserve-dir the
    SAME time-parameterised plan goes straight to this gantry's FollowJointTrajectory controller -- the controller MoveIt
    would pick, the path return_rest / pose_to_g already use. MoveIt's TEM checks kept: start within 0.01 rad
    (allowed_start_tolerance) and result within duration x 3 + 5 s (my_workcell.launch.py)."""
    from control_msgs.action import FollowJointTrajectory
    jt = traj.joint_trajectory
    ctl = f'/{GANTRY_OF[arm]}_with_arm_controller/follow_joint_trajectory'
    if not hasattr(node, '_fjt'):
        node._fjt = {}
    if ctl not in node._fjt:
        node._fjt[ctl] = ActionClient(node, FollowJointTrajectory, ctl)
    ac = node._fjt[ctl]
    if not ac.wait_for_server(timeout_sec=10.0):
        return 'EXEC-FAIL'
    now = node.wait_joints(list(jt.joint_names), seconds=0.2)
    dev = max((abs(now[n] - float(q)) for n, q in zip(jt.joint_names, jt.points[0].positions) if n in now),
              default=float('inf'))
    if len(now) < len(jt.joint_names) or dev > 0.01:
        node.get_logger().error(f'G54 eksekusi langsung: awal rencana {dev:.4f} rad dari keadaan terukur > 0.01 -- MENOLAK.')
        return 'EXEC-FAIL'
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = jt
    dur = jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
    gh = f.result()
    if gh is None or not gh.accepted:
        return 'EXEC-FAIL'
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=dur * 3.0 + 5.0)
    if not rf.done():
        node.get_logger().error(f'G54 eksekusi langsung: {ctl} melewati {dur * 3.0 + 5.0:.1f} s -- goal DIBATALKAN.')
        cf = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cf, timeout_sec=5.0)
        return 'EXEC-FAIL'
    r = rf.result()
    if r is None or r.result.error_code != 0:
        node.get_logger().error(f'G54 eksekusi langsung: {ctl} error_code '
                                f'{None if r is None else r.result.error_code}')
        return 'EXEC-FAIL'
    return 'MOVED'


def _execute(node, traj):
    """The old tail of move_to, unchanged (execution through move_group)."""
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


def _plan_and_screen(arm, p_cmd, node, pos_tol, ori_tol_deg, vel_scale,
                     plan_time, tau_max, other_arm, start_joints=None,
                     retract_check=False, target=None):
    """One plan request plus every pre-execution screen. ('PLANNED', traj) or
    (refusal, None). Split out of move_to so a refusal can be re-sampled
    without duplicating a single line of the screening logic.

    `start_joints` ({joint: pos}) plans from a PLACED state rather than the
    measured one -- MoveIt start_state as a diff, and the same placement handed
    to the inter-arm screen. Selection only (dual_arm_targets.py --pairs-file):
    it lets a rail position and a partner-arm pose be screened with zero motion.
    Never pass it on a path that executes: the plan would start somewhere the
    arm is not."""
    # G44: refuse BEFORE planning when MoveIt itself sees the start in collision
    # -- otherwise FixStartStateCollision answers SUCCESS from a shifted start.
    sv, contacts = start_valid(node, start_joints)
    if sv is None:
        node.get_logger().error(
            '/check_state_validity tidak menjawab -- start tidak tersaring, MENOLAK.')
        return 'START-UNSCREENED', None
    if not sv:
        node.get_logger().error(
            f'START-JUMP: start TIDAK VALID di scene MoveIt ({", ".join(contacts[:4])}) '
            '-- FixStartStateCollision akan menggeser start tanpa pemeriksaan. DITOLAK.')
        return 'START-JUMP', None

    goal = MoveGroup.Goal()
    goal.request.group_name = arm
    if start_joints:
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.name = list(start_joints)
        goal.request.start_state.joint_state.position = \
            [float(v) for v in start_joints.values()]
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = plan_time
    goal.request.max_velocity_scaling_factor = vel_scale
    goal.request.max_acceleration_scaling_factor = vel_scale
    # p_cmd: a PoseStamped on the tool frame, or (G34) a {joint: pos} dict.
    goal.request.goal_constraints.append(
        (_joint_goal(p_cmd) if target is None else _joint_goal(p_cmd, math.radians(TASK_GOAL_TOL_DEG)))
        if isinstance(p_cmd, dict)
        else _goal_constraints(TOOL_FRAME[arm], p_cmd, pos_tol, ori_tol_deg))
    goal.planning_options.plan_only = True

    fut = node._plan_ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return 'NO-PLAN', None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=plan_time + 20.0)
    res = rf.result()
    if res is None or res.result.error_code.val != MoveItErrorCodes.SUCCESS:
        return 'NO-PLAN', None

    traj = res.result.planned_trajectory
    if not traj.joint_trajectory.points:
        return 'NO-PLAN', None

    jt_ = traj.joint_trajectory
    known = dict(start_joints) if start_joints else (
        node.wait_joints(list(jt_.joint_names)) if hasattr(node, 'wait_joints') else {})
    st_ = [known[n] for n in jt_.joint_names] if known and all(
        n in known for n in jt_.joint_names) else None
    d01, d_in, d0s = start_jump([list(p.positions) for p in jt_.points], st_)
    if d01 > START_JUMP_T01 or d_in > START_JUMP_TIN or d0s > START_JUMP_T01:
        node.get_logger().error(
            f'START-JUMP: langkah waypoint 0->1 {d01:.4f} / maks {d_in:.4f} / '
            f'p0-start {d0s:.4f} rad (batas {START_JUMP_T01} / {START_JUMP_TIN}). DITOLAK.')
        return 'START-JUMP', None

    hit = _violates_tuck(traj, arm)
    if hit is not None and hit >= 0:
        node.get_logger().error(
            f'A6/S2: rencana DITOLAK -- titik {hit} berada dalam 10 deg dari '
            f'FORBIDDEN_TUCK {FORBIDDEN_TUCK}. TIDAK dieksekusi.')
        return 'TUCK-REFUSED', None

    # G42: a joint goal says nothing about where the TOOL ends. The monitor
    # scores the tool against `target`, so a goal paired with the wrong target
    # (or a sample at the corner of the 0.2 deg joint box) is refused here.
    if isinstance(p_cmd, dict) and target is not None:
        ge = goal_pose_error(traj, arm, node, target, start_joints)
        if ge is None:
            node.get_logger().error(
                'TIDAK BISA menghitung FK ujung rencana -- MENOLAK.')
            return 'GOAL-UNSCREENED', None
        node.get_logger().info(
            f'ujung rencana vs target: {ge[0] * 1000:.2f} mm, sumbu {ge[1]:.2f} deg')
        if ge[0] > GOAL_POS_TOL_M or ge[1] > GOAL_AXIS_TOL_DEG:
            node.get_logger().error(
                f'GOAL-MISMATCH: ujung rencana {ge[0] * 1000:.2f} mm / {ge[1]:.2f} '
                f'deg dari target (batas {GOAL_POS_TOL_M * 1000:.1f} mm / '
                f'{GOAL_AXIS_TOL_DEG:.1f} deg).')
            return 'GOAL-MISMATCH', None

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
            return 'TORQUE-UNSCREENED', None
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
            corr = v + JOINT_TORQUE_OFFSET_NM.get(j, 6.6)
            parts.append(f'{j}={v:.2f}->{corr:.2f}/{lim:.0f}')
            if corr / lim > worst_frac:
                worst, worst_frac, worst_corr, worst_lim = k, corr / lim, corr, lim
        node.get_logger().info(
            'torsi RNEA+offset(per-aktuator) vs rating: ' + ', '.join(parts)
            + f'  (terketat {worst} {worst_frac * 100:.0f} %)')
        if worst_frac > 1.0:
            node.get_logger().error(
                f'A6/S3: rencana DITOLAK SEBELUM GERAK -- {worst} diperkirakan '
                f'{worst_corr:.2f} N.m > rating {worst_lim:.0f}. Pose ini '
                'menuntut torsi di atas rating sendi; itu sifat POSE, bukan '
                'sifat penjaga. (Penyaring sengaja konservatif: pada kalibrasi '
                'G16 ia menolak 1 dari 4 pose yang ternyata aman.)')
            return 'TORQUE-UNSAFE', None

    # STEP 3. Runs after the torque screen and before execution, because the
    # goal-pose screen in dual_arm_targets.py only clears the ENDPOINTS -- the
    # measured minimum there was 262 mm, while what the arms do in TRANSIT
    # between hanging and those endpoints is not constrained by it at all.
    if other_arm is not None:
        sc = screen_interarm(traj, arm, node, other_arm,
                             other_joints=start_joints)
        if sc is None:
            node.get_logger().error(
                'TIDAK BISA menyaring tabrakan antar-lengan -- MENOLAK '
                'bergerak tanpa penyaringan. SRDF tidak dapat dipakai sebagai '
                'cadangan: 112 dari 121 pasangan silang dimatikan di sana.')
            return 'INTERARM-UNSCREENED', None
        verdict, dist, pair, k = sc
        node.get_logger().info(
            f'jarak antar-lengan minimum sepanjang rencana: {dist * 1000:.1f} mm '
            f'di titik {k} ({pair[0]} <-> {pair[1]}) -> {verdict}')
        if verdict != 'CLEAR':
            node.get_logger().error(
                f'TABRAKAN ANTAR-LENGAN: rencana DITOLAK SEBELUM GERAK -- '
                f'{dist * 1000:.1f} mm di titik {k} antara {pair[0]} dan '
                f'{pair[1]}, di bawah margin {INTERARM_MARGIN_M * 1000:.0f} mm. '
                'MoveIt menilai rencana ini SAH karena pasangan itu dimatikan '
                'di SRDF.')
            return 'INTERARM-COLLIDE', None

    # G33: arm vs ENVIRONMENT (frozen camera map). Always, not only with
    # other_arm: a single arm hit the rack in G32-HW.
    se = screen_env(traj, node, start_joints)
    if se is None:
        node.get_logger().error(
            'TIDAK BISA menyaring tabrakan lingkungan -- MENOLAK bergerak.')
        return 'ENV-UNSCREENED', None
    verdict, dist, geom, k = se
    node.get_logger().info(
        f'jarak ke peta lingkungan minimum: {dist * 1000:.1f} mm di titik {k} '
        f'({geom}) -> {verdict}')
    if verdict != 'CLEAR':
        node.get_logger().error(
            f'TABRAKAN LINGKUNGAN: rencana DITOLAK SEBELUM GERAK -- '
            f'{dist * 1000:.1f} mm di titik {k} ({geom}), di bawah margin '
            f'{node._env.margin * 1000:.0f} mm.')
        return 'ENV-COLLIDE', None

    # G36: a task plan is accepted only if the arm can go home from where it
    # ENDS by the straight line return_rest tries first (same screens). G35b:
    # an ev 10 plan ended on the +100 deg IK branch at the rack edge, the
    # straight retract went through the rack and MoveIt could not plan home
    # either -- on hardware that strands the arm at the rack. Refusing here
    # costs one more OMPL sample (move_to retries it); the arm has not moved.
    if retract_check:
        rv = screen_retract(traj, arm, node, start_joints)
        if rv is None:
            node.get_logger().error('TIDAK BISA menyaring jalur pulang -- MENOLAK.')
            return 'RETRACT-UNSCREENED', None
        if rv != 'CLEAR':
            node.get_logger().error(
                f'JALUR PULANG TERHALANG: rencana DITOLAK SEBELUM GERAK -- garis lurus '
                f'ke REST dari titik akhirnya {rv}.')
            return 'RETRACT-BLOCKED', None

    return 'PLANNED', traj


def screen_retract(traj, arm, node, start_joints=None):
    """G36: return_rest.plan_retract (straight line only, no MoveIt) for `arm`
    alone from the END of traj, everything else held as screen_env holds it.
    'CLEAR' | 'LURUS-<verdicts>' | None when it cannot run."""
    try:
        from env_collision import CONFIG_JOINTS
        from interarm_collision import CrossGantryChecker, InterArmChecker
        from return_rest import plan_retract
    except ImportError:
        return None
    g = GANTRY_OF[arm]
    if not hasattr(node, '_interarm'):
        node._interarm = {}
    try:
        if g not in node._interarm:
            node._interarm[g] = InterArmChecker(gantry=g, margin=INTERARM_MARGIN_M)
        if 'cross' not in node._interarm:
            node._interarm['cross'] = CrossGantryChecker(margin=INTERARM_MARGIN_M)
    except Exception:                                        # noqa: BLE001
        return None
    if not hasattr(node, '_env'):
        return None                     # screen_env ran first and loaded it
    base = node.wait_joints(CONFIG_JOINTS)
    if start_joints:
        base.update({k: v for k, v in start_joints.items() if k in CONFIG_JOINTS})
    if len(base) < len(CONFIG_JOINTS):
        return None
    jt = traj.joint_trajectory
    base.update(zip(jt.joint_names, jt.points[-1].positions))
    try:
        v, _ = plan_retract(int(g[-1]), [arm], base, node._interarm[g],
                            node._interarm['cross'], node._env,
                            log=lambda s: node.get_logger().info(s))
    except KeyError as e:
        node.get_logger().error(f'penyaring jalur pulang: {e}')
        return None
    return v


def dual_trial(node, a, i, arms, fixed2):
    # g20: fixed2 = ONE xyz (step 3/4, arm_2) or a LIST of xyz for arms[1:]
    # (step 5, --quad). Same sequence either way: every target published
    # before any arm moves, then the arms one after another (S11).
    """One step-3 trial: both arms of gantry 1, scored on the COMMON window.

    The arms are commanded SEQUENTIALLY -- arm_1 arrives, then arm_2 -- and that
    is deliberate, not a limitation worked around. The locked criterion asks
    whether a common 2.0 s window EXISTS, not whether the two arms travelled at
    once; arm_1 holding its pose under position control while arm_2 flies is a
    legitimate way to produce one. Commanding both simultaneously would also
    put two arms in motion in a shared volume screened only at waypoints, which
    is a much larger physical risk for no gain in what is being measured.
    """
    node.status.clear()
    # NO /reach_dwell/clear here, and that is a fix rather than an omission.
    # Measured in fake hardware, trial 10 of 10: the probe published clear
    # BEFORE the targets, but clear and target are separate topics with
    # separate DDS match times, and the clear was DELIVERED 1 ms AFTER them --
    #     arm_1: target set (0.714, 0.318, 1.240)
    #     cleared: arm_1
    # -- which wiped the freshly set targets, left the monitor with no active
    # task, and produced NEITHER while both arms were in fact sitting 1.2 mm
    # from their targets. Same family as g16 B3.4(6), mirrored.
    #
    # The clear was redundant anyway: _on_target REPLACES tasks[arm] and resets
    # concurrent_since/concurrent_reported, so re-targeting an arm already does
    # everything clearing it would. Removing it removes the race outright
    # instead of trying to out-wait it.
    p = node.wait_percept(timeout=10.0)
    extra = [fixed2] if not a.screen_all else list(fixed2)
    if p is None:
        return dict(trial=i, verdict='INVALID')
    cmds = {arms[0]: node.command_pose(p)}
    for arm, xyz in zip(arms[1:], extra):
        p2 = PoseStamped()
        p2.header.frame_id = 'world'
        p2.pose.position.x, p2.pose.position.y, p2.pose.position.z = xyz
        p2.pose.orientation.w = 1.0
        cmds[arm] = node.command_pose(p2)

    def _others(arm):
        # --dual: the partner, exactly as steps 3-4 ran. --arms/--quad (g20):
        # ALL three other arms, moving in this trial or not -- with both
        # gantries live, an arm left at REST is still in the way.
        if not a.screen_all:
            return arms[1] if arm == arms[0] else arms[0]
        return [x for x in TOOL_FRAME if x != arm]

    # BOTH before EITHER moves -- the monitor restarts its common-window clock
    # on every incoming target (reach_dwell_monitor._on_target).
    for arm in arms:
        if not node.publish_target(cmds[arm], arm):
            print(f'  [{i:2d}] TIDAK VALID (mesin): tidak ada pelanggan di '
                  f'/reach_dwell/target/{arm} -- monitor tidak memantau '
                  'KEDUA lengan? A5, diulang, di luar penyebut.')
            return dict(trial=i, verdict='INVALID')

    def _goal(arm):
        # G42: --goal-joints plans to the arm's joints; the monitor still
        # scores cmds[arm] (published above), and GOAL-MISMATCH ties the two.
        if not a.goal_joints:
            return cmds[arm]
        return {k: v for k, v in a.goal_joints.items()
                if k.startswith(JOINT_PREFIX[arm])}

    tgt = (lambda arm: cmds[arm]) if a.goal_joints else (lambda arm: None)

    if not a.move:
        plans = {}
        for arm in arms:
            other = _others(arm)
            node._plan_end, node._plan_tries, node._plan_verdicts = None, None, None
            mv = move_to(arm, _goal(arm), node, plan_only=True,
                         tau_max=a.tau_max, other_arm=other,
                         attempts=a.plan_attempts, retract_check=True,
                         start_joints=a.start_joints, target=tgt(arm)) \
                if a.plan_check else 'DRY'
            if a.plan_check:
                plans[arm] = dict(verdict=mv, tries=node._plan_tries,
                                  verdicts=node._plan_verdicts,
                                  end=node._plan_end if mv == 'PLANNED' else None)
            c = cmds[arm].pose.position
            print(f'  [{i:2d}] DRY {arm}: p_cmd = ({c.x:+.3f}, {c.y:+.3f}, '
                  f'{c.z:+.3f})' + (f'   rencana: {mv}' if a.plan_check else ''))
        return dict(trial=i, verdict='DRY', **({'plans': plans} if plans else {}))

    node.tau_peak, node.tau_worst_joint = 0.0, ''
    moves, pverd = {}, {}
    for arm in arms:
        other = _others(arm)
        node._plan_verdicts = None
        moves[arm] = move_to(arm, _goal(arm), node, tau_max=a.tau_max,
                             other_arm=other, attempts=a.plan_attempts,
                             retract_check=True, target=tgt(arm))
        pverd[arm] = node._plan_verdicts
        print(f'       {arm}: moveit {moves[arm]}  (percobaan rencana: {pverd[arm]})')
        if moves[arm] != 'MOVED':
            break

    if any(v != 'MOVED' for v in moves.values()) or len(moves) < len(arms):
        bad = [f'{k}:{v}' for k, v in moves.items() if v != 'MOVED']
        v = 'HALTED'
        print(f'  [{i:2d}] {v:12s} ({", ".join(bad) or "tidak lengkap"}) '
              f'torsi puncak {node.tau_peak:5.2f} N.m')
        return dict(trial=i, verdict=v, move_status=moves, plan_verdicts=pverd,
                    tau_peak=round(node.tau_peak, 3))

    v, won = node.collect_concurrent(a.settle, arms)
    print(f'  [{i:2d}] {v:12s} (lengan sukses sendiri-sendiri: '
          f'{sorted(won) or "tidak ada"}) torsi puncak {node.tau_peak:5.2f} N.m')
    return dict(trial=i, verdict=v, move_status=moves, plan_verdicts=pverd,
                per_arm_success=sorted(won),
                tau_peak=round(node.tau_peak, 3),
                targets={k: [round(c.pose.position.x, 4),
                             round(c.pose.position.y, 4),
                             round(c.pose.position.z, 4)]
                         for k, c in cmds.items()})


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
    ap.add_argument('--dual', action='store_true',
                    help='LANGKAH 3: arm_1 + arm_2 pada gantry_1, dinilai pada '
                         'jendela dwell BERSAMA (A1 N-lengan), bukan pada '
                         'sukses masing-masing')
    ap.add_argument('--plan-attempts', type=int, default=3,
                    help='B1.9: vonis torsi bergantung lintasan yang kebetulan '
                         'ditemukan OMPL (8/10 pada satu pose marginal). Minta '
                         'sampai N lintasan; TIAP rencana tetap disaring penuh, '
                         'dan hanya yang lolos SEMUA saringan dieksekusi.')
    ap.add_argument('--target2', metavar='X,Y,Z',
                    help='target arm_2 untuk --dual; ambil dari '
                         'scripts/dual_arm_targets.py, jangan dikarang')
    ap.add_argument('--arms', nargs='+', default=None, dest='arms_list',
                    choices=sorted(TOOL_FRAME),
                    help='LANGKAH 5 (docs/p1_g20_hw.md): lengan-lengan ini, '
                         'berurutan, dinilai pada SATU jendela 2.0 s bersama; '
                         '--target, --target2, ... sesuai urutan. TIAP rencana '
                         'disaring terhadap KETIGA lengan lain (dua gantry)')
    ap.add_argument('--quad', action='store_true',
                    help='= --arms arm_1 arm_2 arm_3 arm_4')
    ap.add_argument('--target3', metavar='X,Y,Z', help='target lengan ke-3')
    ap.add_argument('--target4', metavar='X,Y,Z', help='target lengan ke-4')
    ap.add_argument('--monitor-csv', default='',
                    help='<csv_log>_samples.csv milik reach_dwell_monitor. '
                         'Dibutuhkan untuk membedakan REACHED-NOT-HELD dari '
                         'EXEC-MISS: monitor TIDAK menerbitkan status per-sampel')
    ap.add_argument('--goal-joints', metavar='JSON', default=None,
                    help='G42: {sendi: rad} tujuan SENDI (tepat 6 sendi tiap '
                         'lengan --arms); --target tetap dinilai monitor, '
                         'GOAL-MISMATCH bila ujung > 3 mm / 2 deg darinya')
    ap.add_argument('--start-joints', metavar='JSON', default=None,
                    help='G42: keadaan awal TERTEMPAT {sendi: rad}; hanya DRY '
                         '(--plan-check, tanpa --move)')
    ap.add_argument('--goal-tol-deg', type=float, default=0.2,
                    help='G47: toleransi tujuan SENDI tugas (--goal-joints), '
                         'deg; retract REST tetap 0.2')
    ap.add_argument('--out-json', default=None,
                    help='G54: berkas hasil percobaan (bawaan /tmp/g20_step5.json dst.); runner paralel memberi satu '
                         'per probe supaya dua probe bersamaan tidak saling menimpa')
    ap.add_argument('--reserve-dir', default=None,
                    help='G54: direktori reservasi runner paralel; rencana disaring vs reservasi gantry lain '
                         'sebelum gerak (RESERVE-BLOCKED = tidak bergerak). Tanpa = perilaku lama')
    a = ap.parse_args()
    global TASK_GOAL_TOL_DEG, RESERVE_DIR
    RESERVE_DIR = a.reserve_dir
    if not 0.01 <= a.goal_tol_deg <= 0.2:
        ap.error('--goal-tol-deg harus di [0.01, 0.2]')
    TASK_GOAL_TOL_DEG = a.goal_tol_deg

    def _jdict(s, flag):
        try:
            d = json.loads(s)
            if not isinstance(d, dict) or not d:
                raise ValueError
            return {str(k): float(v) for k, v in d.items()}
        except (ValueError, TypeError):
            ap.error(f'{flag} harus objek JSON {{sendi: rad}}')

    a.goal_joints = _jdict(a.goal_joints, '--goal-joints') if a.goal_joints else None
    a.start_joints = _jdict(a.start_joints, '--start-joints') if a.start_joints else None
    if a.start_joints and (a.move or not a.plan_check):
        ap.error('--start-joints hanya dengan --plan-check TANPA --move '
                 '(rencana akan mulai di tempat lengan tidak berada)')

    def _xyz(s, flag):
        try:
            v = tuple(float(x) for x in s.split(','))
            if len(v) != 3:
                raise ValueError
            return v
        except ValueError:
            ap.error(f'{flag} harus X,Y,Z (meter, frame world)')

    fixed = _xyz(a.target, '--target') if a.target else None
    fixed2 = None
    arms = None
    if a.dual:
        if not (a.target and a.target2):
            ap.error('--dual butuh --target (arm_1) DAN --target2 (arm_2). '
                     'Pakai scripts/dual_arm_targets.py untuk keduanya.')
        fixed2 = _xyz(a.target2, '--target2')
        arms = ['arm_1', 'arm_2']
    a.screen_all = False
    if a.quad:
        a.arms_list = ['arm_1', 'arm_2', 'arm_3', 'arm_4']
    if a.arms_list:
        if a.dual or len(set(a.arms_list)) != len(a.arms_list):
            ap.error('--arms/--quad: tanpa --dual, tanpa lengan ganda')
        flags = ['--target', '--target2', '--target3', '--target4']
        vals = [a.target, a.target2, a.target3, a.target4]
        n = len(a.arms_list)
        if not all(vals[:n]) or any(vals[n:]):
            ap.error(f'--arms {a.arms_list} butuh tepat {flags[:n]}')
        a.dual = a.screen_all = True
        fixed = _xyz(a.target, '--target')
        fixed2 = [_xyz(v, f) for v, f in zip(vals[1:n], flags[1:n])]
        arms = list(a.arms_list)
    if (a.goal_joints or a.start_joints) and not a.arms_list:
        ap.error('--goal-joints/--start-joints hanya dengan --arms/--quad')
    if a.goal_joints:
        want = {f'{JOINT_PREFIX[x]}joint_{i}' for x in arms for i in range(1, 7)}
        if set(a.goal_joints) != want:
            ap.error(f'--goal-joints harus tepat {sorted(want)}; kurang '
                     f'{sorted(want - set(a.goal_joints))}, lebih '
                     f'{sorted(set(a.goal_joints) - want)}')

    if not a.move:
        print('\033[33mDRY RUN\033[0m -- tidak ada gerak. '
              'Tambahkan --move untuk menggerakkan (A6/S7).\n')

    rclpy.init()
    node = Probe(a.arm, a.approach, a.tau_max, a.move, a.topic, fixed, arms)
    src = f'TETAP {fixed} (persepsi dilewati)' if fixed else f'topik {a.topic}'
    if a.dual:
        print(f'probe LANGKAH {5 if a.screen_all else 3}: {" + ".join(arms)}, '
              f'{a.trials} percobaan, abort torsi {a.tau_max} N.m')
        print(f'sumber target: {arms[0]} {fixed}, {arms[1:]} {fixed2}')
        print('kriteria A1 TERKUNCI, N-lengan: SATU jendela 2.0 s di mana '
              'KEDUA lengan\nmemenuhi pos < 5 mm dan ori < 5 deg BERSAMAAN. '
              'Bergantian TIDAK dihitung.\n')
    else:
        print(f'probe: {a.arm} ({TOOL_FRAME[a.arm]}), {a.trials} percobaan, '
              f'approach {a.approach*1000:.0f} mm, abort torsi {a.tau_max} N.m')
        print(f'sumber target: {src}')
        print('kriteria A1 TERKUNCI: pos < 5 mm, ori < 5 deg, dwell 2.0 s '
              'KONTINU\n')

    tally = {'SUCCESS': 0, 'REACHED-NOT-HELD': 0, 'NO-PLAN': 0,
             'TORQUE-ABORT': 0, 'INVALID': 0}
    rows = []
    try:
        for i in range(1, a.trials + 1):
            if a.dual:
                row = dual_trial(node, a, i, arms, fixed2)
                rows.append(row)
                tally[row['verdict']] = tally.get(row['verdict'], 0) + 1
                if row['verdict'] == 'TORQUE-ABORT':
                    print('\033[31m  BERHENTI: batas torsi. A6/S4 -- red LED '
                          'butuh reset FISIK.\033[0m')
                    break
                continue
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
                             tau_max=a.tau_max,
                             attempts=a.plan_attempts) if a.plan_check else 'DRY'
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
        if a.dual:
            # CONCURRENT is the result. STAGGERED is reported next to it and
            # never added into it: both arms succeeding at different moments is
            # the prior-work behaviour this criterion was written to exclude.
            v = sum(tally.get(k, 0) for k in
                    ('CONCURRENT', 'STAGGERED', 'PARTIAL', 'NEITHER'))
            if v:
                print(f'\n  A1 N-lengan: {tally.get("CONCURRENT", 0)} / {v} '
                      'punya jendela BERSAMA')
                print(f'  BERGANTIAN (kedua lengan sukses, tapi TIDAK '
                      f'bersamaan): {tally.get("STAGGERED", 0)}  <- BUKAN sukses')
                print(f'  SEBAGIAN {tally.get("PARTIAL", 0)}, '
                      f'TIDAK ADA {tally.get("NEITHER", 0)}')
            mesin = {k: n for k, n in tally.items()
                     if k in ('HALTED', 'TORQUE-ABORT', 'INVALID', 'DRY') and n}
            print(f'  TIDAK VALID (mesin, A5, di LUAR penyebut): '
                  f'{mesin or "tidak ada"}')
        valid = sum(tally.get(k, 0) for k in
                    ('SUCCESS', 'REACHED-NOT-HELD', 'EXEC-MISS', 'NO-PLAN'))
        if valid and not a.dual:
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
            out = a.out_json if a.out_json else '/tmp/g20_step5.json' if a.screen_all else \
                '/tmp/g17_step3.json' if a.dual else '/tmp/g16_step2.json'
            # Appended, not overwritten: step 3 runs ONE pair per invocation, so
            # a fresh write would leave only the last trial of ten.
            old = []
            if a.dual and os.path.exists(out):
                try:
                    old = json.load(open(out))
                except ValueError:
                    old = []
                # One pair per invocation means `trial` is always 1 locally;
                # renumber so the accumulated file reads as trials 1..10.
                for k, r in enumerate(rows, len(old) + 1):
                    r['trial'] = k
            json.dump(old + rows, open(out, 'w'), indent=1)
            print(f'  -> {out}')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
