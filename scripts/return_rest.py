#!/usr/bin/env python3
"""Return the gantry-1 arms to the HANGING rest pose, slowly, and FINISH.

WHY THIS FILE EXISTS. docs/p1_g16_hw.md calls this `return_rest` and the G17
prompt says it was "used in G16", but it was never committed: G16 wrote the
interpolation inline and kept only the name (p1_g16_hw.md:1120). The locked
protocol requires the arms back at rest before every trial, and step 3 needs it
for TWO of them, so it is written down here instead of a third time from memory.

TWO DELIBERATE DEPARTURES from reach_dwell_probe.move_to(), both measured:

  1. NO MoveIt plan. The goal configuration is known-good, and a planner is free
     to route to it through configurations that are not -- g16 B3.3 measured a
     10.4 N.m transient on a route nobody inspected. Straight joint-space
     interpolation from the MEASURED current pose is inspectable in full before
     it is sent.

  2. The torque guard WATCHES but never CANCELS. g16 B4.4: a recovery move
     aborted at 13.08 N.m and left the arm stranded 99.4 deg from rest, holding
     9.597 N.m static with the shoulder extended -- strictly worse than not
     guarding at all. A recovery move needs different rules from a trial move:
     slow, and completed. Over-torque here is reported loudly and afterwards.

Both arms travel in ONE trajectory rather than one after the other. They share
gantry_1_with_arm_controller (14 joints), so a second goal would preempt the
first; and rest is the pose where the two arms are furthest apart (622.7 mm),
so moving them together is the low-risk direction. The path is screened against
inter-arm collision anyway, every waypoint, before it is sent.

    python3 scripts/return_rest.py                 # DRY RUN, screens only
    python3 scripts/return_rest.py --move          # A6/S7: actually moves
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# p1_g16_hw.md B1.5, read off the real arm: j1 -0.4635, j2 +0.1071, j3 +0.1292,
# j4 -1.3865, j5 -0.1765, j6 +1.7388. Explicitly NOT FORBIDDEN_TUCK -- the arm
# hangs free here (A6/S1), and rest torque measured 0.069 N.m against tuck's
# 12.78 of 14.
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_',
          'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
# docs/p1_g20_hw.md: one controller per gantry, 14 joints each, so one call
# moves the arms of ONE gantry. Two gantries = two calls, never one goal each
# sent at once -- the second screen needs the first gantry's arms AT rest.
GANTRY = {'arm_1': 1, 'arm_2': 1, 'arm_3': 2, 'arm_4': 2}
CONTROLLER = '/gantry_{}_with_arm_controller/follow_joint_trajectory'
# Reported, never enforced -- see departure 2 above. 13.5 was g16's recovery
# bar; it is ABOVE the KA-75+ 12.0 nominal and must never be used as a trial
# value (g16 B4.2).
TAU_REPORT_NM = 13.5


class RestMover(Node):
    def __init__(self):
        super().__init__('return_rest')
        self.joint_pos = {}
        self.tau_peak = 0.0
        self.tau_worst = ''
        self.watch = ()
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)

    def _on_js(self, msg):
        # Two publishers, separate messages -- merge by name, never read one
        # message and call it the robot state.
        for n, p in zip(msg.name, msg.position or []):
            self.joint_pos[n] = float(p)
        for n, e in zip(msg.name, msg.effort or []):
            # g20: four arms live -- only the arms this call moves count.
            if self.watch and not n.startswith(self.watch):
                continue
            if not math.isnan(e) and abs(e) > self.tau_peak:
                self.tau_peak, self.tau_worst = abs(e), n

    def wait_joints(self, names, seconds=2.0):
        t0 = time.time()
        while time.time() - t0 < seconds or not all(n in self.joint_pos
                                                    for n in names):
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > seconds + 8.0:
                break
        return {n: self.joint_pos[n] for n in names if n in self.joint_pos}


def build(names, start, goal, seconds, steps):
    """Linear joint-space interpolation, start -> goal, over `seconds`.

    Cosine easing so the ends have no velocity step. Velocity matters here for
    a reason g16 measured: peak torque during these moves is dominated by the
    transient, not by the static hold, and a 30 s traverse makes the dynamic
    term negligible against gravity.
    """
    pts = []
    for k in range(1, steps + 1):
        f = 0.5 * (1.0 - math.cos(math.pi * k / steps))
        t = seconds * k / steps
        p = JointTrajectoryPoint()
        p.positions = [s + (g - s) * f for s, g in zip(start, goal)]
        p.velocities = [0.0] * len(names)
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(p)
    return pts


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arms', nargs='+', default=['arm_1', 'arm_2'],
                    choices=sorted(PREFIX))
    ap.add_argument('--seconds', type=float, default=30.0,
                    help='g16 B4.4: LAMBAT. Jangan turunkan untuk buru-buru.')
    ap.add_argument('--steps', type=int, default=60)
    ap.add_argument('--move', action='store_true',
                    help='A6/S7: benar-benar menggerakkan. Tanpa ini DRY RUN.')
    a = ap.parse_args()
    gs = {GANTRY[x] for x in a.arms}
    if len(gs) != 1:
        ap.error(f'--arms harus dari SATU gantry (satu controller); {a.arms} '
                 'mencakup dua. Panggil dua kali, gantry demi gantry.')
    g = gs.pop()
    og = 3 - g
    rail = f't{g}_linear_joint'

    if not a.move:
        print('\033[33mDRY RUN\033[0m -- tidak ada gerak. '
              'Tambahkan --move (A6/S7).\n')

    rclpy.init()
    node = RestMover()
    node.watch = tuple(PREFIX[x] for x in a.arms)
    names = [f'{PREFIX[x]}joint_{i}' for x in a.arms for i in range(1, 7)]
    # The OTHER gantry, held still, for the cross-gantry screen (g20).
    held = [f'{PREFIX[x]}joint_{i}' for x in PREFIX if GANTRY[x] == og
            for i in range(1, 7)] + [f't{og}_linear_joint']
    # A single-arm call holds its PARTNER still; before g20 the partner was
    # left at URDF neutral (all joints 0) in the screen instead of measured.
    partner = [f'{PREFIX[x]}joint_{i}' for x in PREFIX
               if GANTRY[x] == g and x not in a.arms for i in range(1, 7)]
    state = node.wait_joints(names + [rail] + held + partner)
    missing = [n for n in names + [rail] + held + partner if n not in state]
    if missing:
        print(f'🔴 /joint_states tidak lengkap, hilang {missing} -- MENOLAK. '
              'Menginterpolasi dari pose yang tidak diketahui adalah persis '
              'cara membuat gerak yang tak seorang pun memeriksanya.')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    start = [state[n] for n in names]
    goal = REST * len(a.arms)
    err = max(abs(s - g) for s, g in zip(start, goal))
    print(f'lengan: {a.arms}   galat terbesar dari rest: '
          f'{math.degrees(err):.2f} deg')
    print(f'rel {rail} = {state[rail]:.6f} m')
    if math.degrees(err) < 0.5:
        print('sudah di rest (< 0.5 deg) -- tidak ada yang perlu dikerjakan.')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    pts = build(names, start, goal, a.seconds, a.steps)

    # S8/S9. Both arms move in this trajectory, so the screen gets both sets of
    # joints per waypoint and only the rail comes from the measured state.
    try:
        from interarm_collision import CrossGantryChecker, InterArmChecker
        res = []
        chk = InterArmChecker(gantry=f'gantry_{g}')
        res.append(('se-gantry', chk.screen_trajectory(
            names, [p.positions for p in pts],
            {n: state[n] for n in [rail] + partner})))
        # g20: the other gantry's arms, HELD at their measured pose. Pairs
        # restricted to the arms of THIS gantry (only='t{g}_a').
        cross = CrossGantryChecker()
        res.append(('antar-gantry', cross.screen_trajectory(
            names, [p.positions for p in pts],
            {n: state[n] for n in [rail] + held}, only=f't{g}_a')))
        for label, (v, d, pair, k) in res:
            print(f'penyaring {label}: {v}, minimum {d * 1000:.1f} mm '
                  f'di titik {k}/{len(pts)} ({pair[0]} <-> {pair[1]})')
        v = 'CLEAR' if all(r[1][0] == 'CLEAR' for r in res) else 'NOT-CLEAR'
        if v != 'CLEAR':
            print('🔴 jalur PEMULIHAN sendiri bertabrakan -- MENOLAK. '
                  'Pulihkan satu lengan lebih dulu (--arms arm_1).')
            node.destroy_node()
            rclpy.shutdown()
            return 1
    except ImportError:
        print('🔴 penyaring antar-lengan tidak dapat dimuat -- MENOLAK (S9).')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not a.move:
        print(f'\nDRY RUN selesai: {len(pts)} titik, {a.seconds:.0f} s, '
              f'{len(names)} sendi. Tidak ada yang dikirim.')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    ctl = CONTROLLER.format(g)
    ac = ActionClient(node, FollowJointTrajectory, ctl)
    if not ac.wait_for_server(timeout_sec=15.0):
        print(f'🔴 {ctl} tidak ada. Controller aktif? '
              '(enable_gantry_bridge WAJIB di perangkat keras nyata)')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = names
    goal_msg.trajectory.points = pts
    node.tau_peak, node.tau_worst = 0.0, ''
    print(f'\nmengirim: {len(pts)} titik selama {a.seconds:.0f} s. '
          'TIDAK akan dibatalkan penjaga torsi (g16 B4.4).')

    fut = ac.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('🔴 goal DITOLAK controller.')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=a.seconds + 60.0)

    node.wait_joints(names, seconds=1.0)
    final = max(abs(node.joint_pos.get(n, float('nan')) - g)
                for n, g in zip(names, goal))
    print(f'selesai: galat akhir {math.degrees(final):.3f} deg, '
          f'torsi puncak {node.tau_peak:.3f} N.m pada {node.tau_worst}')
    if node.tau_peak > TAU_REPORT_NM:
        print(f'\033[31m⚠️  torsi puncak {node.tau_peak:.2f} N.m melewati '
              f'{TAU_REPORT_NM} N.m. TIDAK dibatalkan, disengaja (B4.4) -- '
              'tetapi catat ini dan periksa lengan.\033[0m')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
