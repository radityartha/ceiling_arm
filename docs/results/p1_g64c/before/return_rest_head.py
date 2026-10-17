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
# G34: the MoveIt fallback is screened like a task plan (g31_screen TAU_MAX).
TAU_MAX_PLAN = 12.0


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


def _rest_line(start, steps=60):
    """Positions only, the same cosine as build() (screens need no timing)."""
    return [[s + (g - s) * 0.5 * (1.0 - math.cos(math.pi * k / steps))
             for s, g in zip(start, REST * (len(start) // 6))]
            for k in range(1, steps + 1)]


def plan_retract(g, arms, state, chk_same, chk_cross, env, plan_fn=None, log=print):
    """G34 env-aware retract, shared by this tool (HW) and g31_screen (plan-only).

    1. Both arms of gantry g, ONE straight joint line to REST (the G16 rule) --
       screened same-gantry, cross-gantry and against the frozen map.
    2. Refused and plan_fn given -> arm by arm, both orders: that arm's straight
       line if it screens CLEAR, else plan_fn(arm, {joint: REST}, placed), which
       must run every screen itself (reach_dwell_probe._plan_and_screen: S18,
       torque, environment) and return ('PLANNED', traj) or (refusal, None).
    G33 B6: the straight line home from the rack edge went THROUGH the rack.

    state: every CONFIG_JOINTS joint (measured or placed). Returns
    ('CLEAR', [(kind, arm|None, names, positions, traj|None), ...]) or
    (refusal, None). Positions are waypoint lists; traj is the MoveIt one."""
    rail = f't{g}_linear_joint'

    def screen(names, pts, placed):
        held = {n: v for n, v in placed.items() if n not in names}
        r = [('se-gantry', chk_same.screen_trajectory(names, pts, held)),
             ('antar-gantry', chk_cross.screen_trajectory(names, pts, held, only=f't{g}_a')),
             ('lingkungan', env.screen_trajectory(names, pts, placed))]
        for label, (v, d, pair, k) in r:
            what = pair if isinstance(pair, str) else f'{pair[0]} <-> {pair[1]}'
            log(f'  penyaring {label}: {v}, minimum {d * 1000:.1f} mm di titik {k}/{len(pts)} ({what})')
        return 'CLEAR' if all(x[1][0] == 'CLEAR' for x in r) else '/'.join(x[1][0] for x in r)

    todo = [x for x in arms
            if max(abs(state[f'{PREFIX[x]}joint_{i}'] - REST[i - 1]) for i in range(1, 7))
            >= math.radians(0.5)]
    if not todo:
        return 'CLEAR', []
    names = [f'{PREFIX[x]}joint_{i}' for x in todo for i in range(1, 7)]
    pts = _rest_line([state[n] for n in names])
    log(f'retract g{g} {todo} lurus (rel {state[rail]:.3f}):')
    v = screen(names, pts, state)
    if v == 'CLEAR':
        return 'CLEAR', [('lurus', None, names, pts, None)]
    if plan_fn is None:
        return f'LURUS-{v}', None
    for order in (todo, todo[::-1]) if len(todo) > 1 else (todo,):
        placed, segs, fail = dict(state), [], None
        for arm in order:
            nm = [f'{PREFIX[arm]}joint_{i}' for i in range(1, 7)]
            p1 = _rest_line([placed[n] for n in nm])
            log(f'retract {arm} sendiri lurus:')
            v1 = screen(nm, p1, placed)
            if v1 == 'CLEAR':
                segs.append(('lurus', arm, nm, p1, None))
            else:
                v2, traj = plan_fn(arm, dict(zip(nm, REST)), placed, not segs)
                log(f'retract {arm} via MoveIt: {v2}')
                if v2 != 'PLANNED':
                    fail = f'{arm}:{v1}+{v2}'
                    break
                jt = traj.joint_trajectory
                segs.append(('moveit', arm, list(jt.joint_names),
                             [list(q.positions) for q in jt.points], traj))
            placed.update(zip(segs[-1][2], segs[-1][3][-1]))
        if fail is None:
            return 'CLEAR', segs
        log(f'urutan {order} gagal: {fail}')
    return f'RETRACT-{fail}', None


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
    ap.add_argument('--no-plan', action='store_true',
                    help='G34: tanpa cadangan MoveIt (hanya garis lurus, perilaku G33).')
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
    # docs/p1_g32_rot_exec.md A2: BOTH gantry rotations, always. Without them
    # the cross screen started from URDF neutral and judged rot = 0 silently.
    held = [f'{PREFIX[x]}joint_{i}' for x in PREFIX if GANTRY[x] == og
            for i in range(1, 7)] + [f't{og}_linear_joint',
                                     't1_rotation_joint', 't2_rotation_joint']
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

    # S8/S9 + G33 + G34: plan_retract screens the straight line (same-gantry,
    # cross-gantry, environment) and, when the environment refuses it, finds a
    # way home arm by arm (MoveIt + every screen). Executed ONE segment at a
    # time, re-measured and re-planned before each.
    try:
        from interarm_collision import CrossGantryChecker, InterArmChecker
        chk = InterArmChecker(gantry=f'gantry_{g}')
        cross = CrossGantryChecker()
        # Strict names: q_from drops a name its model does not know, without a word.
        unknown = sorted({n for c, ns in ((chk, names + [rail] + partner),
                                          (cross, names + [rail] + held))
                          for n in ns if not c.model.existJointName(n)})
        if unknown:
            print(f'🔴 sendi tidak dikenal model penyaring {unknown} -- MENOLAK '
                  'menyaring dengan nama yang akan dibuang diam-diam.')
            node.destroy_node()
            rclpy.shutdown()
            return 1
        from env_collision import EnvChecker
        env = EnvChecker()
    except (ImportError, FileNotFoundError, KeyError, ValueError) as e:
        print(f'🔴 penyaring antar-lengan/lingkungan tidak dapat dimuat ({e}) '
              '-- MENOLAK (S9/G33).')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    probe = []

    def plan_fn(arm, goal, at, first):
        if a.no_plan:
            return 'MOVEIT-MATI', None
        from moveit_msgs.action import MoveGroup
        from reach_dwell_probe import Probe, _plan_and_screen
        if not probe:
            probe.append(Probe(arm, 0.0, TAU_MAX_PLAN, False, arms=list(PREFIX)))
            probe[0]._plan_ac = ActionClient(probe[0], MoveGroup, 'move_action')
        if not probe[0]._plan_ac.wait_for_server(timeout_sec=15.0):
            return 'NO-MOVE-GROUP', None
        # The FIRST segment executes, so it plans from the MEASURED state;
        # later ones are lookahead only (re-planned from measured before sending).
        return _plan_and_screen(arm, goal, probe[0], 0.002, 2.0, 0.15, 15.0, TAU_MAX_PLAN,
                                [x for x in PREFIX if x != arm],
                                start_joints=None if first else dict(at))

    def done(rc):
        for pn in probe:
            pn.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        return rc

    ac = None
    tau_all, tau_who, sent = 0.0, '', []
    for _ in range(4):
        v, segs = plan_retract(g, a.arms, state, chk, cross, env, plan_fn)
        if v != 'CLEAR':
            print(f'🔴 jalur PEMULIHAN ditolak ({v}) -- MENOLAK. Lengan tetap di tempat.')
            return done(1)
        if not segs:
            break
        print('rencana pulang: ' + ' -> '.join(
            f'{k}:{x or "+".join(sorted({n[:6] for n in nm_}))}' for k, x, nm_, *_ in segs))
        if not a.move:
            print(f'\nDRY RUN selesai: {len(segs)} segmen. Tidak ada yang dikirim.')
            return done(0)
        kind, arm, nm, pos, traj = segs[0]
        if kind == 'lurus':
            pts = build(nm, [state[n] for n in nm], REST * (len(nm) // 6), a.seconds, a.steps)
        else:
            pts = traj.joint_trajectory.points
        if ac is None:
            ctl = CONTROLLER.format(g)
            ac = ActionClient(node, FollowJointTrajectory, ctl)
            if not ac.wait_for_server(timeout_sec=15.0):
                print(f'🔴 {ctl} tidak ada. Controller aktif? '
                      '(enable_gantry_bridge WAJIB di perangkat keras nyata)')
                return done(1)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = nm
        goal_msg.trajectory.points = pts
        node.tau_peak, node.tau_worst = 0.0, ''
        print(f'\nmengirim segmen {kind} ({arm or "+".join(sorted({n[:6] for n in nm}))}): '
              f'{len(pts)} titik. '
              'TIDAK akan dibatalkan penjaga torsi (g16 B4.4).')
        fut = ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('🔴 goal DITOLAK controller.')
            return done(1)
        t_end = pts[-1].time_from_start
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=t_end.sec + 60.0)
        sent.append(kind)
        if node.tau_peak > tau_all:
            tau_all, tau_who = node.tau_peak, node.tau_worst
        state = node.wait_joints(list(state), seconds=1.0)
    else:
        print('🔴 4 putaran tanpa sampai REST -- berhenti.')
        return done(1)

    final = max(abs(node.joint_pos.get(n, float('nan')) - q)
                for n, q in zip(names, goal))
    print(f'selesai: galat akhir {math.degrees(final):.3f} deg, '
          f'torsi puncak {tau_all:.3f} N.m pada {tau_who}, segmen {sent}')
    if tau_all > TAU_REPORT_NM:
        print(f'\033[31m⚠️  torsi puncak {tau_all:.2f} N.m melewati '
              f'{TAU_REPORT_NM} N.m. TIDAK dibatalkan, disengaja (B4.4) -- '
              'tetapi catat ini dan periksa lengan.\033[0m')
    return done(0)

if __name__ == '__main__':
    sys.exit(main())
