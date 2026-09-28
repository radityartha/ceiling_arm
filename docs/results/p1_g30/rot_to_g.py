"""G30 ROTATION: t{g}_rotation_joint -> GOAL_DEG via the gantry bridge, gantry 1 OR 2, arms REST.

From p1_g22/rail_to_g.py + p1_g18/rot_home.py. docs/p1_g30_rot_hw.md A1:
  S12  REFUSES unless BOTH arms of gantry g are within 0.5 deg of REST
  S23' |GOAL| <= 10.0 deg; measured |rot g| <= 10.5 deg; the OTHER gantry's |rot| <= 0.5 deg
  S28  sweep_rot(rect) (g29_rot_screen, TRUE hulls, strict names) from the MEASURED state -- all 24 arm
       joints, both rails, BOTH rotations from /joint_states -- (lin, rot) -> (lin, GOAL). Not CLEAR /
       cannot screen -> REFUSE.
  rc 3 if |err| > 1.0 deg, any arm drifts > 0.5 deg, either rail moves > 2 mm, the other rotation moves
       > 0.5 deg, or any arm torque > 14 N.m
Rotation time = encoder first moves > 0.05 deg -> last change (encoder unchanged < 0.01 deg for 1.0 s).
DRY RUN unless --move. Last stdout line = one JSON record.

    python3 rot_to_g.py --gantry 1 10.0            # DRY: S12/S23'/S28 only
    python3 rot_to_g.py --gantry 1 10.0 --move
"""
import argparse
import json
import math
import os
import sys
import time
import warnings

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
import g22_plan as P  # noqa: E402

V_ROT = 10.0                                  # deg/s, bridge.rotate_speed 1000 (p1_g3 C1)
ap = argparse.ArgumentParser()
ap.add_argument('--gantry', type=int, choices=(1, 2), required=True)
ap.add_argument('goal', type=float, help='deg')
ap.add_argument('--move', action='store_true')
a = ap.parse_args()
G, GOAL_DEG = a.gantry, a.goal
RJ, ORJ = f't{G}_rotation_joint', f't{3 - G}_rotation_joint'
LJ, OLJ = f't{G}_linear_joint', f't{3 - G}_linear_joint'
ALL_ARMS = [f'{p}joint_{j}' for p in P.PREFIX.values() for j in range(1, 7)]
if abs(GOAL_DEG) > 10.0:
    print(f"REFUSE: S23' -- target {GOAL_DEG:+.3f} deg, batas |rot| <= 10.0")
    sys.exit(1)

rclpy.init()
n = Node(f'g30_rot_to_g{G}')
pos, eff, log = {}, {}, []


def cb(m):
    t = time.time()
    for i, k in enumerate(m.name):
        pos[k] = m.position[i]
        if i < len(m.effort):
            eff[k] = m.effort[i]
    if RJ in m.name:
        log.append((t, pos[RJ], pos.get(ORJ), pos.get(LJ), pos.get(OLJ),
                    {k: pos.get(k) for k in ALL_ARMS}, {k: abs(eff.get(k, 0.0)) for k in ALL_ARMS}))


n.create_subscription(JointState, '/joint_states', cb, 200)
need = ALL_ARMS + [LJ, OLJ, RJ, ORJ]
t0 = time.time()
while time.time() - t0 < 1.0 or not all(k in pos for k in need):
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 > 10:
        print(f'REFUSE: /joint_states tidak lengkap, hilang {sorted(set(need) - set(pos))}')
        sys.exit(1)
dev = {x: max(abs(pos[f'{P.PREFIX[x]}joint_{j}'] - P.REST[j - 1]) for j in range(1, 7)) for x in P.PREFIX}
print('S12: maks |q - REST| ' + ', '.join(f'{x} {math.degrees(v):.3f}' for x, v in dev.items())
      + f' deg (batas 0.5 untuk gantry {G})')
if any(math.degrees(dev[x]) >= 0.5 for x in P.ARMS[G]):
    print('REFUSE: S12 -- lengan BELUM di REST, gantry tidak diputar')
    sys.exit(1)
s_deg, o_deg = math.degrees(pos[RJ]), math.degrees(pos[ORJ])
print(f"S23': {RJ} {s_deg:+.3f} deg (batas 10.5), {ORJ} {o_deg:+.3f} deg (batas 0.5); target {GOAL_DEG:+.3f}")
if abs(s_deg) > 10.5 or abs(o_deg) > 0.5:
    print("REFUSE: S23' -- rotasi terukur di luar batas sesi (satu gantry, <= 10 deg)")
    sys.exit(1)
state = {k: pos[k] for k in need}
lin = pos[LJ]
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    try:
        import g29_rot_screen as R
        chk = R.RotCrossChecker()
        sw = R.sweep_rot(chk, G, (lin, pos[RJ]), (lin, math.radians(GOAL_DEG)), state)
    except Exception as e:                                   # noqa: BLE001
        print(f'REFUSE: S28 -- penyaring sapuan tidak bisa jalan ({e!r})')
        sys.exit(1)
print(f"S28: sapuan {RJ} {s_deg:+.3f} -> {GOAL_DEG:+.3f} deg (rel {lin:.4f} m, sisanya TERUKUR): {sw['verdict']}, "
      f"min {sw['d'] * 1000:.1f} mm {sw['pair']} di {math.degrees(sw['at'][1]):+.2f} deg; "
      f"lengan {sw['d_arm'] * 1000:.1f}, SS {sw['d_ss'] * 1000:.1f}, n {sw['n']}")
if sw['verdict'] != 'CLEAR':
    print('REFUSE: S28 -- sapuan rotasi tidak bebas (margin 50 mm)')
    sys.exit(1)
s = pos[RJ]
GOAL = math.radians(GOAL_DEG)
D = abs(GOAL_DEG - s_deg)
SECS = max(3.0, math.pi * D / (2 * 0.9 * V_ROT))
STEPS = max(10, int(SECS * 2))
arm0 = {k: pos[k] for k in ALL_ARMS}
l0, ol0, or0 = pos[LJ], pos[OLJ], pos[ORJ]
T_ROT = 0.26 + D / V_ROT
print(f'{RJ}: {s_deg:+.3f} -> {GOAL_DEG:+.3f} deg ({GOAL_DEG - s_deg:+.3f}), T_cmd {SECS:.2f} s, T_rot model {T_ROT:.2f} s')
pts = []
for k in range(1, STEPS + 1):
    f = 0.5 * (1 - math.cos(math.pi * k / STEPS))
    t = SECS * k / STEPS
    p = JointTrajectoryPoint()
    p.positions = [s + (GOAL - s) * f]
    p.velocities = [0.0]
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    pts.append(p)
print(f'setpoint pertama {math.degrees(abs(pts[0].positions[0] - s)):.3f} deg (arm_tol 1.0 deg)')
if not a.move:
    print('DRY RUN: tidak ada yang dikirim.')
    print(json.dumps(dict(dry=True, gantry=G, start_deg=round(s_deg, 4), goal_deg=GOAL_DEG, sweep=sw['verdict'],
                          sweep_min_mm=round(sw['d'] * 1000, 1), sweep_arm_mm=round(sw['d_arm'] * 1000, 1),
                          T_cmd=round(SECS, 3))))
    sys.exit(0)
ctl = f'/gantry_{G}_with_arm_controller/follow_joint_trajectory'
ac = ActionClient(n, FollowJointTrajectory, ctl)
assert ac.wait_for_server(timeout_sec=15), f'{ctl} tidak ada'
g = FollowJointTrajectory.Goal()
g.trajectory.joint_names = [RJ]
g.trajectory.points = pts
t_send = time.time()
log.clear()
f = ac.send_goal_async(g)
rclpy.spin_until_future_complete(n, f, timeout_sec=20)
gh = f.result()
if gh is None or not gh.accepted:
    print('goal DITOLAK')
    sys.exit(1)
rf = gh.get_result_async()
rclpy.spin_until_future_complete(n, rf, timeout_sec=SECS + 40)
t_jtc = time.time()
ec = rf.result().result.error_code if rf.result() else 'timeout'
print('JTC error_code:', ec)
# bridge chases the JTC setpoint (debounce 0.3 deg, deadband 0.5 deg): wait for the ENCODER to STOP
STILL, MOVED = math.radians(0.01), math.radians(0.05)
end, last, t_still = time.time() + 30.0, pos[RJ], time.time()
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if abs(pos[RJ] - last) > STILL:
        last, t_still = pos[RJ], time.time()
    elif time.time() - t_still >= 1.0 and abs(pos[RJ] - s) > MOVED:
        break
mv = [r for r in log if abs(r[1] - s) > MOVED]
t_move = mv[0][0] if mv else None
t_stop = None
for k in range(len(log) - 1, 0, -1):          # last sample where the rotation still changed
    if abs(log[k][1] - log[k - 1][1]) > 1e-6:
        t_stop = log[k][0]
        break
drift = max((abs(r[5][k] - arm0[k]) for r in log for k in ALL_ARMS if r[5][k] is not None),
            default=float('nan'))
odrift = max((abs(r[2] - or0) for r in log if r[2] is not None), default=float('nan'))
ldrift = max((abs(r[3] - l0) for r in log if r[3] is not None), default=float('nan'))
oldrift = max((abs(r[4] - ol0) for r in log if r[4] is not None), default=float('nan'))
win = [r for r in log if t_move and t_stop and t_move <= r[0] <= t_stop]
tpk = max(((v, k) for r in win for k, v in r[6].items()), default=(float('nan'), ''))
rec = dict(gantry=G, goal_deg=GOAL_DEG, start_deg=round(s_deg, 4), end_deg=round(math.degrees(pos[RJ]), 4),
           err_deg=round(math.degrees(pos[RJ]) - GOAL_DEG, 4), T_cmd=round(SECS, 3), T_rot_model=round(T_ROT, 3),
           t_send=t_send, t_jtc_done=t_jtc, t_first_move=t_move, t_stop=t_stop,
           t_rot=round(t_stop - t_move, 3) if (t_stop and t_move) else None,
           arm_drift_deg=round(math.degrees(drift), 4), other_rot_drift_deg=round(math.degrees(odrift), 4),
           rail_drift_mm=round(ldrift * 1000, 3), other_rail_drift_mm=round(oldrift * 1000, 3),
           arm_tau_peak=round(tpk[0], 3), arm_tau_joint=tpk[1], sweep_min_mm=round(sw['d'] * 1000, 1),
           sweep_arm_mm=round(sw['d_arm'] * 1000, 1), sweep_pair=sw['pair'], jtc_error_code=str(ec), n_log=len(log))
print(f"akhir: {RJ} = {rec['end_deg']:+.4f} deg (galat {rec['err_deg']:+.4f}); t_rot {rec['t_rot']} s "
      f"(T_cmd {SECS:.2f}, T_rot {T_ROT:.2f}); lengan (keempat) bergeser maks {rec['arm_drift_deg']:.4f} deg; "
      f"rel sendiri {rec['rail_drift_mm']:.3f} mm, rel lain {rec['other_rail_drift_mm']:.3f} mm, "
      f"rot lain {rec['other_rot_drift_deg']:.4f} deg; torsi puncak {rec['arm_tau_peak']} {rec['arm_tau_joint']}")
print(json.dumps(rec))
sys.exit(3 if (not log or abs(rec['err_deg']) > 1.0 or rec['arm_drift_deg'] > 0.5 or rec['rail_drift_mm'] > 2.0
               or rec['other_rail_drift_mm'] > 2.0 or rec['other_rot_drift_deg'] > 0.5
               or not rec['arm_tau_peak'] <= 14.0) else 0)
