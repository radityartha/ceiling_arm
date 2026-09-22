"""G22 TRAVERSE: t{g}_linear_joint -> GOAL via the gantry bridge, gantry 1 OR 2.

From p1_g19/rail_to.py (gantry 1 only, 0.550/0.950). docs/p1_g22_hw.md A5:
  S12  REFUSES unless BOTH arms of gantry g are within 0.5 deg of REST
  S13' GOAL must be in the locked plan's rails for gantry g (+ its p0), <= 1.600 m
  S23  both rotations read |rot| <= 0.5 deg
  S24  sweep screen: rail of g sampled <= 10 mm from start to GOAL, its arms at
       MEASURED (REST) pose, everything else HELD at MEASURED pose,
       CrossGantryChecker margin 50 mm. Not CLEAR / cannot screen -> REFUSE.
  S17  exit 3 if any arm drifts > 0.5 deg, the OTHER rail moves > 2 mm, or the
       final rail error > 2 mm
Traverse time = encoder first moves > 0.5 mm -> rail STOPS (unchanged < 0.05 mm
for 1.0 s). NOT the 0.52 mm wait of g19 (B3 (2)-(3): bridge debounce 1.0 mm,
the 45 s wait contaminated 2/10 trials).
DRY RUN unless --move. Last stdout line = one JSON record.

    python3 rail_to_g.py --gantry 2 --seed 0 0.45          # DRY: S12/S23/S24 only
    python3 rail_to_g.py --gantry 2 --seed 0 0.45 --move
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

sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g22_plan as P  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--gantry', type=int, choices=(1, 2), required=True)
ap.add_argument('--seed', type=int, required=True, help='locked instance (A2)')
ap.add_argument('--plan', default=None, help='candidates json (default g22_candidates.json)')
ap.add_argument('goal', type=float)
ap.add_argument('--move', action='store_true')
a = ap.parse_args()
G, GOAL = a.gantry, a.goal
J, OJ = f't{G}_linear_joint', f't{3 - G}_linear_joint'
ARMS = [f'{P.PREFIX[x]}joint_{j}' for x in P.ARMS[G] for j in range(1, 7)]
ALL_ARMS = [f'{p}joint_{j}' for p in P.PREFIX.values() for j in range(1, 7)]
ROTS = ['t1_rotation_joint', 't2_rotation_joint']
row = P.load_row(a.seed, a.plan)
ok_goals = P.allowed_rails(row, G)
if round(GOAL, 3) not in ok_goals or not 0.0 <= GOAL <= P.RAIL_MAX:
    print(f'REFUSE: S13\' -- gantry {G} hanya {ok_goals} (jadwal seed {a.seed}), <= {P.RAIL_MAX}')
    sys.exit(1)

rclpy.init()
n = Node(f'g22_rail_to_g{G}')
pos, eff, log = {}, {}, []


def cb(m):
    t = time.time()
    for i, k in enumerate(m.name):
        pos[k] = m.position[i]
        if i < len(m.effort):
            eff[k] = m.effort[i]
    if J in m.name:
        log.append((t, pos[J], pos.get(OJ), {k: pos.get(k) for k in ALL_ARMS},
                    {k: abs(eff.get(k, 0.0)) for k in ARMS}))


n.create_subscription(JointState, '/joint_states', cb, 200)
need = ALL_ARMS + [J, OJ] + ROTS
t0 = time.time()
while time.time() - t0 < 1.0 or not all(k in pos for k in need):
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 > 10:
        print(f'REFUSE: /joint_states tidak lengkap, hilang {sorted(set(need) - set(pos))}')
        sys.exit(1)
dev = max(abs(pos[f'{P.PREFIX[x]}joint_{j}'] - P.REST[j - 1]) for x in P.ARMS[G] for j in range(1, 7))
print(f'S12: lengan gantry {G} maks |q - REST| = {math.degrees(dev):.3f} deg (batas 0.5)')
if math.degrees(dev) >= 0.5:
    print('REFUSE: S12 -- lengan BELUM di REST, gantry tidak digerakkan')
    sys.exit(1)
rot = {k: math.degrees(pos[k]) for k in ROTS}
print(f'S23: rotasi t1 {rot[ROTS[0]]:+.3f} deg, t2 {rot[ROTS[1]]:+.3f} deg (batas 0.5)')
if any(abs(v) > 0.5 for v in rot.values()):
    print('REFUSE: S23 -- rotasi bukan 0; penyaring S18/S24 tidak memuat sendi rotasi')
    sys.exit(1)
s = pos[J]
state = {k: pos[k] for k in need}
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    try:
        from interarm_collision import CrossGantryChecker
        chk = CrossGantryChecker()
        sw = P.sweep_screen(chk, G, s, GOAL, state)
    except Exception as e:                                   # noqa: BLE001
        print(f'REFUSE: S24 -- penyaring sapuan tidak bisa jalan ({e!r})')
        sys.exit(1)
print(f'S24: sapuan {J} {s:.4f} -> {GOAL:.4f} m, lengan lain TERUKUR: {sw[0]}, '
      f'min {sw[1] * 1000:.1f} mm di titik {sw[3]} ({sw[2][0]} <-> {sw[2][1]})')
if sw[0] != 'CLEAR':
    print('REFUSE: S24 -- sapuan rel tidak bebas (margin 50 mm)')
    sys.exit(1)
D = abs(GOAL - s)
SECS = P.t_cmd(D)
STEPS = max(10, int(SECS * 2))
arm0 = {k: pos[k] for k in ALL_ARMS}
o0 = pos[OJ]
print(f'{J}: {s:.6f} -> {GOAL:.6f} m ({(GOAL - s) * 1000:+.1f} mm), T_cmd {SECS:.2f} s, '
      f'T_lin model {0.29 + D / P.V_LIN:.2f} s')
pts = []
for k in range(1, STEPS + 1):
    f = 0.5 * (1 - math.cos(math.pi * k / STEPS))
    t = SECS * k / STEPS
    p = JointTrajectoryPoint()
    p.positions = [s + (GOAL - s) * f]
    p.velocities = [0.0]
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    pts.append(p)
print(f'setpoint pertama {abs(pts[0].positions[0] - s) * 1000:.2f} mm (arm_tol 5 mm)')
if not a.move:
    print('DRY RUN: tidak ada yang dikirim.')
    print(json.dumps(dict(dry=True, gantry=G, start=round(s, 6), goal=GOAL, sweep=sw[0],
                          sweep_min_mm=round(sw[1] * 1000, 1), T_cmd=round(SECS, 3))))
    sys.exit(0)
ctl = f'/gantry_{G}_with_arm_controller/follow_joint_trajectory'
ac = ActionClient(n, FollowJointTrajectory, ctl)
assert ac.wait_for_server(timeout_sec=15), f'{ctl} tidak ada'
g = FollowJointTrajectory.Goal()
g.trajectory.joint_names = [J]
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
# bridge chases the JTC setpoint and debounces at 1.0 mm: wait for the ENCODER to STOP
end, last, t_still = time.time() + 30.0, pos[J], time.time()
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if abs(pos[J] - last) > 0.00005:
        last, t_still = pos[J], time.time()
    elif time.time() - t_still >= 1.0 and abs(pos[J] - s) > 0.0005:
        break
mv = [r for r in log if abs(r[1] - s) > 0.0005]
t_move = mv[0][0] if mv else None
t_stop = None
for k in range(len(log) - 1, 0, -1):          # last sample where the rail still changed
    if abs(log[k][1] - log[k - 1][1]) > 0.00005:
        t_stop = log[k][0]
        break
drift = max((abs(r[3][k] - arm0[k]) for r in log for k in ALL_ARMS if r[3][k] is not None),
            default=float('nan'))
odrift = max((abs(r[2] - o0) for r in log if r[2] is not None), default=float('nan'))
win = [r for r in log if t_move and t_stop and t_move <= r[0] <= t_stop]
tpk = max(((v, k) for r in win for k, v in r[4].items()), default=(float('nan'), ''))
rec = dict(gantry=G, goal=GOAL, start=round(s, 6), end=round(pos[J], 6),
           err_mm=round((pos[J] - GOAL) * 1000, 3), T_cmd=round(SECS, 3),
           T_lin_model=round(0.29 + D / P.V_LIN, 3), t_send=t_send, t_jtc_done=t_jtc,
           t_first_move=t_move, t_stop=t_stop,
           t_traverse=round(t_stop - t_move, 3) if (t_stop and t_move) else None,
           arm_drift_deg=round(math.degrees(drift), 4), other_rail_drift_mm=round(odrift * 1000, 3),
           arm_tau_peak=round(tpk[0], 3), arm_tau_joint=tpk[1], sweep_min_mm=round(sw[1] * 1000, 1),
           jtc_error_code=str(ec))
print(f'akhir: {J} = {pos[J]:.6f} m (galat {rec["err_mm"]:+.2f} mm); t_traverse {rec["t_traverse"]} s '
      f'(T_cmd {SECS:.2f}, T_lin {rec["T_lin_model"]}); lengan (keempat) bergeser maks '
      f'{rec["arm_drift_deg"]:.4f} deg; rel lain {rec["other_rail_drift_mm"]:.3f} mm')
print(json.dumps(rec))
sys.exit(3 if (rec['arm_drift_deg'] > 0.5 or abs(rec['err_mm']) > 2.0
               or rec['other_rail_drift_mm'] > 2.0) else 0)
