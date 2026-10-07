"""G32 POSE: (t{g}_linear_joint, t{g}_rotation_joint) -> (LIN, ROT_DEG) in ONE JTC goal via the gantry bridge.

From p1_g22/rail_to_g.py + p1_g30/rot_to_g.py. docs/p1_g32_rot_exec.md A3:
  S12  REFUSES unless BOTH arms of gantry g are within 0.5 deg of REST
  S13" (LIN, ROT) must be a stop of gantry g in the locked plan (seed, variant) or p0 (0, 0); LIN in [0, 1.600]
  S23" |ROT| <= 10.0 deg; measured |rot g| <= 10.5; measured |rot other| <= 10.5 (R10, G29 B3 envelope)
  S28  sweep_rot(RotCrossChecker, rect) from the MEASURED state -- all 24 arm joints, both rails, BOTH
       rotations, strict names -- (lin, rot) -> (LIN, ROT). Not CLEAR / cannot screen -> REFUSE.
  rc 3 if |err| > 2.0 mm or > 1.0 deg, any arm drifts > 0.5 deg, the other rail moves > 2 mm, the other
       rotation moves > 0.5 deg, or any arm torque > 14 N.m
Both axes go from MEASURED to the plan target in one cosine, T_cmd = max(T_cmd_lin, T_cmd_rot) (an axis that
does not move contributes 0). The bridge sends lin + rot as ONE absolute target (A0.1).
DRY RUN unless --move. Last stdout line = one JSON record.

    python3 pose_to_g.py --gantry 2 --seed 36 --plan g32_candidates.json 0.60 -10      # DRY
    python3 pose_to_g.py --gantry 2 --seed 36 --plan g32_candidates.json 0.60 -10 --move
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
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
import g22_plan as P  # noqa: E402

V_ROT = 10.0                                  # deg/s, bridge.rotate_speed 1000 (p1_g3 C1)
ap = argparse.ArgumentParser()
ap.add_argument('--gantry', type=int, choices=(1, 2), required=True)
ap.add_argument('--seed', type=int, required=True)
ap.add_argument('--plan', default=os.path.join(HERE, 'g32_candidates.json'))
ap.add_argument('--variant', default='R10')
ap.add_argument('lin', type=float, help='m')
ap.add_argument('rot', type=float, help='deg')
ap.add_argument('--move', action='store_true')
a = ap.parse_args()
G, GL, GR_DEG = a.gantry, a.lin, a.rot
LJ, OLJ = f't{G}_linear_joint', f't{3 - G}_linear_joint'
RJ, ORJ = f't{G}_rotation_joint', f't{3 - G}_rotation_joint'
ALL_ARMS = [f'{p}joint_{j}' for p in P.PREFIX.values() for j in range(1, 7)]
row = {r['seed']: r for r in json.load(open(a.plan))}[a.seed]
ok = {(round(row['p0'][str(G)], 3), 0.0)} | {(round(st['rail_m'], 3), round(st['rot_deg'], 3))
                                             for st in row[a.variant]['schedule'][str(G)]}
if (round(GL, 3), round(GR_DEG, 3)) not in ok or not 0.0 <= GL <= P.RAIL_MAX:
    print(f'REFUSE: S13" -- gantry {G} hanya {sorted(ok)} (jadwal seed {a.seed} {a.variant}), rel <= {P.RAIL_MAX}')
    sys.exit(1)
if abs(GR_DEG) > 10.0:
    print(f'REFUSE: S23" -- target rot {GR_DEG:+.3f} deg, batas |rot| <= 10.0')
    sys.exit(1)

rclpy.init()
n = Node(f'g32_pose_to_g{G}')
pos, eff, log = {}, {}, []


def cb(m):
    t = time.time()
    for i, k in enumerate(m.name):
        pos[k] = m.position[i]
        if i < len(m.effort):
            eff[k] = m.effort[i]
    if LJ in m.name or RJ in m.name:
        log.append((t, pos.get(LJ), pos.get(RJ), pos.get(OLJ), pos.get(ORJ),
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
    print('REFUSE: S12 -- lengan BELUM di REST, gantry tidak digerakkan')
    sys.exit(1)
sl, sr = pos[LJ], pos[RJ]
s_deg, o_deg = math.degrees(sr), math.degrees(pos[ORJ])
print(f'S23": {RJ} {s_deg:+.3f} deg (batas 10.5), {ORJ} {o_deg:+.3f} deg (batas 10.5); target {GR_DEG:+.3f}')
if abs(s_deg) > 10.5 or abs(o_deg) > 10.5:
    print('REFUSE: S23" -- rotasi terukur di luar R10')
    sys.exit(1)
state = {k: pos[k] for k in need}
GR = math.radians(GR_DEG)
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    try:
        import g29_rot_screen as R
        chk = R.RotCrossChecker()
        unknown = [k for k in state if not chk.model.existJointName(k)]
        if unknown:
            raise KeyError(f'sendi tidak dikenal model {unknown}')
        sw = R.sweep_rot(chk, G, (sl, sr), (GL, GR), state)
    except Exception as e:                                   # noqa: BLE001
        print(f'REFUSE: S28 -- penyaring sapuan tidak bisa jalan ({e!r})')
        sys.exit(1)
print(f"S28: sapuan g{G} ({sl:.4f} m, {s_deg:+.3f} deg) -> ({GL:.4f}, {GR_DEG:+.3f}), sisanya TERUKUR: "
      f"{sw['verdict']}, min {sw['d'] * 1000:.1f} mm {sw['pair']} di ({sw['at'][0]:.3f}, "
      f"{math.degrees(sw['at'][1]):+.2f}); lengan {sw['d_arm'] * 1000:.1f}, SS {sw['d_ss'] * 1000:.1f}, n {sw['n']}")
if sw['verdict'] != 'CLEAR':
    print('REFUSE: S28 -- sapuan (lin, rot) tidak bebas (margin 50 mm)')
    sys.exit(1)
# G33 (docs/p1_g33_map.md): the same (lin, rot) sweep against the frozen camera
# map. S28 is robot-vs-robot only; G32-HW hit a rack. Missing map -> REFUSE.
with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    try:
        sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
        from env_collision import EnvChecker
        ev, ed, eg, ek = EnvChecker().screen_trajectory(
            [f't{G}_linear_joint', f't{G}_rotation_joint'],
            R.rect_points((sl, sr), (GL, GR)), state)
    except Exception as e:                                   # noqa: BLE001
        print(f'REFUSE: G33 -- penyaring lingkungan tidak bisa jalan ({e!r})')
        sys.exit(1)
print(f'G33: sapuan g{G} vs peta lingkungan: {ev}, min {ed * 1000:.1f} mm ({eg}) di titik {ek}')
if ev != 'CLEAR':
    print('REFUSE: G33 -- sapuan (lin, rot) terlalu dekat lingkungan (margin 50 mm)')
    sys.exit(1)
DL, DR = abs(GL - sl), abs(GR_DEG - s_deg)
T_LIN = P.t_cmd(DL) if DL > 1e-6 else 0.0
T_ROT = max(3.0, math.pi * DR / (2 * 0.9 * V_ROT)) if DR > 1e-6 else 0.0
SECS = max(T_LIN, T_ROT, 3.0)
STEPS = max(10, int(SECS * 2))
arm0 = {k: pos[k] for k in ALL_ARMS}
ol0, or0 = pos[OLJ], pos[ORJ]
print(f'g{G}: lin {sl:.6f} -> {GL:.6f} m ({(GL - sl) * 1000:+.1f} mm), rot {s_deg:+.3f} -> {GR_DEG:+.3f} deg '
      f'({GR_DEG - s_deg:+.3f}); T_cmd {SECS:.2f} s (lin {T_LIN:.2f}, rot {T_ROT:.2f})')
pts = []
for k in range(1, STEPS + 1):
    f = 0.5 * (1 - math.cos(math.pi * k / STEPS))
    t = SECS * k / STEPS
    p = JointTrajectoryPoint()
    p.positions = [sl + (GL - sl) * f, sr + (GR - sr) * f]
    p.velocities = [0.0, 0.0]
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    pts.append(p)
print(f'setpoint pertama {abs(pts[0].positions[0] - sl) * 1000:.2f} mm / '
      f'{math.degrees(abs(pts[0].positions[1] - sr)):.3f} deg (arm_tol 5 mm / 1.0 deg)')
if not a.move:
    print('DRY RUN: tidak ada yang dikirim.')
    print(json.dumps(dict(dry=True, gantry=G, start=[round(sl, 6), round(s_deg, 4)], goal=[GL, GR_DEG],
                          sweep=sw['verdict'], sweep_min_mm=round(sw['d'] * 1000, 1),
                          sweep_arm_mm=round(sw['d_arm'] * 1000, 1), T_cmd=round(SECS, 3),
                          T_cmd_lin=round(T_LIN, 3), T_cmd_rot=round(T_ROT, 3))))
    sys.exit(0)
ctl = f'/gantry_{G}_with_arm_controller/follow_joint_trajectory'
ac = ActionClient(n, FollowJointTrajectory, ctl)
assert ac.wait_for_server(timeout_sec=15), f'{ctl} tidak ada'
g = FollowJointTrajectory.Goal()
g.trajectory.joint_names = [LJ, RJ]
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
# bridge chases the JTC setpoint (debounce 1.0 mm / 0.3 deg, deadband 50 pulses): wait for BOTH encoders to STOP
STILL_L, STILL_R = 0.00005, math.radians(0.01)
MOV_L, MOV_R = 0.0005, math.radians(0.05)
end, last, t_still = time.time() + 30.0, (pos[LJ], pos[RJ]), time.time()
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if abs(pos[LJ] - last[0]) > STILL_L or abs(pos[RJ] - last[1]) > STILL_R:
        last, t_still = (pos[LJ], pos[RJ]), time.time()
    elif time.time() - t_still >= 1.0 and (abs(pos[LJ] - sl) > MOV_L or abs(pos[RJ] - sr) > MOV_R):
        break
L = [r for r in log if r[1] is not None and r[2] is not None]


def span(ix, s, mov, eps):
    mv = [r for r in L if abs(r[ix] - s) > mov]
    t_m = mv[0][0] if mv else None
    t_s = None
    for k in range(len(L) - 1, 0, -1):          # last sample where this axis still changed
        if abs(L[k][ix] - L[k - 1][ix]) > eps:
            t_s = L[k][0]
            break
    return t_m, t_s


tml, tsl = span(1, sl, MOV_L, 1e-6)
tmr, tsr = span(2, sr, MOV_R, 1e-6)
t_move = min([t for t in (tml, tmr) if t is not None], default=None)
t_stop = max([t for t in (tsl if tml else None, tsr if tmr else None) if t is not None], default=None)
drift = max((abs(r[5][k] - arm0[k]) for r in L for k in ALL_ARMS if r[5][k] is not None), default=float('nan'))
oldrift = max((abs(r[3] - ol0) for r in L if r[3] is not None), default=float('nan'))
ordrift = max((abs(r[4] - or0) for r in L if r[4] is not None), default=float('nan'))
win = [r for r in L if t_move and t_stop and t_move <= r[0] <= t_stop]
tpk = max(((v, k) for r in win for k, v in r[6].items()), default=(float('nan'), ''))
rec = dict(gantry=G, goal=[GL, GR_DEG], start=[round(sl, 6), round(s_deg, 4)],
           end=[round(pos[LJ], 6), round(math.degrees(pos[RJ]), 4)],
           err_mm=round((pos[LJ] - GL) * 1000, 3), err_deg=round(math.degrees(pos[RJ]) - GR_DEG, 4),
           T_cmd=round(SECS, 3), T_cmd_lin=round(T_LIN, 3), T_cmd_rot=round(T_ROT, 3),
           t_send=t_send, t_jtc_done=t_jtc, t_first_move=t_move, t_stop=t_stop,
           t_traverse=round(t_stop - t_move, 3) if (t_stop and t_move) else None,
           t_lin=round(tsl - tml, 3) if (tml and tsl) else None, t_rot=round(tsr - tmr, 3) if (tmr and tsr) else None,
           arm_drift_deg=round(math.degrees(drift), 4), other_rail_drift_mm=round(oldrift * 1000, 3),
           other_rot_drift_deg=round(math.degrees(ordrift), 4), arm_tau_peak=round(tpk[0], 3), arm_tau_joint=tpk[1],
           sweep_min_mm=round(sw['d'] * 1000, 1), sweep_arm_mm=round(sw['d_arm'] * 1000, 1), sweep_pair=sw['pair'],
           jtc_error_code=str(ec), n_log=len(L))
print(f"akhir: g{G} = ({rec['end'][0]:.6f} m, {rec['end'][1]:+.4f} deg), galat {rec['err_mm']:+.2f} mm / "
      f"{rec['err_deg']:+.4f} deg; t_traverse {rec['t_traverse']} s (lin {rec['t_lin']}, rot {rec['t_rot']}; "
      f"T_cmd {SECS:.2f}); lengan (keempat) bergeser maks {rec['arm_drift_deg']:.4f} deg; rel lain "
      f"{rec['other_rail_drift_mm']:.3f} mm, rot lain {rec['other_rot_drift_deg']:.4f} deg; torsi puncak "
      f"{rec['arm_tau_peak']} {rec['arm_tau_joint']}")
print(json.dumps(rec))
sys.exit(3 if (not L or abs(rec['err_mm']) > 2.0 or abs(rec['err_deg']) > 1.0 or rec['arm_drift_deg'] > 0.5
               or rec['other_rail_drift_mm'] > 2.0 or rec['other_rot_drift_deg'] > 0.5
               or not rec['arm_tau_peak'] <= 14.0) else 0)
