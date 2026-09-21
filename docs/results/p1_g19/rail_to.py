"""G19 step 4 TRAVERSE: t1_linear_joint -> 0.550 or 0.950 m via the gantry bridge.

From g18 rail_to.py. Partial goal: ONLY the rail -- rotation and all 12 arm joints are
held by JTC. DRY RUN unless --move. docs/p1_g19_hw.md A2/A3/A5:
  S12  REFUSES unless BOTH arms are within 0.5 deg of REST (merged /joint_states >= 0.5 s)
  S13  only 0.550 / 0.950; cosine profile, peak setpoint speed = 0.9 * v_lin
  A3   t_traverse = encoder moves > 0.5 mm -> |rail - goal| <= 0.52 mm and stays
Exit 0 ok; 1 refused/failed; 3 S17 trip (arm drift > 0.5 deg, rail error > 2 mm).
Last line of stdout is one JSON record."""
import json, math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
J = 't1_linear_joint'; GOAL = float(sys.argv[1]); V_LIN = 3000 / 95.4930 / 1000.0   # m/s
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
ARMS = [f't1_a{a}_joint_{j}' for a in (1, 2) for j in range(1, 7)]
if round(GOAL, 3) not in (0.550, 0.950): print('REFUSE: S13 -- rel hanya 0.550 / 0.950'); sys.exit(1)
rclpy.init(); n = Node('g19_rail_to'); pos = {}; eff = {}; log = []
def cb(m):
    t = time.time()
    for i, k in enumerate(m.name):
        pos[k] = m.position[i]
        if i < len(m.effort): eff[k] = m.effort[i]
    if J in m.name: log.append((t, pos[J], {k: pos.get(k) for k in ARMS}, {k: abs(eff.get(k, 0.0)) for k in ARMS}))
n.create_subscription(JointState, '/joint_states', cb, 200)
t0 = time.time()
while time.time() - t0 < 1.0 or not all(k in pos for k in ARMS + [J]):
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 > 10: print('REFUSE: /joint_states tidak lengkap'); sys.exit(1)
dev = max(abs(pos[f't1_a{a}_joint_{j}'] - REST[j - 1]) for a in (1, 2) for j in range(1, 7))
print(f'S12: lengan maks |q - REST| = {math.degrees(dev):.3f} deg (batas 0.5)')
if math.degrees(dev) >= 0.5: print('REFUSE: S12 -- lengan BELUM di REST, gantry tidak digerakkan'); sys.exit(1)
s = pos[J]; D = abs(GOAL - s); SECS = max(3.0, math.pi * D / (2 * 0.9 * V_LIN)); STEPS = max(10, int(SECS * 2))
arm0 = {k: pos[k] for k in ARMS}
print(f'{J}: {s:.6f} -> {GOAL:.6f} m ({(GOAL - s) * 1000:+.1f} mm), T_cmd {SECS:.2f} s, '
      f'T_lin model {0.29 + D * 1000 / (V_LIN * 1000):.2f} s; t1_rotation {pos.get("t1_rotation_joint"):+.6f}')
pts = []
for k in range(1, STEPS + 1):
    f = 0.5 * (1 - math.cos(math.pi * k / STEPS)); t = SECS * k / STEPS
    p = JointTrajectoryPoint(); p.positions = [s + (GOAL - s) * f]; p.velocities = [0.0]
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(p)
print(f'setpoint pertama {abs(pts[0].positions[0] - s) * 1000:.2f} mm (arm_tol 5 mm)')
if '--move' not in sys.argv: print('DRY RUN: tidak ada yang dikirim.'); sys.exit(0)
ac = ActionClient(n, FollowJointTrajectory, '/gantry_1_with_arm_controller/follow_joint_trajectory')
assert ac.wait_for_server(timeout_sec=15), 'controller tidak ada'
g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = [J]; g.trajectory.points = pts
t_send = time.time(); log.clear()
f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20); gh = f.result()
if gh is None or not gh.accepted: print('goal DITOLAK'); sys.exit(1)
rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=SECS + 40)
t_jtc = time.time(); ec = rf.result().result.error_code if rf.result() else 'timeout'
print('JTC error_code:', ec)
end = time.time() + 45.0   # bridge chases the JTC setpoint; wait for the ENCODER, not the JTC
while time.time() < end and abs(pos[J] - GOAL) > 0.00052: rclpy.spin_once(n, timeout_sec=0.05)
t1 = time.time() + 2.0
while time.time() < t1: rclpy.spin_once(n, timeout_sec=0.05)
mv = [r for r in log if abs(r[1] - s) > 0.0005]
t_move = mv[0][0] if mv else None
arr = None   # first sample after which the rail stays inside the 0.52 mm deadband
for k in range(len(log) - 1, -1, -1):
    if abs(log[k][1] - GOAL) > 0.00052: break
    arr = log[k][0]
drift = max(abs(r[2][k] - arm0[k]) for r in log for k in ARMS) if log else float('nan')
win = [r for r in log if t_move and arr and t_move <= r[0] <= arr]
tpk = max(((v, k) for r in win for k, v in r[3].items()), default=(float('nan'), ''))
rec = dict(goal=GOAL, start=round(s, 6), end=round(pos[J], 6), err_mm=round((pos[J] - GOAL) * 1000, 3),
           T_cmd=round(SECS, 3), T_lin_model=round(0.29 + D / V_LIN, 3), t_send=t_send, t_jtc_done=t_jtc,
           t_first_move=t_move, t_arrive=arr, t_traverse=round(arr - t_move, 3) if (arr and t_move) else None,
           arm_drift_deg=round(math.degrees(drift), 4), arm_tau_peak=round(tpk[0], 3), arm_tau_joint=tpk[1],
           jtc_error_code=str(ec))
print(f'akhir: {J} = {pos[J]:.6f} m (galat {rec["err_mm"]:+.2f} mm); t_traverse {rec["t_traverse"]} s '
      f'(T_cmd {SECS:.2f}, T_lin {rec["T_lin_model"]}); lengan bergeser maks {rec["arm_drift_deg"]:.4f} deg, '
      f'torsi lengan puncak {rec["arm_tau_peak"]} {tpk[1]} selama traverse')
print(json.dumps(rec))
sys.exit(3 if (rec['arm_drift_deg'] > 0.5 or abs(rec['err_mm']) > 2.0) else 0)
