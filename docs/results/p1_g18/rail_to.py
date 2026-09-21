"""Move t1_linear_joint to a target (m) via the gantry bridge. Partial goal: ONLY the
rail -- rotation and all 12 arm joints are held by JTC. DRY RUN unless --move."""
import math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
J = 't1_linear_joint'; GOAL = float(sys.argv[1]); SECS, STEPS = 30.0, 60
ARMS = [f't1_a{a}_joint_{j}' for a in (1, 2) for j in range(1, 7)]
if not (0.0 <= GOAL <= 1.0): print('REFUSE: target outside 0..1.0 m (end stop ~1.656 m, guard 1.6)'); sys.exit(1)
rclpy.init(); n = Node('g18_rail_to'); pos = {}
n.create_subscription(JointState, '/joint_states', lambda m: pos.update(zip(m.name, m.position)), 50)
t0 = time.time()
while time.time() - t0 < 1.5 or J not in pos: rclpy.spin_once(n, timeout_sec=0.05)
s = pos[J]; arm0 = {k: pos.get(k) for k in ARMS}
print(f'{J}: {s:.6f} -> {GOAL:.6f} m ({(GOAL - s) * 1000:+.1f} mm), {SECS:.0f} s; t1_rotation {pos.get("t1_rotation_joint"):+.6f}')
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
f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20); gh = f.result()
if gh is None or not gh.accepted: print('goal DITOLAK'); sys.exit(1)
rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=SECS + 40)
print('JTC error_code:', rf.result().result.error_code if rf.result() else 'timeout')
end = time.time() + 45.0   # bridge chases at a fixed 31.4 mm/s; wait for the ENCODER, not the JTC
while time.time() < end and abs(pos[J] - GOAL) > 0.002: rclpy.spin_once(n, timeout_sec=0.05)
t1 = time.time() + 2.0
while time.time() < t1: rclpy.spin_once(n, timeout_sec=0.05)
drift = max(abs(pos[k] - arm0[k]) for k in ARMS)
print(f'akhir: {J} = {pos[J]:.6f} m (galat {(pos[J] - GOAL) * 1000:+.2f} mm); '
      f'lengan bergeser maks {math.degrees(drift):.4f} deg selama gerak rel')
