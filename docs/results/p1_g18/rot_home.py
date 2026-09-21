"""Rotate t1_rotation_joint to 0 rad (encoder home) via the gantry bridge.
Partial goal: ONLY t1_rotation_joint -- rail and all 12 arm joints are held by JTC.
DRY RUN unless --move (A6/S7)."""
import math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
J, GOAL, SECS, STEPS = 't1_rotation_joint', 0.0, 10.0, 20
rclpy.init(); n = Node('g18_rot_home'); pos = {}
n.create_subscription(JointState, '/joint_states', lambda m: pos.update(zip(m.name, m.position)), 50)
t0 = time.time()
while time.time() - t0 < 1.5 or J not in pos: rclpy.spin_once(n, timeout_sec=0.05)
s = pos[J]; lin0 = pos.get('t1_linear_joint')
print(f'{J} = {s:+.6f} rad ({math.degrees(s):+.3f} deg) -> {GOAL:+.3f}; t1_linear = {lin0:.6f} m')
if abs(math.degrees(s - GOAL)) > 5.0: print('REFUSE: > 5 deg is not the ~1 deg move that was approved'); sys.exit(1)
pts = []
for k in range(1, STEPS + 1):
    f = 0.5 * (1 - math.cos(math.pi * k / STEPS)); t = SECS * k / STEPS
    p = JointTrajectoryPoint(); p.positions = [s + (GOAL - s) * f]; p.velocities = [0.0]
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(p)
if '--move' not in sys.argv: print(f'DRY RUN: {STEPS} titik, {SECS:.0f} s, 1 sendi. Tidak ada yang dikirim.'); sys.exit(0)
ac = ActionClient(n, FollowJointTrajectory, '/gantry_1_with_arm_controller/follow_joint_trajectory')
assert ac.wait_for_server(timeout_sec=15), 'controller tidak ada'
g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = [J]; g.trajectory.points = pts
f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20); gh = f.result()
if gh is None or not gh.accepted: print('goal DITOLAK'); sys.exit(1)
rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=SECS + 40)
print('JTC error_code:', rf.result().result.error_code if rf.result() else 'timeout')
end = time.time() + 8.0   # bridge moves at fixed speed; let the encoder settle
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
print(f'akhir: {J} = {pos[J]:+.6f} rad ({math.degrees(pos[J]):+.3f} deg); '
      f't1_linear = {pos.get("t1_linear_joint"):.6f} m (awal {lin0:.6f})')
