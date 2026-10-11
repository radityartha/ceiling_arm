"""G54 A2.1: can two trajectories for DIFFERENT gantries execute at the same time? MOCK ONLY (ROS_DOMAIN_ID=77).
arm_1 and arm_3 joint_6 each go +5 deg and back in one 6 s trajectory, sent 0.2 s apart, through
  (default)   move_group /execute_trajectory          (the probe's old path)
  --direct    /gantry_g_with_arm_controller/follow_joint_trajectory  (the probe's G54 path)
Overlap is measured from /joint_states (first / last sample each joint is > 0.05 deg off its start), NOT from client
timestamps (B1: the client saw both 'done' together while move_group had run them one after the other).
"""
import math
import os
import sys
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if os.environ.get('ROS_DOMAIN_ID') != '77':
    sys.exit('MOCK SAJA: ROS_DOMAIN_ID harus 77')
DIRECT = '--direct' in sys.argv
rclpy.init()
n = Node('g54_concurrent_exec')
JN = {'arm_1': ('t1_a1_joint_6', 1), 'arm_3': ('t2_a1_joint_6', 2)}
pos, log = {}, []


def _js(m):
    t = time.time()
    pos.update(zip(m.name, m.position))
    log.append((t, {k: pos.get(k) for k, _ in JN.values()}))


n.create_subscription(JointState, '/joint_states', _js, 200)
if DIRECT:
    acs = {a: ActionClient(n, FollowJointTrajectory, f'/gantry_{g}_with_arm_controller/follow_joint_trajectory')
           for a, (_, g) in JN.items()}
else:
    acs = {a: ActionClient(n, ExecuteTrajectory, 'execute_trajectory') for a in JN}
th = threading.Thread(target=lambda: rclpy.spin(n), daemon=True)
th.start()
for ac in acs.values():
    assert ac.wait_for_server(timeout_sec=10)
while not all(j in pos for j, _ in JN.values()):
    time.sleep(0.05)
q0 = {a: pos[j] for a, (j, _) in JN.items()}
res = {}


def go(arm):
    j = JN[arm][0]
    jt = JointTrajectory()
    jt.joint_names = [j]
    for k in range(0, 31):
        t = 6.0 * k / 30
        p = JointTrajectoryPoint()
        p.positions = [q0[arm] + math.radians(5.0) * math.sin(math.pi * t / 6.0)]
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        jt.points.append(p)
    if DIRECT:
        g = FollowJointTrajectory.Goal()
        g.trajectory = jt
    else:
        g = ExecuteTrajectory.Goal()
        g.trajectory.joint_trajectory = jt
    f = acs[arm].send_goal_async(g)
    while not f.done():
        time.sleep(0.01)
    gh = f.result()
    if not gh.accepted:
        res[arm] = dict(accepted=False)
        return
    rf = gh.get_result_async()
    while not rf.done():
        time.sleep(0.01)
    r = rf.result().result
    res[arm] = dict(accepted=True, ok=(r.error_code == 0) if DIRECT else (r.error_code.val == 1))


ts = [threading.Thread(target=go, args=('arm_1',)), threading.Thread(target=go, args=('arm_3',))]
ts[0].start()
time.sleep(0.2)
ts[1].start()
for t in ts:
    t.join(timeout=40)
time.sleep(0.5)
win = {}
for a, (j, _) in JN.items():
    mv = [t for t, d in log if d[j] is not None and abs(d[j] - q0[a]) > math.radians(0.05)]
    win[a] = (mv[0], mv[-1]) if mv else None
print('jalur', 'FJT langsung' if DIRECT else 'move_group execute_trajectory', res)
if all(win.values()):
    t0 = min(w[0] for w in win.values())
    for a, w in win.items():
        print(f'  {a}: bergerak {w[0] - t0:.2f} .. {w[1] - t0:.2f} s')
    ov = min(w[1] for w in win.values()) - max(w[0] for w in win.values())
    print(f'tumpang-tindih gerak terukur {ov:.2f} s (tiap lintasan 6 s)')
    print('BERSAMAAN' if ov > 4.0 and all(r.get('ok') for r in res.values()) else 'TIDAK BERSAMAAN')
sys.stdout.flush()
os._exit(0)                      # spin thread still alive: rclpy.shutdown() here aborts the process (core)
