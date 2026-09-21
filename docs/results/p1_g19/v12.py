"""G19 validation V1/V2 -- plan_only, zero motion. Uses the live move_group."""
import sys, numpy as np, rclpy, pinocchio as pin
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from reach_dwell_probe import Probe, _plan_and_screen
from dual_arm_targets import REST
from interarm_collision import InterArmChecker, _ik
from geometry_msgs.msg import PoseStamped
def pose(x):
    p = PoseStamped(); p.header.frame_id = 'world'
    p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, x)
    p.pose.orientation.x, p.pose.orientation.w = 1.0, 0.0; return p
def base(lin):
    b = {'t1_linear_joint': lin, 't1_rotation_joint': 0.0}
    for p in ('t1_a1_', 't1_a2_'): b.update({f'{p}joint_{i}': v for i, v in enumerate(REST, 1)})
    return b
c = InterArmChecker()
def fk(jt, lin, arm):
    js = base(lin); js.update(zip(jt.joint_names, jt.points[-1].positions))
    q = c.q_from(js); pin.framesForwardKinematics(c.model, c.data, q)
    return c.data.oMf[c.model.getFrameId(f't1_a{arm}_tool_frame')].translation
rclpy.init(); n = Probe('arm_1', 0.0, 12.0, False, arms=['arm_1', 'arm_2'])
n._plan_ac = ActionClient(n, MoveGroup, 'move_action'); assert n._plan_ac.wait_for_server(timeout_sec=15)
P = lambda arm, x, sj, other: _plan_and_screen(arm, pose(x), n, 0.002, 2.0, 0.15, 15.0, 12.0, other, start_joints=sj)
# V1: start_state rail honoured. B2 pair 1 arm_1 target shifted +0.4 m, planned with rail PLACED at 0.95.
t = np.array([1.400, 0.318, 1.240])
v, tr = P('arm_1', t, base(0.95), 'arm_2')
print('V1a rel DITEMPATKAN 0.95, target geser :', v, '| FK(ujung, rel 0.95) - target =',
      None if tr is None else np.round((fk(tr.joint_trajectory, 0.95, 1) - t) * 1000, 1), 'mm')
v0, tr0 = P('arm_1', np.array([1.000, 0.318, 1.240]), base(0.55), 'arm_2')
print('V1b rel 0.55, target asli B2 #1       :', v0, '| FK(ujung, rel 0.55) - target =',
      None if tr0 is None else np.round((fk(tr0.joint_trajectory, 0.55, 1) - [1.0, .318, 1.24]) * 1000, 1), 'mm')
if tr is not None and tr0 is not None:
    a = np.array(tr.joint_trajectory.points[-1].positions); b = np.array(tr0.joint_trajectory.points[-1].positions)
    print('     ujung sendi 0.95-geser vs 0.55-asli, maks |selisih| =', np.round(np.degrees(np.abs(a - b)).max(), 2), 'deg (roll bebas, jadi tak harus 0)')
# V2: placement reaches MoveIt AND the screen. arm_2 to its B2 #1 target; arm_1 placed at IK for the SAME point.
t2 = np.array([0.214, 0.247, 1.240])
q, ok = _ik(c.model, c.data, 't1_a1_', t2, c.q_from(base(0.55)))
a1 = {f't1_a1_joint_{i}': q[c.model.joints[c.model.getJointId(f't1_a1_joint_{i}')].idx_q] for i in range(1, 7)}
pl = base(0.55); pl.update(a1)
print('V2a arm_2, arm_1 di REST               :', P('arm_2', t2, base(0.55), 'arm_1')[0])
print('V2b arm_2, arm_1 DITEMPATKAN di titik arm_2 (IK ok=%s):' % ok, P('arm_2', t2, pl, 'arm_1')[0])
n.destroy_node(); rclpy.shutdown()
