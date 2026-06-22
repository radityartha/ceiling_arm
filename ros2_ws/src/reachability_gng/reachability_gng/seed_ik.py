"""GNG-seeded MoveIt IK for the redundant arm+table chain (Phase 2).

Subscribes a target end-effector pose, computes a joint seed from the GNG map,
and calls MoveIt's /compute_ik service with that seed placed in the IK request's
robot_state. With ``use_gng_seed:=false`` it sends a neutral/zero seed instead,
so the same node serves the A/B comparison the paper needs.

    sub : ~/target_pose   geometry_msgs/PoseStamped
    pub : ~/ik_solution   sensor_msgs/JointState   (on success)
    cli : /compute_ik     moveit_msgs/srv/GetPositionIK

Requires move_group running (e.g. my_workcell.launch.py) so /compute_ik exists,
and the SRDF group `table_1_with_arm_1` (added in Phase 2).

The shared `build_ik_request` / `solve_ik` helpers are reused by eval.py.
"""

from __future__ import annotations

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from reachability_gng.gng import GNG


def build_ik_request(group, ee_frame, pose_stamped, seed_names, seed_pos,
                     timeout_s=0.05, avoid_collisions=True):
    """Assemble a GetPositionIK.Request with the given seed in robot_state."""
    req = GetPositionIK.Request()
    ik = PositionIKRequest()
    ik.group_name = group
    ik.ik_link_name = ee_frame
    ik.pose_stamped = pose_stamped
    ik.avoid_collisions = avoid_collisions
    ik.timeout = Duration(sec=int(timeout_s),
                          nanosec=int((timeout_s % 1.0) * 1e9))
    seed = RobotState()
    seed.joint_state = JointState()
    seed.joint_state.name = list(seed_names)
    seed.joint_state.position = [float(v) for v in seed_pos]
    ik.robot_state = seed
    req.ik_request = ik
    return req


def solve_ik(client, request, spin_node, timeout_s=2.0):
    """Call /compute_ik synchronously; return (ok, joint_state, error_val)."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(spin_node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        return False, None, None
    res = future.result()
    ok = res.error_code.val == 1  # MoveItErrorCodes.SUCCESS
    return ok, res.solution.joint_state, res.error_code.val


class SeedIK(Node):
    def __init__(self):
        super().__init__('gng_seed_ik')
        self.declare_parameter('model_path', 'model.npz')
        self.declare_parameter('group', 'table_1_with_arm_1')
        self.declare_parameter('ee_frame', 't1_a1_tool_frame')
        self.declare_parameter('task', 'pos')
        self.declare_parameter('ori_weight', 0.3)
        self.declare_parameter('use_gng_seed', True)
        self.declare_parameter('ik_timeout', 0.05)

        self.group = self.get_parameter('group').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.task = self.get_parameter('task').value
        self.ori_weight = self.get_parameter('ori_weight').value
        self.use_seed = self.get_parameter('use_gng_seed').value
        self.ik_timeout = self.get_parameter('ik_timeout').value

        model_path = self.get_parameter('model_path').value
        self.gng = GNG.load(model_path)
        self.joint_names = self._load_names(model_path)

        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('waiting for /compute_ik ...')
        self.cli.wait_for_service()

        self.sub = self.create_subscription(
            PoseStamped, '~/target_pose', self._on_pose, 10)
        self.pub = self.create_publisher(JointState, '~/ik_solution', 10)
        self.get_logger().info(
            f'ready: group={self.group}, use_gng_seed={self.use_seed}, '
            f'{len(self.gng.W)} nodes')

    def _load_names(self, model_path):
        stats = (model_path[:-4] if model_path.endswith('.npz')
                 else model_path) + '_stats.npz'
        try:
            names = np.load(stats)['joint_names']
            if len(names):
                return [str(n) for n in names]
        except OSError:
            pass
        return [
            't1_linear_joint', 't1_rotation_joint',
            't1_a1_joint_1', 't1_a1_joint_2', 't1_a1_joint_3',
            't1_a1_joint_4', 't1_a1_joint_5', 't1_a1_joint_6',
        ]

    def _task_vec(self, pose):
        p = pose.position
        if self.task == 'pos':
            return np.array([p.x, p.y, p.z])
        o = pose.orientation
        return np.array([p.x, p.y, p.z,
                         o.x * self.ori_weight, o.y * self.ori_weight,
                         o.z * self.ori_weight, o.w * self.ori_weight])

    def _on_pose(self, msg: PoseStamped):
        if self.use_seed:
            seed = self.gng.seed_q(self._task_vec(msg.pose))
        else:
            seed = np.zeros(len(self.joint_names))
        req = build_ik_request(self.group, self.ee_frame, msg,
                               self.joint_names, seed, self.ik_timeout)
        t0 = self.get_clock().now()
        ok, js, err = solve_ik(self.cli, req, self)
        dt = (self.get_clock().now() - t0).nanoseconds / 1e6
        if ok:
            self.pub.publish(js)
            self.get_logger().info(f'IK ok in {dt:.1f} ms')
        else:
            self.get_logger().warn(f'IK failed (err={err}) in {dt:.1f} ms')


def main():
    rclpy.init()
    node = SeedIK()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
