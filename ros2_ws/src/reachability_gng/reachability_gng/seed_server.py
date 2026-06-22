"""Serve GNG joint-configuration seeds for MoveIt IK.

For now this uses a simple topic interface (avoids a custom .srv package):

    sub : ~/query_pose   geometry_msgs/PoseStamped   (target EE pose)
    pub : ~/seed_state   sensor_msgs/JointState       (nearest-node q)

A MoveIt IK caller can take seed_state as the initial guess instead of the
default/random seed. TODO(phase2): promote this to a proper service
(e.g. moveit_msgs-style GetSeed) once an interfaces package is added.

The model is produced offline by data_gen.py + train.py.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from reachability_gng.gng import GNG


class SeedServer(Node):
    def __init__(self):
        super().__init__('gng_seed_server')
        self.declare_parameter('model_path', 'model.npz')
        self.declare_parameter('task', 'pos')          # 'pos' or 'pose'
        self.declare_parameter('ori_weight', 0.3)
        self.declare_parameter('joint_names', [
            't1_linear_joint', 't1_rotation_joint',
            't1_a1_joint_1', 't1_a1_joint_2', 't1_a1_joint_3',
            't1_a1_joint_4', 't1_a1_joint_5', 't1_a1_joint_6',
        ])

        model_path = self.get_parameter('model_path').value
        self.task = self.get_parameter('task').value
        self.ori_weight = self.get_parameter('ori_weight').value
        self.joint_names = list(self.get_parameter('joint_names').value)

        # prefer the joint order stored at training time, if present
        stats_path = (model_path[:-4] if model_path.endswith('.npz')
                      else model_path) + '_stats.npz'
        try:
            names = np.load(stats_path)['joint_names']
            if len(names):
                self.joint_names = [str(n) for n in names]
        except OSError:
            pass

        self.gng = GNG.load(model_path)
        self.get_logger().info(
            f'Loaded GNG model "{model_path}": {len(self.gng.W)} nodes, '
            f'task_dim={self.gng.task_dim}')

        self.sub = self.create_subscription(
            PoseStamped, '~/query_pose', self._on_pose, 10)
        self.pub = self.create_publisher(JointState, '~/seed_state', 10)

    def _task_vec(self, pose):
        p = pose.position
        if self.task == 'pos':
            return np.array([p.x, p.y, p.z])
        o = pose.orientation
        return np.array([p.x, p.y, p.z,
                         o.x * self.ori_weight, o.y * self.ori_weight,
                         o.z * self.ori_weight, o.w * self.ori_weight])

    def _on_pose(self, msg: PoseStamped):
        q = self.gng.seed_q(self._task_vec(msg.pose))
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = [float(v) for v in q]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SeedServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
