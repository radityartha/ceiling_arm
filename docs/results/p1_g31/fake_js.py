"""Synthetic /joint_states for DRY runs WITHOUT a stack (run under ROS_DOMAIN_ID=77). Never commands anything.

    ROS_DOMAIN_ID=77 python3 fake_js.py --off-deg 1.0 --arms arm_3 arm_4 --r1 0.0007 --r2 0.0007 --rot2 0.0
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
ap = argparse.ArgumentParser()
ap.add_argument('--off-deg', type=float, default=0.0)
ap.add_argument('--arms', nargs='*', default=[])
for k in ('r1', 'r2', 'rot1', 'rot2'):
    ap.add_argument(f'--{k}', type=float, default=0.0)
a = ap.parse_args()
names, pos = [], []
for arm, p in PREFIX.items():
    for i, v in enumerate(REST, 1):
        names.append(f'{p}joint_{i}')
        pos.append(v + (math.radians(a.off_deg) if arm in a.arms else 0.0))
names += ['t1_linear_joint', 't2_linear_joint', 't1_rotation_joint', 't2_rotation_joint']
pos += [a.r1, a.r2, math.radians(a.rot1), math.radians(a.rot2)]
rclpy.init()
n = Node('g31_fake_js')
pub = n.create_publisher(JointState, '/joint_states', 10)


def tick():
    m = JointState()
    m.header.stamp = n.get_clock().now().to_msg()
    m.name, m.position, m.effort = names, pos, [0.0] * len(names)
    pub.publish(m)


n.create_timer(0.01, tick)
rclpy.spin(n)
