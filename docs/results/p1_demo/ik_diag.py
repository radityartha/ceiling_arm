#!/usr/bin/env python3
"""DEMO-OFF diagnostic: WHY a p_cmd is NO-PLAN -- out of reach, or in collision?

For each (arm, p_cmd), tool pointing down (as reach_dwell_probe.command_pose), roll
swept over --rolls values: /compute_ik with avoid_collisions False (pure
kinematics) and True (planning scene incl. env_static_map). Counts solutions.
    kin 0          -> out of reach
    kin > 0, col 0 -> reachable, every solution collides (scene refuses it)
Read-only (no motion).

    python3 ik_diag.py arm_1:1.75,0.55,1.22 arm_2:1.25,0.98,1.45
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node

TOOL = {'arm_1': 't1_a1_tool_frame', 'arm_2': 't1_a2_tool_frame'}


def main():
    rolls = 12
    rclpy.init()
    n = Node('ik_diag')
    cli = n.create_client(GetPositionIK, '/compute_ik')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('no /compute_ik')
        return 1
    for spec in sys.argv[1:]:
        arm, xyz = spec.split(':')
        x, y, z = (float(v) for v in xyz.split(','))
        res = {}
        for avoid in (False, True):
            ok = 0
            for k in range(rolls):
                yaw = 2 * math.pi * k / rolls
                req = GetPositionIK.Request()
                r = req.ik_request
                r.group_name = arm
                r.ik_link_name = TOOL[arm]
                r.avoid_collisions = avoid
                r.timeout.sec = 1
                ps = PoseStamped()
                ps.header.frame_id = 'world'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
                # Rz(yaw) * Rx(pi): tool z pointing down, roll = yaw
                ps.pose.orientation.x = math.cos(yaw / 2)
                ps.pose.orientation.y = math.sin(yaw / 2)
                ps.pose.orientation.z = 0.0
                ps.pose.orientation.w = 0.0
                r.pose_stamped = ps
                f = cli.call_async(req)
                rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
                if f.result() is not None and f.result().error_code.val == 1:
                    ok += 1
            res[avoid] = ok
        print(f'{arm} ({x:.2f}, {y:.2f}, {z:.2f}): kin {res[False]}/{rolls}  '
              f'col-free {res[True]}/{rolls}', flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
