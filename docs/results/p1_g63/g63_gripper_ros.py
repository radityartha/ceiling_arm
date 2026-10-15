#!/usr/bin/env python3
"""G63 end of session: OPEN one gripper through the running stack (GripperCommand on gripper_<n>_controller, same
goal as scripts/hardware_check.py --grippers: max_effort 50). Only after the board is removed. HW MOTION (fingers):
one operator approval per invocation. Prints the finger joint before/after.

    g63_gripper_ros.py --arm arm_2 [--position 0.0]
"""
import argparse
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

CTRL = {'arm_1': ('gripper_1_controller', 't1_a1_'), 'arm_2': ('gripper_2_controller', 't1_a2_'),
        'arm_3': ('gripper_3_controller', 't2_a1_'), 'arm_4': ('gripper_4_controller', 't2_a2_')}


def finger(node, joint, secs=1.0):
    v = []
    sub = node.create_subscription(JointState, '/joint_states',
                                   lambda m: v.extend(p for k, p in zip(m.name, m.position) if k == joint), 50)
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return sorted(v)[len(v) // 2] if v else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', required=True, choices=sorted(CTRL))
    ap.add_argument('--position', type=float, default=0.0)
    a = ap.parse_args()
    ctrl, pre = CTRL[a.arm]
    joint = pre + 'right_finger_bottom_joint'
    rclpy.init()
    n = rclpy.create_node('g63_gripper_ros')
    before = finger(n, joint)
    ac = ActionClient(n, GripperCommand, f'/{ctrl}/gripper_cmd')
    if not ac.wait_for_server(timeout_sec=5.0):
        print(f'{ctrl}: action server tidak ada -- TIDAK dikirim')
        return 2
    g = GripperCommand.Goal()
    g.command.position, g.command.max_effort = a.position, 50.0
    f = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    h = f.result()
    if h is None or not h.accepted:
        print(f'{ctrl}: goal DITOLAK')
        return 1
    r = h.get_result_async()
    rclpy.spin_until_future_complete(n, r, timeout_sec=15.0)
    res = r.result().result if r.result() else None
    after = finger(n, joint)
    print(f'{a.arm} {joint}: {before:.4f} -> {after:.4f} rad (target {a.position:.3f}); hasil '
          f'{None if res is None else dict(pos=round(res.position, 4), reached=res.reached_goal, stalled=res.stalled)}')
    ok = abs(after - a.position) <= 0.02
    print('OK' if ok else 'TIDAK sampai target')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
