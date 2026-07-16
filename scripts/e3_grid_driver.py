#!/usr/bin/env python3
"""E3 driver: sweep a grid of target-object positions through gantry_reach_executor
and log one CSV per selection_mode (plan-only, so the arm never moves and every
trial is scored from the same home state -- a clean cross-mode comparison).

The executor must already be running (launched with the desired `selection_mode`,
`csv:=...`, and `execute:=false`), together with move_group. This node publishes
each grid pose on /target_object, triggers a plan-only pick with the non-numeric
arg 'target' (so the executor reads /target_object rather than a /detected_objects
index), waits for the pick to finish, and moves on.

Grid comes from the measured UNION reachable hull of the four arms (ceiling-capped),
symmetric in y so arms on both rails compete. Points outside the per-arm reach are
simply -31'd by the executor and logged as failures, which is fine.

Example (run once per mode; see docs/experiment_plan.md E3):
    ros2 launch reachability_gng gantry_pick.launch.py execute:=false \
        compute_traj_energy:=true selection_mode:=energy csv:=/tmp/e3_energy.csv
    python3 scripts/e3_grid_driver.py --wait 8.0
"""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


def grid_positions(nx, ny, nz):
    # table-band reachable region measured 2026-07-16 (union of 4 arms, z<=2.05):
    # x in [~0.0, 2.8], y in [-1.2, 1.2] symmetric, z on the work-surface band.
    xs = np.linspace(0.0, 2.8, nx)
    ys = np.linspace(-1.2, 1.2, ny)
    zs = np.linspace(1.05, 1.15, nz) if nz > 1 else np.array([1.08])
    return [(float(x), float(y), float(z))
            for x in xs for y in ys for z in zs]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nx', type=int, default=6)
    ap.add_argument('--ny', type=int, default=5)
    ap.add_argument('--nz', type=int, default=2)
    ap.add_argument('--wait', type=float, default=8.0,
                    help='seconds to wait after each pick trigger (must exceed '
                         'the worst-case IK+plan time for all candidates)')
    ap.add_argument('--settle', type=float, default=1.0,
                    help='seconds between publishing the target and triggering '
                         'the pick, so /target_object lands first')
    ap.add_argument('--executor', default='/gantry_reach_executor',
                    help='executor node namespace (for the ~/pick topic)')
    ap.add_argument('--limit', type=int, default=0,
                    help='stop after N positions (0 = all); handy for a smoke run')
    args = ap.parse_args()

    rclpy.init()
    node = Node('e3_grid_driver')
    tgt_pub = node.create_publisher(PoseStamped, '/target_object', 1)
    pick_pub = node.create_publisher(String, f'{args.executor}/pick', 1)
    # give the publishers a moment to connect
    time.sleep(1.0)

    pts = grid_positions(args.nx, args.ny, args.nz)
    if args.limit > 0:
        pts = pts[:args.limit]
    node.get_logger().info(f'E3 driver: {len(pts)} grid positions, '
                           f'wait={args.wait}s each')

    for i, (x, y, z) in enumerate(pts):
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        tgt_pub.publish(ps)
        node.get_logger().info(
            f'[{i + 1}/{len(pts)}] target ({x:+.2f}, {y:+.2f}, {z:+.2f})')
        _spin(node, args.settle)
        pick_pub.publish(String(data='target'))
        _spin(node, args.wait)

    node.get_logger().info('E3 driver: done')
    node.destroy_node()
    rclpy.shutdown()


def _spin(node, secs):
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


if __name__ == '__main__':
    main()
