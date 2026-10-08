#!/usr/bin/env python3
"""Put the frozen G33 environment map into MoveIt's planning scene (one object).

The octomap path is dead in this cell (moveit_ros_perception is not installed,
docs/p1_g33_map.md A0.1), so the map goes in as an ordinary CollisionObject,
id `env_static_map`: vertical runs of map voxels in each column are merged into
one box, and every box is grown by the SAME margin the script screen uses
(env_collision.ENV_MARGIN_M) -- so MoveIt's "in collision" means at least what
EnvChecker's MARGIN/COLLIDE means (closer than 5 cm). A grown box is an L-inf
inflation, so at box edges/corners MoveIt is up to sqrt(3)x stricter than 5 cm;
a 4 cm re-bin made that worse (53/135 R0 configs invalid, docs/p1_g33_map.md B).

Strict: no map file -> exit 2. After publishing it asks /get_planning_scene
whether `env_static_map` is really there, and re-publishes whenever it is gone
(move_group restart). `--once` publishes, verifies and exits (0 only if verified).

    python3 scripts/env_static_map_pub.py            # keep alive, re-assert
    python3 scripts/env_static_map_pub.py --once
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env_collision import ENV_MAP, ENV_MARGIN_M, load_env_map  # noqa: E402

OBJECT_ID = 'env_static_map'
# G36: the raw voxel centres for RViz (latched). The scene object above is the
# 5 cm-GROWN boxes MoveIt plans against; this is the map EnvChecker screens with.
CLOUD_TOPIC = '/env_static_map/cloud'
BIN = 0.02      # = map resolution; coarser bins fatten every box


def column_boxes(centers, bin_=BIN, grow=ENV_MARGIN_M):
    """(N,3) centres, (N,3) sizes: 4 cm bins, vertical runs merged, grown by `grow`."""
    k = np.unique(np.floor(centers / bin_).astype(np.int64), axis=0)
    k = k[np.lexsort((k[:, 2], k[:, 1], k[:, 0]))]
    new = np.ones(len(k), bool)
    new[1:] = ((k[1:, 0] != k[:-1, 0]) | (k[1:, 1] != k[:-1, 1])
               | (k[1:, 2] != k[:-1, 2] + 1))
    starts = np.nonzero(new)[0]
    ends = np.r_[starts[1:], len(k)] - 1
    lo, hi = k[starts], k[ends]
    c = np.c_[(lo[:, :2] + 0.5) * bin_, (lo[:, 2] + hi[:, 2] + 1) * bin_ / 2]
    s = np.c_[np.full((len(lo), 2), bin_), (hi[:, 2] - lo[:, 2] + 1) * bin_] + 2 * grow
    return c, s


class Pub(Node):
    def __init__(self, map_file):
        super().__init__('env_static_map_pub')
        m = load_env_map(map_file)
        c, s = column_boxes(m['centers'].astype(float))
        co = CollisionObject()
        co.id = OBJECT_ID
        co.header.frame_id = 'world'
        co.operation = CollisionObject.ADD
        co.pose.orientation.w = 1.0
        for ci, si in zip(c, s):
            p = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(x) for x in si])
            q = Pose()
            q.position.x, q.position.y, q.position.z = (float(x) for x in ci)
            q.orientation.w = 1.0
            co.primitives.append(p)
            co.primitive_poses.append(q)
        self.scene = PlanningScene(is_diff=True)
        self.scene.world.collision_objects.append(co)
        self.n_box, self.n_vox = len(c), len(m['centers'])
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, qos)
        self.centers = m['centers'].astype(float)
        self.cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.get_logger().info(f'{map_file}: {self.n_vox} voxel -> {self.n_box} kotak '
                               f'(bin {BIN}, +{ENV_MARGIN_M} m)')

    def present(self):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=20.0)
        if f.result() is None:
            return None
        return OBJECT_ID in [o.id for o in f.result().scene.world.collision_objects]

    def assert_once(self):
        self.pub.publish(self.scene)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.present():
                return True
            time.sleep(0.5)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=ENV_MAP)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--period', type=float, default=15.0)
    a = ap.parse_args()
    try:
        load_env_map(a.map)
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f'TOLAK: {e}')
        sys.exit(2)
    rclpy.init()
    n = Pub(a.map)
    ok = n.assert_once()
    n.get_logger().info(f'{OBJECT_ID} di planning scene: {ok}')
    # AFTER the scene is verified: sent first, this 1.6 MB latched cloud kept the
    # scene diff from reaching move_group (G36, mock: False 3x in a row).
    n.cloud_pub.publish(create_cloud_xyz32(Header(frame_id='world'), n.centers.tolist()))
    if a.once:
        rclpy.shutdown()
        sys.exit(0 if ok else 1)
    try:
        while rclpy.ok():
            t = time.time()
            while time.time() - t < a.period:
                rclpy.spin_once(n, timeout_sec=0.5)
            if n.present() is False:
                n.get_logger().warn(f'{OBJECT_ID} HILANG dari planning scene -- publish ulang')
                n.get_logger().info(f'{OBJECT_ID} di planning scene: {n.assert_once()}')
    except KeyboardInterrupt:
        pass
    if rclpy.ok():                      # SIGINT (launch shutdown) already shut it down
        rclpy.shutdown()


if __name__ == '__main__':
    main()
