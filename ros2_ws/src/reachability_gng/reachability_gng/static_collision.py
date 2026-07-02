"""Publish mapped STATIC geometry as MoveIt collision boxes.

Loads every box saved by map_static.py and publishes each as a moveit_msgs/
CollisionObject (BOX) into the planning scene, so MoveIt treats known static
geometry -- the work table, a cabinet, a fridge, walls -- as reliable, complete,
occlusion-free collision geometry. Unlike the live octomap (collision_cloud),
which only sees each surface's visible side and suffers occlusion "shadows" and
self-filter leakage, a mapped box is exact and hole-free. The live octomap is left
to handle genuinely unknown / dynamic obstacles.

It republishes on a timer so the objects survive move_group (re)starts and any
planning-scene resets, and retries loading so mapping a piece AFTER launch is
picked up without a restart. If the map file is missing it warns and idles -- run
map_static.py first.

    ros2 run reachability_gng static_collision
    ros2 run reachability_gng static_collision --ros-args -p map_file:=/tmp/static_geometry.npz
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive


class StaticCollision(Node):
    def __init__(self):
        super().__init__('static_collision')
        self.declare_parameter('map_file', '/tmp/static_geometry.npz')
        self.declare_parameter('publish_period', 2.0)

        self.map_file = self.get_parameter('map_file').value

        # transient-local so a late-joining move_group still receives the objects.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)

        self._scene = self._build_scene()
        if self._scene is None:
            self.get_logger().warn(
                f'map file {self.map_file} not found -- run map_static first; '
                'will keep checking and publish once it appears')
        else:
            self._publish()
        # one timer: publishes if loaded, else retries loading (so mapping a
        # piece AFTER launch is picked up without restarting this node).
        self.create_timer(
            float(self.get_parameter('publish_period').value), self._tick)

    def _build_scene(self):
        if not os.path.exists(self.map_file):
            return None
        d = np.load(self.map_file, allow_pickle=True)
        names = [str(x) for x in d['names']]
        centers = np.asarray(d['centers'], float)
        sizes = np.asarray(d['sizes'], float)
        frame = str(d['frame'])

        scene = PlanningScene()
        scene.is_diff = True
        for name, center, size in zip(names, centers, sizes):
            co = CollisionObject()
            co.header.frame_id = frame
            co.id = name
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, center)
            pose.orientation.w = 1.0
            co.primitives.append(box)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
        self.get_logger().info(
            f'static geometry: {len(names)} box(es) {names} in frame {frame}')
        return scene

    def _tick(self):
        if self._scene is None:
            self._scene = self._build_scene()
            if self._scene is None:
                return
        self._publish()

    def _publish(self):
        self._scene.is_diff = True        # PlanningScene has no header; merge diff
        self.pub.publish(self._scene)


def main():
    rclpy.init()
    node = StaticCollision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
