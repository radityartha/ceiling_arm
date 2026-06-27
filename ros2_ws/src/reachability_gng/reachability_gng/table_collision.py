"""Publish the mapped work table as a STATIC MoveIt collision box.

Loads the box saved by map_table.py and publishes it as a moveit_msgs/
CollisionObject (BOX) into the planning scene, so MoveIt treats the static work
table as reliable, complete, occlusion-free collision geometry -- unlike the live
octomap, which only sees the table's visible surface and is subject to occlusion
and self-filter leakage. The live octomap (collision_cloud) is left to handle
genuinely unknown / dynamic obstacles.

It republishes on a timer so the object survives move_group (re)starts and any
planning-scene resets. If the map file is missing it warns and idles -- run
map_table.py first.

    ros2 run reachability_gng table_collision
    ros2 run reachability_gng table_collision --ros-args -p map_file:=/tmp/work_table.npz
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


class TableCollision(Node):
    def __init__(self):
        super().__init__('table_collision')
        self.declare_parameter('map_file', '/tmp/work_table.npz')
        self.declare_parameter('object_id', 'work_table')
        self.declare_parameter('publish_period', 2.0)

        self.map_file = self.get_parameter('map_file').value
        self.object_id = self.get_parameter('object_id').value

        # transient-local so a late-joining move_group still receives the object.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)

        self._scene = self._build_scene()
        if self._scene is None:
            self.get_logger().warn(
                f'map file {self.map_file} not found -- run map_table.py first; '
                'will keep checking and publish once it appears')
        else:
            self._publish()
        # one timer: publishes if loaded, else retries loading (so mapping the
        # table AFTER launch is picked up without restarting this node).
        self.create_timer(
            float(self.get_parameter('publish_period').value), self._tick)

    def _build_scene(self):
        if not os.path.exists(self.map_file):
            return None
        d = np.load(self.map_file, allow_pickle=True)
        center = np.asarray(d['center'], float)
        size = np.asarray(d['size'], float)
        frame = str(d['frame'])

        co = CollisionObject()
        co.header.frame_id = frame
        co.id = self.object_id
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        self.get_logger().info(
            f'work table collision box: frame={frame} '
            f'center={center.round(3).tolist()} size={size.round(3).tolist()}')
        return scene

    def _tick(self):
        if self._scene is None:
            self._scene = self._build_scene()
            if self._scene is None:
                return
        self._publish()

    def _publish(self):
        self._scene.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._scene)


def main():
    rclpy.init()
    node = TableCollision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
