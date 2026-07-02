"""Publish a THIN, hole-free collision slab at the sensed table surface.

Occlusion shadows leave gaps in the octomap that the planner treats as free, so
it can route the arm THROUGH the table surface -- dangerous. The octomap cannot
guarantee a hole-free surface (occlusion is physical, and predicted fills cannot
be injected without the updater carving real voxels). So instead we give MoveIt a
SOLID collision surface: fit the dominant horizontal plane + its xy extent LIVE
from the fused collision clouds each cycle, and publish one thin box
CollisionObject at the table top. Solid -> no holes by construction -> the arm
cannot pass through. Sensor-derived (fit from the live cloud, not from any model/
code) and re-fit continuously, so it tracks the real surface.

It is a THIN slab at the top (slab_thickness, e.g. 3 cm), NOT a floor-to-top box,
so it does not block anything below and objects resting ON the table (above the
slab top) stay graspable -- the grasp target is additionally handled by
object_collision + the executor's ACM allowance.

    /planning_scene   moveit_msgs/PlanningScene (diff)   id = table_slab

    ros2 run reachability_gng table_slab
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import SolidPrimitive
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


class TableSlab(Node):
    def __init__(self):
        super().__init__('table_slab')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('cloud_topic_suffix', 'collision_cloud')
        self.declare_parameter('rate', 2.0)             # Hz to re-fit + publish
        self.declare_parameter('min_z', 0.3)            # plane search band
        self.declare_parameter('max_z', 1.9)
        self.declare_parameter('z_bin', 0.01)           # top-layer histogram bin
        self.declare_parameter('plane_thickness', 0.03)  # inlier band around top
        self.declare_parameter('min_inliers', 500)
        self.declare_parameter('xy_percentile', 1.0)    # robust extent clip
        self.declare_parameter('slab_thickness', 0.03)  # box z-thickness at the top
        self.declare_parameter('object_id', 'table_slab')

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('cloud_topic_suffix').value
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.z_bin = float(self.get_parameter('z_bin').value)
        self.thick = float(self.get_parameter('plane_thickness').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.pct = float(self.get_parameter('xy_percentile').value)
        self.slab = float(self.get_parameter('slab_thickness').value)
        self.object_id = str(self.get_parameter('object_id').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._latest = {ns: None for ns in nss}
        self._last = None       # last good (center, size) -> republish if a fit fails
        for ns in nss:
            self.create_subscription(
                PointCloud2, f'/{ns}/{self.suffix}',
                lambda m, ns=ns: self._on_cloud(ns, m), 1)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)
        self.get_logger().info(
            f'table_slab up; cameras={nss}, slab_thickness={self.slab} m '
            '(live sensor-fit solid table surface for collision)')

    def _on_cloud(self, ns, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        a = np.frombuffer(bytes(msg.data), np.uint8)
        a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
        a = a[np.isfinite(a).all(axis=1)]
        if a.size == 0:
            self._latest[ns] = None
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        self._latest[ns] = a @ R.T + np.array([t.x, t.y, t.z])

    def _fit(self, pts):
        band = pts[(pts[:, 2] >= self.min_z) & (pts[:, 2] <= self.max_z)]
        if len(band) < self.min_inliers:
            return None
        # dominant horizontal layer = densest z bin (the table top)
        edges = np.arange(self.min_z, self.max_z + self.z_bin, self.z_bin)
        hist, _ = np.histogram(band[:, 2], bins=edges)
        top_z = float(edges[hist.argmax()] + self.z_bin / 2)
        inliers = band[np.abs(band[:, 2] - top_z) <= self.thick]
        if len(inliers) < self.min_inliers:
            return None
        xmin, xmax = np.percentile(inliers[:, 0], [self.pct, 100 - self.pct])
        ymin, ymax = np.percentile(inliers[:, 1], [self.pct, 100 - self.pct])
        # thin slab whose TOP face sits at top_z (objects rest above it)
        zmax, zmin = top_z, top_z - self.slab
        center = np.array([(xmin + xmax) / 2, (ymin + ymax) / 2,
                           (zmin + zmax) / 2], float)
        size = np.array([max(xmax - xmin, 1e-3), max(ymax - ymin, 1e-3),
                         self.slab], float)
        return center, size

    def _tick(self):
        clouds = [c for c in self._latest.values() if c is not None and len(c)]
        box = self._fit(np.concatenate(clouds, axis=0)) if clouds else None
        if box is not None:
            self._last = box
        elif self._last is not None:
            box = self._last            # keep the last good slab if a fit fails
        else:
            return
        center, size = box
        self.pub.publish(self._make_scene(center, size))

    def _make_scene(self, center, size):
        co = CollisionObject()
        co.header.frame_id = self.world_frame
        co.id = self.object_id
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        return scene


def main():
    rclpy.init()
    node = TableSlab()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
