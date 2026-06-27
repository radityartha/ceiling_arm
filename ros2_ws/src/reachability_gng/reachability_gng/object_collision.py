"""Turn segmented objects into MoveIt CollisionObjects + attach/detach for grasp.

Companion to collision_cloud (which keeps objects OUT of the octomap): this node
represents each detected object as an exact CollisionObject BOX in the planning
scene so MoveIt avoids every object during transit, and lets the grasp pipeline
ATTACH the target object to the gripper (so the gripper may contact/carry it)
then DETACH it after release.

Per camera it time-syncs depth + instance_segmentation, deprojects each object's
pixels (seg id > 1) to `world`, groups points by label across cameras, fits an
axis-aligned box per object, and publishes them as a PlanningScene diff on
`/planning_scene`. Objects unseen for `ttl` seconds are removed. Boxes use the
instance-seg label as the CollisionObject id (stable per object), so an
open-vocab detector can later replace the Isaac segmentation with no change here.

Attach / detach via a std_msgs/String command on `/object_collision/command`
(no custom .srv needed -- this package is ament_python):

    attach <object_id> <gripper_link>      # e.g. attach red_box t1_a1_gripper_base_link
    detach <object_id>

`touch_links` (param) lists the gripper links allowed to contact the attached
object (fingers + gripper base) so holding it is not flagged as a collision.

    ros2 run reachability_gng object_collision
    ros2 topic pub --once /object_collision/command std_msgs/String \\
        "{data: 'attach red_box t1_a1_gripper_base_link'}"
"""
from __future__ import annotations

import json
import time

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import deproject, quat_to_R


class ObjectCollision(Node):
    def __init__(self):
        super().__init__('object_collision')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('min_pixels', 30)
        self.declare_parameter('stride', 2)
        self.declare_parameter('padding', 0.01)      # inflate each AABB (m)
        self.declare_parameter('ttl', 1.0)           # s an object persists unseen
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('touch_links', [''])  # gripper links allowed to touch

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.padding = float(self.get_parameter('padding').value)
        self.ttl = float(self.get_parameter('ttl').value)
        self.touch_links = [s for s in self.get_parameter('touch_links').value if s]
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._K = {ns: None for ns in nss}
        self._labels = {ns: {} for ns in nss}
        # per camera: label -> (points_world Nx3, last_seen_monotonic)
        self._latest = {ns: {} for ns in nss}
        self._boxes = {}        # label -> (center3, size3)  current published boxes
        self._published = set()  # ids currently ADDed to the world scene
        self._attached = {}     # id -> gripper_link (excluded from world publish)
        self._syncs = []

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        self.create_subscription(String, '/object_collision/command',
                                 self._on_command, 10)

        for ns in nss:
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, ns=ns: self._on_info(ns, m), 1)
            self.create_subscription(
                String, f'/{ns}/instance_segmentation_labels',
                lambda m, ns=ns: self._on_labels(ns, m), 1)
            depth_sub = message_filters.Subscriber(self, Image, f'/{ns}/depth')
            seg_sub = message_filters.Subscriber(
                self, Image, f'/{ns}/instance_segmentation')
            sync = message_filters.ApproximateTimeSynchronizer(
                [depth_sub, seg_sub], queue_size=5, slop=0.1)
            sync.registerCallback(lambda d, s, ns=ns: self._on_pair(ns, d, s))
            self._syncs.append(sync)

        self.create_timer(
            float(self.get_parameter('publish_period').value), self._publish)
        self.get_logger().info(
            f'object_collision up; cameras={nss}, touch_links={self.touch_links}')

    # ---- subscriber callbacks ----------------------------------------------
    def _on_info(self, ns, m):
        k = m.k
        self._K[ns] = (k[0], k[4], k[2], k[5])

    def _on_labels(self, ns, m):
        try:
            raw = json.loads(m.data)
        except (ValueError, TypeError):
            return
        d = {}
        for key, val in raw.items():
            if not key.isdigit() or val in ('BACKGROUND', 'UNLABELLED'):
                continue
            d[int(key)] = str(val).rsplit('/', 1)[-1]
        if d:
            self._labels[ns] = d

    def _decode(self, msg, dtype):
        a = np.frombuffer(bytes(msg.data), dtype=dtype)
        cols = msg.step // np.dtype(dtype).itemsize
        return a.reshape(msg.height, cols)[:, :msg.width]

    def _on_pair(self, ns, depth_msg, seg_msg):
        if self._K[ns] is None or not self._labels[ns]:
            return
        fx, fy, cx, cy = self._K[ns]
        try:
            depth = self._decode(depth_msg, np.float32)
            seg = self._decode(seg_msg, np.int32)
        except ValueError:
            return
        if depth.shape != seg.shape:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        valid = np.isfinite(depth) & (depth > self.min_depth) & (depth < self.max_depth)
        now = time.monotonic()
        for inst_id, label in self._labels[ns].items():
            mask = (seg == inst_id) & valid
            ys, xs = np.nonzero(mask)
            if xs.size < self.min_pixels:
                continue
            pts = deproject(xs[::self.stride], ys[::self.stride],
                            depth[ys[::self.stride], xs[::self.stride]],
                            fx, fy, cx, cy)
            self._latest[ns][label] = (pts @ R.T + T, now)

    def _on_command(self, msg):
        parts = msg.data.split()
        if len(parts) >= 2 and parts[0] == 'attach':
            link = parts[2] if len(parts) >= 3 else None
            self._attach(parts[1], link)
        elif len(parts) >= 2 and parts[0] == 'detach':
            self._detach(parts[1])
        else:
            self.get_logger().warn(
                f"bad command '{msg.data}'; use 'attach <id> <link>' or 'detach <id>'")

    # ---- world publish ------------------------------------------------------
    def _publish(self):
        now = time.monotonic()
        # fuse latest in-TTL points per label across cameras
        fused = {}
        for ns in self._latest:
            for label, (pts, t) in list(self._latest[ns].items()):
                if now - t > self.ttl:
                    del self._latest[ns][label]
                    continue
                fused.setdefault(label, []).append(pts)

        scene = PlanningScene()
        scene.is_diff = True
        active = set()
        for label, chunks in fused.items():
            pts = np.concatenate(chunks, axis=0)
            lo = pts.min(axis=0) - self.padding
            hi = pts.max(axis=0) + self.padding
            center = (lo + hi) / 2
            size = np.maximum(hi - lo, 1e-3)
            self._boxes[label] = (center, size)
            active.add(label)
            if label in self._attached:
                continue   # attached objects are managed in the robot state
            scene.world.collision_objects.append(
                self._box_co(label, center, size, CollisionObject.ADD))
            self._published.add(label)

        # remove world objects that vanished (and aren't attached)
        for label in list(self._published):
            if label not in active and label not in self._attached:
                co = CollisionObject()
                co.id = label
                co.header.frame_id = self.world_frame
                co.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(co)
                self._published.discard(label)
                self._boxes.pop(label, None)

        if scene.world.collision_objects:
            scene.header.stamp = self.get_clock().now().to_msg()
            self.scene_pub.publish(scene)

    # ---- attach / detach ----------------------------------------------------
    def _attach(self, obj_id, link):
        if obj_id not in self._boxes:
            self.get_logger().warn(f"attach: object '{obj_id}' not detected")
            return
        if not link:
            self.get_logger().warn('attach: missing <gripper_link>')
            return
        center, size = self._boxes[obj_id]
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object = self._box_co(obj_id, center, size, CollisionObject.ADD)
        aco.touch_links = self.touch_links
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        # also drop it from the world so it isn't double-counted
        rm = CollisionObject()
        rm.id = obj_id
        rm.header.frame_id = self.world_frame
        rm.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(rm)
        self.scene_pub.publish(scene)
        self._attached[obj_id] = link
        self._published.discard(obj_id)
        self.get_logger().info(f"attached '{obj_id}' to {link}")

    def _detach(self, obj_id):
        link = self._attached.pop(obj_id, None)
        if link is None:
            self.get_logger().warn(f"detach: '{obj_id}' is not attached")
            return
        aco = AttachedCollisionObject()
        aco.link_name = link
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = self.world_frame
        co.operation = CollisionObject.REMOVE
        aco.object = co
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        self.scene_pub.publish(scene)
        self.get_logger().info(f"detached '{obj_id}' (returns to world scene)")

    def _box_co(self, obj_id, center, size, operation):
        co = CollisionObject()
        co.header.frame_id = self.world_frame
        co.id = obj_id
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = operation
        return co


def main():
    rclpy.init()
    node = ObjectCollision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
