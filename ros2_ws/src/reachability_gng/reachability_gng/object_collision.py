"""Turn segmented objects into MoveIt CollisionObjects + attach/detach for grasp.

Companion to collision_cloud (which keeps ALL objects in the octomap by default,
carving out only the grasp target): this node represents ONLY the chosen grasp
target as an exact CollisionObject BOX in the planning scene, so the gripper can
reach it, be ACM-allowed into it, and ATTACH it to the gripper (contact/carry)
then DETACH after release. Non-target objects are left as octomap obstacles.

Per camera it time-syncs depth + instance_segmentation, deprojects the TARGET
object's pixels to `world`, groups points by label across cameras, fits an
axis-aligned box, and publishes it as a PlanningScene diff on `/planning_scene`.
The target unseen for `ttl` seconds is removed. Boxes use the instance-seg label
as the CollisionObject id (stable per object), so an open-vocab detector can later
replace the Isaac segmentation with no change here. No target set -> no boxes.

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

from reachability_gng.object_localizer import (deproject, quat_to_R,
                                               resolve_target_ids)
from reachability_gng.pause_gate import PauseGate


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
        # Seg-mask edge pixels can read BACKGROUND depth (floor/wall/far table),
        # deprojecting to points metres away that blow the raw AABB up into a
        # room-filling box (planning then fails: start state in collision). Drop
        # the scattered far minority radially before fitting, and refuse to
        # publish anything larger than a graspable object could plausibly be.
        self.declare_parameter('reject_pct', 95.0)   # keep this % nearest median
        self.declare_parameter('max_box_size', 0.6)  # m; skip box if any axis over
        self.declare_parameter('ttl', 1.0)           # s an object persists unseen
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('touch_links', [''])  # gripper links allowed to touch
        # Grasp target: empty -> box NOTHING (all objects stay in the octomap as
        # obstacles, see collision_cloud). When set, ONLY the matching object
        # becomes a CollisionObject box (reachable + attachable for the grasp).
        self.declare_parameter('target_label', '')
        self.declare_parameter('target_id', -1)

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.padding = float(self.get_parameter('padding').value)
        self.reject_pct = float(self.get_parameter('reject_pct').value)
        self.max_box_size = float(self.get_parameter('max_box_size').value)
        self.ttl = float(self.get_parameter('ttl').value)
        self.touch_links = [s for s in self.get_parameter('touch_links').value if s]
        self.target_label = str(self.get_parameter('target_label').value)
        self.target_id = int(self.get_parameter('target_id').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Freeze the CollisionObject republish during a pick (see pause_gate) so
        # the 0.5 s re-ADD doesn't bump the scene version mid-plan.
        self.declare_parameter('pause_timeout', 8.0)   # resume 8 s after heartbeat stops
        self.gate = PauseGate(self, float(self.get_parameter('pause_timeout').value))
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
        # Runtime target selection: publish a label on /grasp_target to box that
        # object (reachable + attachable) without a restart; empty string clears it
        # (all objects then stay octomap-only).
        self.create_subscription(String, '/grasp_target',
                                 self._on_grasp_target, 10)

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
        target_ids = resolve_target_ids(
            self._labels[ns], self.target_label, self.target_id)
        for inst_id, label in self._labels[ns].items():
            # ONLY the chosen grasp target becomes a CollisionObject box (so it can
            # be reached + attached); every other object stays an octomap obstacle
            # (collision_cloud keeps them). No target set -> box nothing.
            if not target_ids or inst_id not in target_ids:
                continue
            mask = (seg == inst_id) & valid
            ys, xs = np.nonzero(mask)
            if xs.size < self.min_pixels:
                continue
            pts = deproject(xs[::self.stride], ys[::self.stride],
                            depth[ys[::self.stride], xs[::self.stride]],
                            fx, fy, cx, cy)
            self._latest[ns][label] = (pts @ R.T + T, now)

    def _on_grasp_target(self, msg):
        label = msg.data.strip()
        if label == self.target_label:
            return
        self.target_label = label
        self.target_id = -1   # label is the runtime interface; clear numeric
        self.get_logger().info(
            f"grasp target -> '{label}' (boxed as CollisionObject)" if label
            else 'grasp target cleared (no object boxes; all octomap obstacles)')

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

    def _reject_outliers(self, pts):
        """Drop the scattered far minority (seg-mask leakage onto background).

        Keeps the `reject_pct` fraction of points nearest the median, radially,
        so a compact object survives intact but stray metres-away points that
        would explode the AABB are removed."""
        if len(pts) < 10:
            return pts
        d = np.linalg.norm(pts - np.median(pts, axis=0), axis=1)
        keep = d <= np.percentile(d, self.reject_pct)
        return pts[keep] if keep.any() else pts

    # ---- world publish ------------------------------------------------------
    def _publish(self):
        if self.gate.paused():
            return                 # keep the scene frozen while a pick plans
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
            pts = self._reject_outliers(np.concatenate(chunks, axis=0))
            lo = pts.min(axis=0) - self.padding
            hi = pts.max(axis=0) + self.padding
            size = np.maximum(hi - lo, 1e-3)
            if np.any(size > self.max_box_size):
                # seg-mask leaked to background: a room-sized box would break
                # planning. Skip it and mark the label published-but-inactive so
                # the removal loop below REMOVEs any stale huge box for it that is
                # already in the scene (self-healing across a node restart too).
                self.get_logger().warn(
                    f"'{label}' box {size.round(2)} m exceeds max "
                    f"{self.max_box_size} m -- likely seg-mask leak; not published")
                self._published.add(label)
                continue
            center = (lo + hi) / 2
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
            scene.is_diff = True          # merge into the scene, not replace it
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
