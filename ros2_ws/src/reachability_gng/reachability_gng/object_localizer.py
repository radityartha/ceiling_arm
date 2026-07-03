"""Localize segmented objects in `world` by fusing one or more RGBD cameras.

For each camera namespace this node time-syncs depth + instance_segmentation,
deprojects the masked depth of every object instance to a 3D centroid in the
camera optical frame, transforms it to `world` via tf2, then fuses detections
across cameras (an object seen by >1 camera is merged). It publishes:

    /detected_objects          geometry_msgs/PoseArray         (frame `world`)
    /detected_objects/markers   visualization_msgs/MarkerArray  (spheres + labels)

The segmentation source is generic: it consumes an instance-id image (32SC1) +
an id->label JSON, so an open-vocab detector (YOLOE / YOLO-World) can later
replace the Isaac ground-truth publisher with no change downstream.

    ros2 run reachability_gng object_localizer
    ros2 topic echo /detected_objects
"""
from __future__ import annotations

import json
from collections import Counter

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_R(x, y, z, w):
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def deproject(xs, ys, z, fx, fy, cx, cy):
    """Pinhole back-projection of pixels (xs,ys) at depth z (along +Z optical).

    Returns (N,3) points in the ROS optical frame (x right, y down, z forward).
    """
    X = (xs - cx) * z / fx
    Y = (ys - cy) * z / fy
    return np.stack([X, Y, z], axis=1)


def fuse(dets, radius):
    """Merge detections (label, xyz) whose centroids are within `radius` (greedy).

    Dedups an object seen by multiple cameras; returns [(label, xyz), ...].
    """
    merged = []  # [label, xyz, count]
    for label, xyz in dets:
        for m in merged:
            if np.linalg.norm(m[1] - xyz) <= radius:
                m[1] = (m[1] * m[2] + xyz) / (m[2] + 1)
                m[2] += 1
                break
        else:
            merged.append([label, np.asarray(xyz, float).copy(), 1])
    return [(m[0], m[1]) for m in merged]


def resolve_target_ids(labels, target_label, target_id=-1):
    """Seg ids in this camera's {id: label} map that are the grasp TARGET.

    Returns None when no target is configured (target_label=="" and
    target_id<0) -> the caller treats EVERY object as the target (legacy
    behaviour). Matches by label name first (label basenames are stable across
    cameras even though the numeric seg ids are not), falling back to target_id
    only when the label is absent in THIS camera's map.
    """
    if not target_label and target_id < 0:
        return None
    ids = ({i for i, lab in labels.items() if lab == target_label}
           if target_label else set())
    if not ids and target_id >= 0 and target_id in labels:
        ids = {target_id}
    return ids


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('min_pixels', 20)
        self.declare_parameter('fuse_radius', 0.10)
        self.declare_parameter('publish_period', 0.5)
        # Temporal tracking: keeps the detected-object list stable across the
        # detector's per-frame flicker (esp. YOLOE at ~0.75 Hz). A detection is
        # matched to the nearest existing track within track_match_radius, its
        # position EMA-smoothed by track_smooth; a track survives track_ttl
        # seconds without a fresh hit (bridges dropped frames) and keeps a stable
        # id so the published order/index does not jump.
        self.declare_parameter('track_ttl', 1.5)
        self.declare_parameter('track_match_radius', 0.12)
        self.declare_parameter('track_smooth', 0.5)
        # Two tracks closer than this are the SAME object (e.g. one object seen
        # by both cameras that fuse() missed) -> merged, so no duplicate rows.
        self.declare_parameter('track_merge_dist', 0.15)
        # Grasp target: when set, the matching object's pose is republished on
        # /target_object (PoseStamped). Empty -> no target topic (legacy).
        self.declare_parameter('target_label', '')
        self.declare_parameter('target_id', -1)

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.fuse_radius = float(self.get_parameter('fuse_radius').value)
        self.track_ttl = float(self.get_parameter('track_ttl').value)
        self.track_match_radius = float(
            self.get_parameter('track_match_radius').value)
        self.track_smooth = float(self.get_parameter('track_smooth').value)
        self.track_merge_dist = float(
            self.get_parameter('track_merge_dist').value)
        self.target_label = str(self.get_parameter('target_label').value)
        self.target_id = int(self.get_parameter('target_id').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._K = {ns: None for ns in nss}        # ns -> (fx, fy, cx, cy)
        self._labels = {ns: {} for ns in nss}     # ns -> {id: label}
        self._dets = {ns: [] for ns in nss}       # ns -> [(label, xyz_world)]
        self._syncs = []                          # keep refs alive
        self._tracks = {}    # tid -> {'label', 'xyz'(np3), 'last'(sec)}
        self._next_tid = 0   # monotonic stable-id counter

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

        self.pose_pub = self.create_publisher(PoseArray, '/detected_objects', 1)
        self.target_pub = self.create_publisher(PoseStamped, '/target_object', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_objects/markers', 1)
        # Runtime target selection: publish a label on /grasp_target to make that
        # object the target on /target_object without a restart; empty clears it.
        self.create_subscription(String, '/grasp_target',
                                 self._on_grasp_target, 10)
        self.create_timer(
            float(self.get_parameter('publish_period').value), self._publish)
        self.get_logger().info(f'object_localizer up; cameras={nss}')

    # ---- subscriber callbacks ----------------------------------------------
    def _on_grasp_target(self, msg):
        label = msg.data.strip()
        if label == self.target_label:
            return
        self.target_label = label
        self.target_id = -1   # label is the runtime interface; clear numeric
        self.get_logger().info(
            f"grasp target -> '{label}'" if label else 'grasp target cleared')

    def _on_info(self, ns, m):
        k = m.k
        self._K[ns] = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _on_labels(self, ns, m):
        try:
            raw = json.loads(m.data)
        except (ValueError, TypeError):
            return
        d = {}
        for key, val in raw.items():
            if not key.isdigit() or val in ('BACKGROUND', 'UNLABELLED'):
                continue
            d[int(key)] = str(val).rsplit('/', 1)[-1]  # basename of prim path
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
        dets = []
        for inst_id, label in self._labels[ns].items():
            mask = (seg == inst_id) & valid
            ys, xs = np.nonzero(mask)
            if xs.size < self.min_pixels:
                continue
            pts = deproject(xs, ys, depth[ys, xs], fx, fy, cx, cy)
            c_world = R @ np.median(pts, axis=0) + T
            dets.append((label, c_world))
        self._dets[ns] = dets

    def _target_names(self):
        """Set of target label names, or None when no target is configured."""
        if self.target_label:
            return {self.target_label}
        if self.target_id >= 0:
            return {lab for labs in self._labels.values()
                    for i, lab in labs.items() if i == self.target_id}
        return None

    def _update_tracks(self, cur):
        """Fold this cycle's detections into persistent, stable-id tracks.

        `cur` is [(label, xyz)] fused across cameras for THIS cycle. Each
        detection updates the nearest track within track_match_radius (position
        EMA-smoothed, label added to that track's vote histogram); unmatched ->
        new track. Tracks older than track_ttl are dropped, then tracks closer
        than track_merge_dist are merged (kills cross-camera doubles of one
        object). Returns alive tracks as [(voted_label, xyz)] ordered by stable
        id -- persistent, deduplicated, and MAJORITY-VOTED so the class stops
        flickering frame to frame.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        used = set()
        for label, xyz in cur:
            xyz = np.asarray(xyz, float)
            best_tid, best_d = None, self.track_match_radius
            for tid, tr in self._tracks.items():
                if tid in used:
                    continue
                d = float(np.linalg.norm(tr['xyz'] - xyz))
                if d <= best_d:
                    best_d, best_tid = d, tid
            if best_tid is None:
                self._tracks[self._next_tid] = {
                    'votes': Counter([label]), 'xyz': xyz, 'last': now}
                used.add(self._next_tid)
                self._next_tid += 1
            else:
                tr = self._tracks[best_tid]
                a = self.track_smooth
                tr['xyz'] = a * tr['xyz'] + (1.0 - a) * xyz
                tr['votes'][label] += 1
                tr['last'] = now
                used.add(best_tid)
        self._tracks = {tid: tr for tid, tr in self._tracks.items()
                        if now - tr['last'] <= self.track_ttl}
        self._merge_tracks()
        return [(tr['votes'].most_common(1)[0][0], tr['xyz'])
                for _, tr in sorted(self._tracks.items())]

    def _merge_tracks(self):
        """Merge track pairs within track_merge_dist (one object seen by two
        cameras -> one track). Keeps the older (smaller) id, sums the vote
        histograms, and averages position."""
        tids = sorted(self._tracks)
        for i, a in enumerate(tids):
            ta = self._tracks.get(a)
            if ta is None:
                continue
            for b in tids[i + 1:]:
                tb = self._tracks.get(b)
                if tb is None:
                    continue
                if float(np.linalg.norm(ta['xyz'] - tb['xyz'])) <= self.track_merge_dist:
                    ta['votes'] += tb['votes']
                    ta['xyz'] = 0.5 * (ta['xyz'] + tb['xyz'])
                    ta['last'] = max(ta['last'], tb['last'])
                    del self._tracks[b]

    # ---- output -------------------------------------------------------------
    def _publish(self):
        alld = [d for dets in self._dets.values() for d in dets]
        merged = self._update_tracks(fuse(alld, self.fuse_radius))

        stamp = self.get_clock().now().to_msg()
        pa = PoseArray()
        pa.header.frame_id = self.world_frame
        pa.header.stamp = stamp

        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Number same-named objects for the DISPLAY only: 'yellow bottle' ->
        # 'yellow bottle 1', 'yellow bottle 2' when >1 share a label (order is
        # stable via the track ids). The underlying label stays unnumbered for
        # target matching; pick_cli strips the suffix before /grasp_target.
        dup_counts = Counter(lab for lab, _ in merged)
        dup_seen = {}

        for i, (label, xyz) in enumerate(merged):
            pose = Pose()
            pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            pose.orientation.w = 1.0
            pa.poses.append(pose)

            sphere = Marker()
            sphere.header.frame_id = self.world_frame
            sphere.ns = 'objects'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
            sphere.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            ma.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.world_frame
            text.ns = 'labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]) + 0.08)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.05
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            if dup_counts[label] > 1:
                dup_seen[label] = dup_seen.get(label, 0) + 1
                text.text = f'{label} {dup_seen[label]}'
            else:
                text.text = label
            ma.markers.append(text)

        self.pose_pub.publish(pa)
        self.marker_pub.publish(ma)

        # republish only the grasp target's pose (if one is configured + seen)
        names = self._target_names()
        if names:
            for label, xyz in merged:
                if label in names:
                    ts = PoseStamped()
                    ts.header.frame_id = self.world_frame
                    ts.header.stamp = stamp
                    ts.pose.position = Point(
                        x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
                    ts.pose.orientation.w = 1.0
                    self.target_pub.publish(ts)
                    break


def main():
    rclpy.init()
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
