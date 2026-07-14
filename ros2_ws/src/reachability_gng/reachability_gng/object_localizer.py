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
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
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
    """Merge detections (label, xyz, top_z) whose centroids are within `radius`.

    Dedups an object seen by multiple cameras; averages the centroid and keeps
    the HIGHEST observed top (each camera sees a partial top). Returns
    [(label, xyz, top_z), ...].
    """
    merged = []  # [label, xyz, top_z, count]
    for label, xyz, top in dets:
        for m in merged:
            if np.linalg.norm(m[1] - xyz) <= radius:
                m[1] = (m[1] * m[3] + xyz) / (m[3] + 1)
                m[2] = max(m[2], top)
                m[3] += 1
                break
        else:
            merged.append([label, np.asarray(xyz, float).copy(), float(top), 1])
    return [(m[0], m[1], m[2]) for m in merged]


# Color words seg_router prefixes onto a label (see seg_router.name_color). The
# color is OPTIONAL disambiguation: matching falls back to the class name alone
# when the exact color+class label is not present.
_COLORS = {'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple',
           'pink', 'brown', 'white', 'gray', 'grey', 'black'}


def _strip_color(label):
    """Drop a single leading color word: 'yellow banana' -> 'banana'."""
    parts = label.strip().lower().split()
    if len(parts) > 1 and parts[0] in _COLORS:
        return ' '.join(parts[1:])
    return ' '.join(parts)


def resolve_target_ids(labels, target_label, target_id=-1):
    """Seg ids in this camera's {id: label} map that are the grasp TARGET.

    Returns None when no target is configured (target_label=="" and
    target_id<0) -> the caller treats EVERY object as the target (legacy
    behaviour). Matches the exact color+class label first; when that is absent
    (e.g. the detector measured a different color this frame) it falls back to
    matching by the object NAME (class) alone -- color is only optional
    disambiguation, so a color mismatch never drops the target. target_id is the
    last resort when the label is absent in THIS camera's map.
    """
    if not target_label and target_id < 0:
        return None
    ids = set()
    if target_label:
        ids = {i for i, lab in labels.items() if lab == target_label}
        if not ids:
            # color optional -> match the class name, ignoring the color prefix
            tgt_cls = _strip_color(target_label)
            ids = {i for i, lab in labels.items()
                   if _strip_color(lab) == tgt_cls}
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
        # Association is GLOBAL (Hungarian) rather than greedy nearest-neighbour
        # so two objects that pass close never swap identity; a detection can
        # only bind to a track within track_match_radius (gating) and same-class
        # pairs are preferred (class_cost added to cross-class pairs). A track is
        # only PUBLISHED once seen in confirm_frames separate cycles, which
        # rejects 1-frame detector flicker (spurious YOLOE blobs). The published
        # CLASS switches only when a challenger out-votes the committed class by
        # label_hysteresis_margin (vote hysteresis), so a single mislabelled
        # frame does not rename a stable object. Identity itself comes from the
        # spatial track id, never the (per-frame, class-only) label -- so this is
        # source-agnostic (Isaac GT and YOLOE alike).
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('class_cost', 0.10)
        self.declare_parameter('label_hysteresis_margin', 2)
        # Votes DECAY each frame (multiply by vote_decay before adding the new
        # one) so the committed label reflects the RECENT window (~1/(1-decay)
        # frames), not the whole lifetime. Without decay, votes accumulate
        # unbounded and an early label becomes permanently sticky -- e.g. after
        # switching seg_source isaac->yoloe the object's class would stay 'obj_N'
        # for a very long time. Decay keeps flicker-rejection (a 1-frame
        # challenger can't beat the steady count by the margin) while letting a
        # SUSTAINED new label take over in a handful of frames. 1.0 = no decay.
        self.declare_parameter('vote_decay', 0.9)
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
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.class_cost = float(self.get_parameter('class_cost').value)
        self.label_hysteresis_margin = int(
            self.get_parameter('label_hysteresis_margin').value)
        self.vote_decay = float(self.get_parameter('vote_decay').value)
        self.target_label = str(self.get_parameter('target_label').value)
        self.target_id = int(self.get_parameter('target_id').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._K = {ns: None for ns in nss}        # ns -> (fx, fy, cx, cy)
        self._labels = {ns: {} for ns in nss}     # ns -> {id: label}
        self._label_conf = {}                     # label -> latest YOLOE conf (display)
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
            self.create_subscription(
                String, f'/{ns}/instance_segmentation_conf',
                lambda m, ns=ns: self._on_conf(ns, m), 1)
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
        # Target's stable, size-adaptive box (center + size) so the executor can
        # stand the EE off the object TOP -- computed from the SAME tracked points
        # as the centroid (so it inherits the tracking's stability), unlike
        # object_collision's per-frame AABB. Latched. Empty [] when no target.
        box_qos = QoSProfile(depth=1)
        box_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.box_pub = self.create_publisher(
            String, '/target_collision_boxes', box_qos)
        # Stable-identity handle for downstream targeting: JSON list of the
        # confirmed tracks [{tid,label,x,y,z}], SAME order as /detected_objects,
        # so reach_fusion/target_cli can target a persistent track id (#<tid>)
        # instead of the flaky per-frame class label. Latched. Source-agnostic.
        self.tracks_pub = self.create_publisher(
            String, '/detected_objects/tracks', box_qos)
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

    def _on_conf(self, ns, m):
        """Correlate this camera's {id: conf} with its labels -> {label: conf}.

        Display only: keyed by label so the CLI/marker can show YOLOE's
        confidence next to each object. Isaac ground truth publishes {} (no
        conf), which simply leaves prior values untouched.
        """
        try:
            raw = json.loads(m.data)
        except (ValueError, TypeError):
            return
        labels = self._labels[ns]
        for key, val in raw.items():
            if not key.isdigit():
                continue
            label = labels.get(int(key))
            if label:
                self._label_conf[label] = float(val)

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
            pts_world = pts @ R.T + T
            c_world = np.median(pts_world, axis=0)
            # object TOP in world z; 95th pct (not max) rejects a few noisy far
            # pixels so the stand-off height doesn't jump on outliers.
            top_z = float(np.percentile(pts_world[:, 2], 95))
            dets.append((label, c_world, top_z))
        self._dets[ns] = dets

    def _target_names(self):
        """Set of target label names, or None when no target is configured."""
        if self.target_label:
            return {self.target_label}
        if self.target_id >= 0:
            return {lab for labs in self._labels.values()
                    for i, lab in labs.items() if i == self.target_id}
        return None

    def _associate(self, cur):
        """Global (Hungarian) match of detections `cur` to existing tracks.

        Returns (pairs, unmatched): pairs = [(det_idx, tid)] accepted within the
        gating radius; unmatched = det indices with no track. Cost = Euclidean
        distance + class_cost when the detection's class differs from the track's
        committed class; pairs beyond track_match_radius are forbidden (BIG) so a
        detection never jumps onto a far track. Global assignment (vs greedy NN)
        keeps two objects that pass close from swapping identity.
        """
        tids = list(self._tracks)
        if not tids or not cur:
            return [], list(range(len(cur)))
        from scipy.optimize import linear_sum_assignment
        BIG = 1e6
        C = np.full((len(cur), len(tids)), BIG)
        for r, (label, xyz, _top) in enumerate(cur):
            cls = _strip_color(label)
            xyz = np.asarray(xyz, float)
            for c, tid in enumerate(tids):
                tr = self._tracks[tid]
                d = float(np.linalg.norm(tr['xyz'] - xyz))
                if d <= self.track_match_radius:
                    C[r, c] = d + (0.0 if _strip_color(tr['label']) == cls
                                   else self.class_cost)
        rows, cols = linear_sum_assignment(C)
        pairs, matched = [], set()
        for r, c in zip(rows, cols):
            if C[r, c] < BIG:
                pairs.append((r, tids[c]))
                matched.add(r)
        return pairs, [r for r in range(len(cur)) if r not in matched]

    def _commit_label(self, tr):
        """Vote hysteresis: switch the track's published class only when a
        challenger out-votes the committed class by label_hysteresis_margin, so
        one mislabelled frame never renames a stable object."""
        top_label, top_n = tr['votes'].most_common(1)[0]
        cur = tr.get('label')
        if cur is None:
            tr['label'] = top_label
        elif (top_label != cur
              and top_n - tr['votes'][cur] >= self.label_hysteresis_margin):
            tr['label'] = top_label
        return tr['label']

    def _update_tracks(self, cur):
        """Fold this cycle's detections into persistent, stable-id tracks.

        `cur` is [(label, xyz, top_z)] fused across cameras for THIS cycle.
        Detections are matched to tracks by GLOBAL (Hungarian) assignment within
        track_match_radius (see _associate); a matched track is EMA-smoothed and
        its label voted, an unmatched detection starts a new track. Tracks older
        than track_ttl are dropped, then near-duplicates merged. Only tracks seen
        in >= confirm_frames cycles are returned (rejects flicker), as
        [(tid, label, xyz, top)] ordered by stable id -- persistent, deduped,
        vote-hysteresis labelled. Identity is the tid, not the class label.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        pairs, unmatched = self._associate(cur)
        for det_idx, tid in pairs:
            label, xyz, top = cur[det_idx]
            tr = self._tracks[tid]
            a = self.track_smooth
            tr['xyz'] = a * tr['xyz'] + (1.0 - a) * np.asarray(xyz, float)
            tr['top'] = a * tr['top'] + (1.0 - a) * float(top)
            if self.vote_decay < 1.0:      # forget old frames -> bounded window
                for k in list(tr['votes']):
                    tr['votes'][k] *= self.vote_decay
                    if tr['votes'][k] < 0.05:
                        del tr['votes'][k]
            tr['votes'][label] += 1.0
            tr['last'] = now
            tr['hits'] += 1
            self._commit_label(tr)
        for det_idx in unmatched:
            label, xyz, top = cur[det_idx]
            self._tracks[self._next_tid] = {
                'votes': Counter([label]), 'label': label,
                'xyz': np.asarray(xyz, float), 'top': float(top),
                'last': now, 'hits': 1}
            self._next_tid += 1
        self._tracks = {tid: tr for tid, tr in self._tracks.items()
                        if now - tr['last'] <= self.track_ttl}
        self._merge_tracks()
        return [(tid, tr['label'], tr['xyz'], tr['top'])
                for tid, tr in sorted(self._tracks.items())
                if tr['hits'] >= self.confirm_frames]

    def _merge_tracks(self):
        """Merge track pairs within track_merge_dist (one object seen by two
        cameras -> one track). Keeps the older (smaller) id, sums the vote
        histograms, averages position, and re-commits the voted label."""
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
                    ta['top'] = max(ta['top'], tb['top'])
                    ta['last'] = max(ta['last'], tb['last'])
                    ta['hits'] = max(ta['hits'], tb['hits'])
                    self._commit_label(ta)
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
        dup_counts = Counter(lab for _tid, lab, _, _ in merged)
        dup_seen = {}

        for i, (tid, label, xyz, top) in enumerate(merged):
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

            # YOLOE confidence for this object (ns 'conf', id aligned to the
            # pose index) -- consumed by pick_cli for its list, absent for Isaac.
            conf = self._label_conf.get(label)
            if conf is not None:
                cm = Marker()
                cm.header.frame_id = self.world_frame
                cm.ns = 'conf'
                cm.id = i
                cm.type = Marker.TEXT_VIEW_FACING
                cm.action = Marker.ADD
                cm.pose.orientation.w = 1.0
                cm.scale.z = 0.01
                cm.color = ColorRGBA(a=0.0)   # data channel only, not drawn
                cm.text = f'{conf:.2f}'
                ma.markers.append(cm)

        self.pose_pub.publish(pa)
        self.marker_pub.publish(ma)

        # Stable-identity handles (same order as the PoseArray) for downstream
        # targeting by persistent track id rather than the per-frame label.
        self.tracks_pub.publish(String(data=json.dumps(
            [{'tid': int(tid), 'label': label,
              'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2]),
              'conf': float(self._label_conf.get(label, 0.0))}
             for tid, label, xyz, _top in merged])))

        # republish only the grasp target's pose + its size-adaptive box (if a
        # target is configured + seen). The box's TOP (center_z + size_z/2) is the
        # object top, so the executor stands the EE off ABOVE it -- dynamic per
        # object height. Publish [] when no target so the executor clears it.
        names = self._target_names()
        boxes = []
        if names:
            for tid, label, xyz, top in merged:
                if label in names:
                    ts = PoseStamped()
                    ts.header.frame_id = self.world_frame
                    ts.header.stamp = stamp
                    ts.pose.position = Point(
                        x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
                    ts.pose.orientation.w = 1.0
                    self.target_pub.publish(ts)
                    # size_z chosen so center_z + size_z/2 == top (object top).
                    half = max(float(top) - float(xyz[2]), 1e-3)
                    boxes.append({
                        'id': label,
                        'center': [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                        'size': [0.05, 0.05, 2.0 * half]})
                    break
        self.box_pub.publish(String(data=json.dumps(boxes)))


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
