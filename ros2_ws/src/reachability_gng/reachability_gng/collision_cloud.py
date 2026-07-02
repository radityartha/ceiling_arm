"""Publish the ENVIRONMENT depth reading (per camera) for MoveIt's octomap.

For collision avoidance MoveIt should see the WHOLE scene -- the work table,
fixtures, walls, AND the objects -- so by default this node deprojects every valid
depth pixel (objects included) so the octomap is dense and matches the sensor.

The ONE exception is the chosen grasp target: once an object is selected as the
target (target_label / target_id), its pixels are carved OUT of this cloud so it
does not become octomap voxels -- otherwise the gripper could not reach it (it
would collide with the target's own voxels) and, since the octomap is a single
ACM entity, you could not allow the gripper through just that object. The target
is instead represented by object_collision.py as an exact CollisionObject box that
can be reached, ACM-allowed, and attached to the gripper. Everything else stays in
the octomap. Published on `/<ns>/collision_cloud` in the camera OPTICAL frame.

MoveIt's PointCloudOctomapUpdater (workcell_moveit_config/config/sensors_3d.yaml)
voxelizes both cameras' clouds, self-filters the robot's own links, and the
planners avoid the resulting environment voxels. We publish in the optical frame
(not `world`) so the updater keeps the sensor origin and can RAY-CARVE free space
along each ray -- that clears moving-arm voxels incrementally, so the whole-map
octomap_refresher wipe is no longer needed. The earlier empty-octomap failure of
the optical path was the sim-time depth stamp, not the frame: we now stamp the
cloud wall-now (see _on_pair), which matches the robot TF (use_sim_time:=false),
and world<-optical is a static TF, so the updater's transform + self-filter both
resolve. On real hardware (wall-clock drivers) this is the natural, stable path.

    /<ns>/collision_cloud   sensor_msgs/PointCloud2   (frame <ns>_camera_optical)

    ros2 run reachability_gng collision_cloud
"""
from __future__ import annotations

import json
import os

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import (deproject, quat_to_R,
                                               resolve_target_ids)
from reachability_gng.pause_gate import PauseGate


class CollisionCloud(Node):
    def __init__(self):
        super().__init__('collision_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        # Publish in the camera OPTICAL frame and let MoveIt's octomap updater do
        # the optical->world transform, so it keeps the sensor origin and can
        # RAY-CARVE (clear free space along each ray) -- that clears moving-arm
        # voxels incrementally, removing the need for the whole-map octomap_refresher
        # wipe (the earlier world-frame workaround lost carving -> stale trails ->
        # churn). world<-optical is a static TF and the cloud is stamped wall-now
        # (below), so MoveIt's transform + robot self-filter both resolve correctly.
        # `world_frame` is now only the octomap map frame used for the TF-ready guard.
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('object_seg_id', 1)   # drop pixels with seg id > this
        self.declare_parameter('stride', 3)           # pixel subsample (1 = full res)
        # Grasp target: empty -> drop EVERY object from the octomap (legacy,
        # all handled by object_collision boxes). When set, drop ONLY the target
        # so non-target objects remain as octomap obstacles.
        self.declare_parameter('target_label', '')
        self.declare_parameter('target_id', -1)
        # Static geometry mapped by map_static (the work table, cabinet, ...) is
        # published as exact solid CollisionObject boxes by static_collision, so we
        # DROP any octomap point that falls inside a mapped box: that surface is
        # already covered by the reliable box, and this removes the octomap's holey
        # occlusion "shadow" voxels on it. Exclusion volume == the box (margin=0),
        # so everything removed stays covered -> no collision gap. Auto-reloads the
        # map file when it changes, so mapping AFTER launch takes effect w/o restart.
        self.declare_parameter('static_exclude', True)
        self.declare_parameter('static_map_file', '/tmp/static_geometry.npz')
        self.declare_parameter('static_margin', 0.0)
        # Live, per-frame occlusion hole-fill (sensor-driven, no saved prior). A
        # depth camera cannot see behind/under a standing object, so its shadow on
        # a continuous surface (e.g. the table) reads as empty -> holey octomap.
        # Each frame we grow trusted depth into those holes from their own
        # neighbours (iterative closing), but ONLY across a small depth step
        # (fill_depth_tol) so we never bridge distinct surfaces (table->floor) into
        # a phantom wall. Fills the object footprint + its shadow at the surrounding
        # surface depth. fill_iters bounds the max hole width (~iters*stride px).
        # Off by default: the per-frame depth-image fill only closes holes that
        # are surrounded by same-depth surface, so it cannot fill the table's OPEN
        # occlusion shadows, and cranking fill_iters instead bloats other regions
        # (e.g. voxels bleeding into the arm). Density comes from stride instead.
        self.declare_parameter('fill_holes', False)
        self.declare_parameter('fill_iters', 6)
        self.declare_parameter('fill_min_neighbors', 3)
        self.declare_parameter('fill_depth_tol', 0.05)   # m; edge guard

        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.world_frame = self.get_parameter('world_frame').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.obj_id = int(self.get_parameter('object_seg_id').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.target_label = str(self.get_parameter('target_label').value)
        self.target_id = int(self.get_parameter('target_id').value)
        self.static_exclude = bool(self.get_parameter('static_exclude').value)
        self.static_map_file = self.get_parameter('static_map_file').value
        self.static_margin = float(self.get_parameter('static_margin').value)
        self.fill_holes = bool(self.get_parameter('fill_holes').value)
        self.fill_iters = int(self.get_parameter('fill_iters').value)
        self.fill_min_nb = int(self.get_parameter('fill_min_neighbors').value)
        self.fill_tol = float(self.get_parameter('fill_depth_tol').value)
        self._boxes = []          # [(center(3,), half(3,))] in world_frame
        self._boxes_mtime = None
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Freeze the octomap during a pick (see pause_gate): while paused we stop
        # feeding new clouds so move_group plans against a stable scene.
        self.declare_parameter('pause_timeout', 8.0)   # resume 8 s after heartbeat stops
        self.gate = PauseGate(self, float(self.get_parameter('pause_timeout').value))

        self._K = {ns: None for ns in nss}
        self._labels = {ns: {} for ns in nss}
        self._pubs = {}
        self._syncs = []
        for ns in nss:
            self._pubs[ns] = self.create_publisher(
                PointCloud2, f'/{ns}/collision_cloud', 1)
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
        # Runtime target selection: publish a label on /grasp_target to carve that
        # object out of the octomap without a restart; empty string clears it.
        self.create_subscription(String, '/grasp_target',
                                 self._on_grasp_target, 10)
        self.get_logger().info(
            f'collision_cloud up; cameras={nss}, stride={self.stride} '
            '(all objects kept in octomap; grasp target carved out when set)')

    def _on_grasp_target(self, msg):
        label = msg.data.strip()
        if label == self.target_label:
            return
        self.target_label = label
        self.target_id = -1   # label is the runtime interface; clear numeric
        self.get_logger().info(
            f"grasp target -> '{label}' (carved out of octomap)" if label
            else 'grasp target cleared (all objects kept in octomap)')

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
            d[int(key)] = str(val).rsplit('/', 1)[-1]
        if d:
            self._labels[ns] = d

    def _decode(self, msg, dtype):
        a = np.frombuffer(bytes(msg.data), dtype=dtype)
        cols = msg.step // np.dtype(dtype).itemsize
        return a.reshape(msg.height, cols)[:, :msg.width]

    def _on_pair(self, ns, depth_msg, seg_msg):
        if self._K[ns] is None or self.gate.paused():
            return
        fx, fy, cx, cy = self._K[ns]
        try:
            depth = self._decode(depth_msg, np.float32)
            seg = self._decode(seg_msg, np.int32)
        except ValueError:
            return
        if depth.shape != seg.shape:
            return

        st = self.stride
        sub_d = depth[::st, ::st]
        sub_s = seg[::st, ::st]
        hs, ws = sub_d.shape
        uu, vv = np.meshgrid(np.arange(ws) * st, np.arange(hs) * st)
        mask = (np.isfinite(sub_d) & (sub_d > self.min_depth)
                & (sub_d < self.max_depth))
        target_ids = resolve_target_ids(
            self._labels[ns], self.target_label, self.target_id)
        if target_ids:
            # A grasp target is set AND visible here -> carve ONLY it out of the
            # octomap so the gripper can reach + attach it (object_collision boxes
            # it separately). EVERY other object stays in the octomap as a dense
            # obstacle. No target (target_ids None/empty) -> keep ALL objects in
            # the octomap, matching the sensor (dense environment).
            mask &= ~np.isin(sub_s, list(target_ids))
        # Stamp the output with THIS node's clock, NOT the incoming Isaac depth
        # stamp. Isaac stamps depth with sim time (~seconds since start) while the
        # robot TF / move_group run on wall clock (use_sim_time:=False). If we
        # forwarded the sim-time stamp, MoveIt's octomap self-filter would look up
        # the arm TF at a timestamp far in the past, fail, and NOT mask the arm --
        # so the moving arm bakes into the octomap. Wall-now matches the robot TF,
        # so the latest arm pose is used and the arm is filtered out correctly.
        # MoveIt does the optical->world transform (it needs the sensor origin to
        # ray-carve), so we emit the cloud in the optical frame. Guard: skip until
        # the (static) camera TF exists rather than publish an unlocatable cloud.
        if not self.tf_buffer.can_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time()):
            return

        stamp = self.get_clock().now().to_msg()
        # Grow trusted depth (mask) into occlusion holes from its own neighbours,
        # then deproject the trusted + newly-filled pixels together.
        depth_f, use = self._fill_holes(sub_d, mask)
        z = depth_f[use]
        if z.size == 0:
            self._pubs[ns].publish(self._make_cloud(
                np.empty((0, 3), np.float32), ns, stamp))
            return
        xs = uu[use].astype(np.float32)
        ys = vv[use].astype(np.float32)
        pts = deproject(xs, ys, z, fx, fy, cx, cy)   # (N,3) in optical frame
        pts = self._drop_static(ns, pts)             # remove mapped-box surfaces
        self._pubs[ns].publish(self._make_cloud(pts, ns, stamp))

    def _fill_holes(self, depth, keep):
        """Iteratively fill holes in `keep` from neighbouring trusted depths.

        Returns (filled_depth, use_mask). Only fills a hole pixel when it has
        >= fill_min_neighbors trusted neighbours whose depth spread <= fill_tol
        (edge guard: never bridge a real depth step into a phantom surface).
        """
        if not self.fill_holes:
            return depth, keep
        D = np.where(keep, depth.astype(np.float32), np.nan)
        filled = keep.copy()
        h, w = D.shape
        for _ in range(self.fill_iters):
            todo = ~filled
            if not todo.any():
                break
            P = np.pad(D, 1, constant_values=np.nan)
            neigh = np.stack([
                P[0:h, 0:w], P[0:h, 1:w + 1], P[0:h, 2:w + 2],
                P[1:h + 1, 0:w], P[1:h + 1, 2:w + 2],
                P[2:h + 2, 0:w], P[2:h + 2, 1:w + 1], P[2:h + 2, 2:w + 2],
            ], axis=0)
            fin = np.isfinite(neigh)
            cnt = fin.sum(0)
            nmax = np.where(fin, neigh, -np.inf).max(0)
            nmin = np.where(fin, neigh, np.inf).min(0)
            nmean = np.where(fin, neigh, 0.0).sum(0) / np.maximum(cnt, 1)
            can = todo & (cnt >= self.fill_min_nb) & ((nmax - nmin) <= self.fill_tol)
            if not can.any():
                break
            D = np.where(can, nmean, D)
            filled |= can
        return D, filled

    def _static_boxes(self):
        """Lazily (re)load the mapped static boxes; [] if none/disabled."""
        if not self.static_exclude:
            return []
        try:
            m = os.path.getmtime(self.static_map_file)
        except OSError:
            return self._boxes
        if m != self._boxes_mtime:
            d = np.load(self.static_map_file, allow_pickle=True)
            centers = np.asarray(d['centers'], float)
            sizes = np.asarray(d['sizes'], float)
            self._boxes = [(c, s / 2.0 + self.static_margin)
                           for c, s in zip(centers, sizes)]
            self._boxes_mtime = m
            self.get_logger().info(
                f'excluding {len(self._boxes)} mapped static box(es) from octomap '
                f'({[str(x) for x in d["names"]]})')
        return self._boxes

    def _drop_static(self, ns, pts):
        boxes = self._static_boxes()
        if not boxes or pts.shape[0] == 0:
            return pts
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return pts
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        world = pts @ R.T + np.array([t.x, t.y, t.z])  # optical -> world
        keep = np.ones(len(world), bool)
        for center, half in boxes:
            keep &= ~np.all(np.abs(world - center) <= half, axis=1)
        return pts[keep]

    def _make_cloud(self, pts, ns, stamp):
        n = len(pts)
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        if n:
            arr['x'] = pts[:, 0].astype(np.float32)
            arr['y'] = pts[:, 1].astype(np.float32)
            arr['z'] = pts[:, 2].astype(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = ns + self.suffix   # optical frame; MoveIt transforms
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.data = arr.tobytes()
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = CollisionCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
