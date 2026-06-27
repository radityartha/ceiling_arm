"""Per-object reachability of the segmented objects against the GNG map(s),
following each object's ACTUAL SHAPE (bottle, bag, anything) -- in two modes:

  * VOXEL (default, voxel_size > 0): voxelise each object's segmented points into
    a uniform grid, fuse voxels across cameras, classify each voxel blue
    (reachable: dist to nearest GNG node <= reach_radius) or red, and publish a
    CUBE_LIST + a per-object "% reachable by volume" label. Uniform voxels make
    the percentage a fair volumetric metric (unlike raw points, whose density is
    biased by camera distance/viewing angle).

  * POINT CLOUD (voxel_size <= 0): the raw deprojected points coloured blue/red
    per point, published per camera as a PointCloud2 (fine detail).

Outputs:
    /reachability/voxels        visualization_msgs/MarkerArray  (voxel mode)
    /<ns>/reachability_cloud    sensor_msgs/PointCloud2         (point mode)

    ros2 run reachability_gng reachability_cloud                 # voxel (default)
    ros2 run reachability_gng reachability_cloud -p voxel_size:=0.0   # raw points
"""
from __future__ import annotations

import json
import time

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.object_localizer import quat_to_R
from reachability_gng.reachability_check import ArmMap

# blue (not green) for reachable so it isn't confused with the green
# obstacle/octomap voxels in the same scene; red = unreachable.
_REACH = (30, 150, 255)
_RED = (220, 30, 30)


class ReachabilityCloud(Node):
    def __init__(self):
        super().__init__('reachability_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('stride', 1)
        # reach_radius <= 0 -> density-adaptive (reach_radius_factor * node
        # spacing), independent of the GNG `lam`; > 0 -> absolute radius (legacy).
        self.declare_parameter('reach_radius', 0.0)
        self.declare_parameter('reach_radius_factor', 1.0)
        # enclosure gate: 0..1, lower = stricter (rejects boundary bleed); >=1 off
        self.declare_parameter('enclose_thresh', 0.5)
        self.declare_parameter('enclose_k', 8)
        self.declare_parameter('voxel_size', 0.015)   # >0 voxel mode, <=0 raw pts
        self.declare_parameter('voxel_ttl', 1.0)      # s a voxel persists unseen
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('arm_models',
                               ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz'])
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        radius_abs = float(self.get_parameter('reach_radius').value)
        radius_factor = float(self.get_parameter('reach_radius_factor').value)
        enclose_thresh = float(self.get_parameter('enclose_thresh').value)
        enclose_k = int(self.get_parameter('enclose_k').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.voxel_ttl = float(self.get_parameter('voxel_ttl').value)
        self.voxel_mode = self.voxel_size > 0.0
        nss = list(self.get_parameter('camera_namespaces').value)
        models = list(self.get_parameter('arm_models').value)
        names = list(self.get_parameter('arm_names').value)

        self.arms = []
        for nm, mp in zip(names, models):
            try:
                arm = ArmMap(nm, mp, radius_factor=radius_factor,
                             radius_abs=radius_abs, enclose_thresh=enclose_thresh,
                             enclose_k=enclose_k)
            except OSError:
                self.get_logger().error(f'could not load {nm} model {mp}')
                continue
            self.arms.append(arm)
            self.get_logger().info(
                f'loaded {nm}: {len(arm.W3)} nodes, spacing={arm.spacing:.3f} m, '
                f'reach_radius={arm.reach_radius:.3f} m')
        if not self.arms:
            self.get_logger().error('no GNG maps loaded; nothing to classify')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._K = {ns: None for ns in nss}
        self._labels = {ns: {} for ns in nss}
        self._syncs = []
        # voxel mode: label -> {voxel_idx (ix,iy,iz): last_seen_monotonic}, fused
        # across cameras + time (a voxel persists voxel_ttl s after last seen so
        # jitter / async cameras / momentary dropouts don't flicker it). Point
        # mode: per-camera PointCloud2 publishers.
        self._vox_seen = {}
        self._pc_pubs = {}

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
            if not self.voxel_mode:
                self._pc_pubs[ns] = self.create_publisher(
                    PointCloud2, f'/{ns}/reachability_cloud', 1)

        if self.voxel_mode:
            self.vox_pub = self.create_publisher(
                MarkerArray, '/reachability/voxels', 1)
            self.create_timer(
                float(self.get_parameter('publish_period').value),
                self._publish_voxels)
        rmode = 'absolute' if radius_abs > 0.0 else f'adaptive (x{radius_factor})'
        self.get_logger().info(
            f'reachability_cloud up; mode={"voxel" if self.voxel_mode else "points"}'
            f', voxel_size={self.voxel_size}, reach_radius={rmode}')

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

    def _object_points(self, ns, depth_msg, seg_msg):
        """Deproject object pixels (seg id>1) to world; return (pts Nx3, ids N)."""
        if self._K[ns] is None:
            return None
        fx, fy, cx, cy = self._K[ns]
        try:
            depth = self._decode(depth_msg, np.float32)
            seg = self._decode(seg_msg, np.int32)
        except ValueError:
            return None
        if depth.shape != seg.shape:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        st = self.stride
        sub_d = depth[::st, ::st]
        sub_s = seg[::st, ::st]
        hs, ws = sub_d.shape
        uu, vv = np.meshgrid(np.arange(ws) * st, np.arange(hs) * st)
        mask = (sub_s > 1) & np.isfinite(sub_d) \
            & (sub_d > self.min_depth) & (sub_d < self.max_depth)
        z = sub_d[mask]
        if z.size == 0:
            return np.empty((0, 3)), np.empty((0,), dtype=np.int32)
        u = uu[mask].astype(np.float32)
        v = vv[mask].astype(np.float32)
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        pts = np.stack([X, Y, z], axis=1) @ R.T + T
        return pts, sub_s[mask]

    def _reach_mask(self, pts):
        """Boolean reachable mask: a point is reachable if some arm's map both
        has a node within its (density-adaptive) radius AND encloses the point."""
        reach = np.zeros(len(pts), dtype=bool)
        for arm in self.arms:
            reach |= arm.reach_mask(pts)
        return reach

    def _on_pair(self, ns, depth_msg, seg_msg):
        if not self.arms:
            return
        res = self._object_points(ns, depth_msg, seg_msg)
        if res is None:
            return
        pts, ids = res
        if self.voxel_mode:
            if len(pts):
                now = time.monotonic()
                idx = np.floor(pts / self.voxel_size).astype(np.int64)
                for uid in np.unique(ids):
                    label = self._labels[ns].get(int(uid), f'id{int(uid)}')
                    seen = self._vox_seen.setdefault(label, {})
                    for cell in map(tuple, idx[ids == uid].tolist()):
                        seen[cell] = now   # refresh last-seen (fuses both cams)
            return
        # point-cloud mode: colour each point + publish per-camera PointCloud2
        if len(pts) == 0:
            return
        reach = self._reach_mask(pts)
        rgb = np.where(reach[:, None], np.array(_REACH, np.uint8),
                       np.array(_RED, np.uint8))
        self._pc_pubs[ns].publish(
            self._make_cloud(pts, rgb, depth_msg.header.stamp))

    # ---- voxel output -------------------------------------------------------
    def _publish_voxels(self):
        now = time.monotonic()

        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        cubes = Marker()
        cubes.header.frame_id = self.world_frame
        cubes.ns = 'reach_voxels'
        cubes.id = 0
        cubes.type = Marker.CUBE_LIST
        cubes.action = Marker.ADD
        cubes.scale.x = cubes.scale.y = cubes.scale.z = self.voxel_size
        cubes.pose.orientation.w = 1.0

        report = []
        for i, label in enumerate(sorted(self._vox_seen)):
            seen = self._vox_seen[label]
            # drop voxels not refreshed within the TTL (debounces jitter/dropouts)
            for cell in [c for c, t in seen.items() if now - t > self.voxel_ttl]:
                del seen[cell]
            if not seen:
                continue
            idx = np.array(sorted(seen), dtype=np.int64)
            centers = (idx + 0.5) * self.voxel_size
            reach = self._reach_mask(centers)
            frac = float(reach.mean())
            for c, r in zip(centers, reach):
                cubes.points.append(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])))
                g = _REACH if r else _RED
                cubes.colors.append(ColorRGBA(r=g[0] / 255, g=g[1] / 255,
                                              b=g[2] / 255, a=0.9))
            top = centers[np.argmax(centers[:, 2])]
            txt = Marker()
            txt.header.frame_id = self.world_frame
            txt.ns = 'reach_voxel_label'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(x=float(top[0]), y=float(top[1]),
                                      z=float(top[2]) + 0.06)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.04
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f'{label} {frac * 100:.0f}% reach'
            ma.markers.append(txt)
            report.append(f'{label}:{frac * 100:.0f}%({int(reach.sum())}/{len(reach)})')

        ma.markers.append(cubes)
        self.vox_pub.publish(ma)
        if report:
            self.get_logger().info('voxel reach -> ' + ' '.join(report))

    # ---- point-cloud packing ------------------------------------------------
    def _make_cloud(self, pts, rgb, stamp):
        rgb_u32 = ((rgb[:, 0].astype(np.uint32) << 16)
                   | (rgb[:, 1].astype(np.uint32) << 8)
                   | rgb[:, 2].astype(np.uint32))
        n = len(pts)
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'),
                                 ('z', '<f4'), ('rgb', '<f4')])
        arr['x'] = pts[:, 0].astype(np.float32)
        arr['y'] = pts[:, 1].astype(np.float32)
        arr['z'] = pts[:, 2].astype(np.float32)
        arr['rgb'] = rgb_u32.view(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = arr.tobytes()
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = ReachabilityCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
