"""Publish the ENVIRONMENT depth reading (per camera) for MoveIt's octomap.

For collision avoidance MoveIt should see the static environment -- the work
table, fixtures, walls, anything unmodelled -- but NOT the graspable objects:
those are represented separately as exact CollisionObject boxes by
object_collision.py (and attached to the gripper at grasp time). If the objects
were also voxelized into the octomap, leftover object voxels would block the
gripper even after the object is attached. So this node deprojects every valid
depth pixel EXCEPT the segmented objects (instance-seg id > 1) and publishes them
on `/<ns>/collision_cloud` in the `world` frame (transformed here via TF).

MoveIt's PointCloudOctomapUpdater (workcell_moveit_config/config/sensors_3d.yaml)
voxelizes both cameras' clouds, self-filters the robot's own links, and the
planners avoid the resulting environment voxels. NOTE: we publish already in the
octomap map frame (`world`) so the updater's map_frame==cloud_frame fast path is
taken -- the camera-optical-frame + message-filter path was silently dropping
every cloud, leaving the octomap empty. Tradeoff: the updater can no longer infer
the sensor origin for free-space ray-carving, so stale voxels are cleared by the
octomap_refresher node instead.

    /<ns>/collision_cloud   sensor_msgs/PointCloud2   (frame `world`)

    ros2 run reachability_gng collision_cloud
"""
from __future__ import annotations

import json

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


class CollisionCloud(Node):
    def __init__(self):
        super().__init__('collision_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        # EXPERIMENT: publish in `world` (transform points here via TF) instead of
        # the camera optical frame. map_frame==cloud_frame in MoveIt's octomap
        # updater -> no message-filter/sensor-transform path (which was silently
        # dropping every cloud, leaving the octomap empty). Tradeoff: sensor origin
        # for ray-carving is lost, so stale voxels rely on octomap_refresher.
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

        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.world_frame = self.get_parameter('world_frame').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.obj_id = int(self.get_parameter('object_seg_id').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.target_label = str(self.get_parameter('target_label').value)
        self.target_id = int(self.get_parameter('target_id').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.get_logger().info(
            f'collision_cloud up; cameras={nss}, stride={self.stride} '
            f'(objects seg>{self.obj_id} excluded -> handled by object_collision)')

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
        if self._K[ns] is None:
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
        if target_ids is None:
            mask &= (sub_s <= self.obj_id)             # legacy: drop every object
        elif target_ids:
            mask &= ~np.isin(sub_s, list(target_ids))  # drop ONLY the grasp target
        # target configured but unseen in THIS camera -> nothing to exclude here
        # Stamp the output with THIS node's clock, NOT the incoming Isaac depth
        # stamp. Isaac stamps depth with sim time (~seconds since start) while the
        # robot TF / move_group run on wall clock (use_sim_time:=False). If we
        # forwarded the sim-time stamp, MoveIt's octomap self-filter would look up
        # the arm TF at a timestamp far in the past, fail, and NOT mask the arm --
        # so the moving arm bakes into the octomap. Wall-now matches the robot TF,
        # so the latest arm pose is used and the arm is filtered out correctly.
        # world <- camera optical (latest available, like object_localizer); if TF
        # isn't ready yet, skip this frame rather than publish in the wrong frame.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        rot = tf.transform.rotation
        tr = tf.transform.translation
        R = quat_to_R(rot.x, rot.y, rot.z, rot.w)
        T = np.array([tr.x, tr.y, tr.z])

        stamp = self.get_clock().now().to_msg()
        z = sub_d[mask]
        if z.size == 0:
            self._pubs[ns].publish(self._make_cloud(
                np.empty((0, 3), np.float32), ns, stamp))
            return
        xs = uu[mask].astype(np.float32)
        ys = vv[mask].astype(np.float32)
        pts = deproject(xs, ys, z, fx, fy, cx, cy)   # (N,3) in optical frame
        pts = (R @ pts.T).T + T                       # -> world frame
        self._pubs[ns].publish(self._make_cloud(pts, ns, stamp))

    def _make_cloud(self, pts, ns, stamp):
        n = len(pts)
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        if n:
            arr['x'] = pts[:, 0].astype(np.float32)
            arr['y'] = pts[:, 1].astype(np.float32)
            arr['z'] = pts[:, 2].astype(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = self.world_frame   # points already in world frame
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
