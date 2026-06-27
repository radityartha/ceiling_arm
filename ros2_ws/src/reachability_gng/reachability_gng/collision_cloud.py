"""Publish the ENVIRONMENT depth reading (per camera) for MoveIt's octomap.

For collision avoidance MoveIt should see the static environment -- the work
table, fixtures, walls, anything unmodelled -- but NOT the graspable objects:
those are represented separately as exact CollisionObject boxes by
object_collision.py (and attached to the gripper at grasp time). If the objects
were also voxelized into the octomap, leftover object voxels would block the
gripper even after the object is attached. So this node deprojects every valid
depth pixel EXCEPT the segmented objects (instance-seg id > 1) and publishes them
on `/<ns>/collision_cloud` in the camera optical frame.

MoveIt's PointCloudOctomapUpdater (workcell_moveit_config/config/sensors_3d.yaml)
voxelizes both cameras' clouds, self-filters the robot's own links, and the
planners avoid the resulting environment voxels. Publishing in the camera optical
frame lets the updater carve free space correctly from the camera viewpoint.

    /<ns>/collision_cloud   sensor_msgs/PointCloud2   (frame `<ns>_camera_optical`)

    ros2 run reachability_gng collision_cloud
"""
from __future__ import annotations

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from reachability_gng.object_localizer import deproject


class CollisionCloud(Node):
    def __init__(self):
        super().__init__('collision_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('object_seg_id', 1)   # drop pixels with seg id > this
        self.declare_parameter('stride', 3)           # pixel subsample (1 = full res)

        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.obj_id = int(self.get_parameter('object_seg_id').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        nss = list(self.get_parameter('camera_namespaces').value)

        self._K = {ns: None for ns in nss}
        self._pubs = {}
        self._syncs = []
        for ns in nss:
            self._pubs[ns] = self.create_publisher(
                PointCloud2, f'/{ns}/collision_cloud', 1)
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, ns=ns: self._on_info(ns, m), 1)
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
                & (sub_d < self.max_depth) & (sub_s <= self.obj_id))
        # Stamp the output with THIS node's clock, NOT the incoming Isaac depth
        # stamp. Isaac stamps depth with sim time (~seconds since start) while the
        # robot TF / move_group run on wall clock (use_sim_time:=False). If we
        # forwarded the sim-time stamp, MoveIt's octomap self-filter would look up
        # the arm TF at a timestamp far in the past, fail, and NOT mask the arm --
        # so the moving arm bakes into the octomap. Wall-now matches the robot TF,
        # so the latest arm pose is used and the arm is filtered out correctly.
        stamp = self.get_clock().now().to_msg()
        z = sub_d[mask]
        if z.size == 0:
            self._pubs[ns].publish(self._make_cloud(
                np.empty((0, 3), np.float32), ns, stamp))
            return
        xs = uu[mask].astype(np.float32)
        ys = vv[mask].astype(np.float32)
        pts = deproject(xs, ys, z, fx, fy, cx, cy)  # (N,3) in optical frame
        self._pubs[ns].publish(self._make_cloud(pts, ns, stamp))

    def _make_cloud(self, pts, ns, stamp):
        n = len(pts)
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        if n:
            arr['x'] = pts[:, 0].astype(np.float32)
            arr['y'] = pts[:, 1].astype(np.float32)
            arr['z'] = pts[:, 2].astype(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = ns + self.suffix
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
