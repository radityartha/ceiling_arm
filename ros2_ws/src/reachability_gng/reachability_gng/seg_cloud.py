"""Publish a colour-by-segmentation 3D point cloud in `world` for each camera.

RViz Image displays are 2D panels; to see the segmentation IN the 3D view (like
the GNG cloud and the detected-object markers) it must be 3D geometry. This node
deprojects each camera's depth to 3D points, colours every point by its instance
segmentation id (objects vivid, background grey), transforms the points to
`world`, and publishes a PointCloud2 -- add a PointCloud2 display in RViz to
overlay it on the reachability cloud. Colours match seg_colorizer's 2D output.

    /<ns>/seg_cloud   sensor_msgs/PointCloud2  (frame `world`)

    ros2 run reachability_gng seg_cloud            # stride:=3 by default
"""
from __future__ import annotations

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R
from reachability_gng.seg_colorizer import color_for_id

_BG_GREY = (120, 120, 120)


class SegCloud(Node):
    def __init__(self):
        super().__init__('seg_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('stride', 3)   # pixel subsample (1 = full res)

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('optical_frame_suffix').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._K = {ns: None for ns in nss}
        self._pubs = {}
        self._syncs = []
        for ns in nss:
            self._pubs[ns] = self.create_publisher(
                PointCloud2, f'/{ns}/seg_cloud', 1)
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
            f'seg_cloud up; cameras={nss}, stride={self.stride}')

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
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, ns + self.suffix, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        st = self.stride
        sub_d = depth[::st, ::st]
        sub_s = seg[::st, ::st]
        hs, ws = sub_d.shape
        uu, vv = np.meshgrid(np.arange(ws) * st, np.arange(hs) * st)
        valid = (np.isfinite(sub_d) & (sub_d > self.min_depth)
                 & (sub_d < self.max_depth))
        z = sub_d[valid]
        if z.size == 0:
            return
        sid = sub_s[valid]
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        pts = np.stack([X, Y, z], axis=1) @ R.T + T   # (N,3) in world

        rgb = np.tile(np.array(_BG_GREY, np.uint8), (z.size, 1))
        for uid in np.unique(sid):
            if uid > 1:
                rgb[sid == uid] = color_for_id(int(uid))
        rgb_u32 = ((rgb[:, 0].astype(np.uint32) << 16)
                   | (rgb[:, 1].astype(np.uint32) << 8)
                   | rgb[:, 2].astype(np.uint32))

        n = z.size
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'),
                                 ('z', '<f4'), ('rgb', '<f4')])
        arr['x'] = pts[:, 0].astype(np.float32)
        arr['y'] = pts[:, 1].astype(np.float32)
        arr['z'] = pts[:, 2].astype(np.float32)
        arr['rgb'] = rgb_u32.view(np.float32)

        msg = PointCloud2()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = depth_msg.header.stamp
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
        self._pubs[ns].publish(msg)


def main():
    rclpy.init()
    node = SegCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
