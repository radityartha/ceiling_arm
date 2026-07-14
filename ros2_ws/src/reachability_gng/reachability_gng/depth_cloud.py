"""Publish a segmentation-INDEPENDENT geometric point cloud in `world` per camera.

Same depth deprojection as seg_cloud, but WITHOUT the instance-segmentation sync:
each camera's `/depth` is deprojected to xyz and transformed to `world`, with no
dependency on any detector. seg_cloud gates every frame on a synced segmentation
image (so it stops publishing when YOLOE/isaac produces nothing, e.g. YOLOE on
synthetic sim imagery); the environment topo map only needs GEOMETRY (env_gng
reads xyz only), so pointing env_gng / map_topo_static here decouples the map from
`seg_source` entirely -- the real-world path with RGBD cameras only.

    /<ns>/depth_cloud   sensor_msgs/PointCloud2  (frame `world`, xyz only)

    ros2 run reachability_gng depth_cloud            # stride:=3 by default
    # then: env_gng / map_topo_static with cloud_topic_suffix:=depth_cloud
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


class DepthCloud(Node):
    def __init__(self):
        super().__init__('depth_cloud')
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
        for ns in nss:
            self._pubs[ns] = self.create_publisher(
                PointCloud2, f'/{ns}/depth_cloud', qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, ns=ns: self._on_info(ns, m), 1)
            # RELIABLE (default) for the depth Image: the Isaac depth publisher
            # is RELIABLE and a BEST_EFFORT subscriber receives NOTHING from it
            # here (verified) -- seg_cloud reads depth over a RELIABLE
            # message_filters sub and works, so match that. (The BEST_EFFORT rule
            # applies to the large OUTPUT PointCloud env_gng consumes, not this
            # depth-image INPUT.)
            self.create_subscription(
                Image, f'/{ns}/depth',
                lambda m, ns=ns: self._on_depth(ns, m), 10)
        self.get_logger().info(
            f'depth_cloud up; cameras={nss}, stride={self.stride}')

    def _on_info(self, ns, m):
        k = m.k
        self._K[ns] = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _decode(self, msg, dtype):
        a = np.frombuffer(bytes(msg.data), dtype=dtype)
        cols = msg.step // np.dtype(dtype).itemsize
        return a.reshape(msg.height, cols)[:, :msg.width]

    def _on_depth(self, ns, depth_msg):
        if self._K[ns] is None:
            return
        fx, fy, cx, cy = self._K[ns]
        try:
            depth = self._decode(depth_msg, np.float32)
        except ValueError:
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
        hs, ws = sub_d.shape
        uu, vv = np.meshgrid(np.arange(ws) * st, np.arange(hs) * st)
        valid = (np.isfinite(sub_d) & (sub_d > self.min_depth)
                 & (sub_d < self.max_depth))
        z = sub_d[valid]
        if z.size == 0:
            return
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        pts = (np.stack([X, Y, z], axis=1) @ R.T + T).astype(np.float32)

        n = z.size
        msg = PointCloud2()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = depth_msg.header.stamp
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
        msg.data = pts.tobytes()
        msg.is_dense = True
        self._pubs[ns].publish(msg)


def main():
    rclpy.init()
    node = DepthCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
