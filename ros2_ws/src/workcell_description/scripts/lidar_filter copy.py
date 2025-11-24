#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter_py")

        # --- CONFIGURATION ---
        # 1. Input Topic (Must match the Launch file remapping)
        self.input_topic = "/livox/points"

        # 2. QoS Profile (Required to hear the Driver)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.listener_callback, qos_profile
        )

        self.publisher = self.create_publisher(PointCloud2, "/livox/filtered", 1)

        # --- CROP BOX SETTINGS ---

        # X and Y: 3.0m (Wide enough to see the whole room)
        # Z Min: 0.3 (Start 30cm AWAY from the sensor face to hide the rail)
        self.min_bound = np.array([-4.0, -4.0, 0.1])

        # Z Max: 5.0 (Far enough to see the floor)
        self.max_bound = np.array([4.0, 4.0, 3.0])

        self.bbox = o3d.geometry.AxisAlignedBoundingBox(self.min_bound, self.max_bound)

        self.get_logger().info(
            f"Lidar Filter Started. Listening to {self.input_topic}..."
        )

    def listener_callback(self, msg):
        # --- Fix for 'numpy.void' error ---
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        xyz = [[p[0], p[1], p[2]] for p in gen]

        if not xyz:
            return

        points = np.array(xyz, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # --- Filter ---
        cropped_pcd = pcd.crop(self.bbox)

        # (Optional) Downsample for speed
        # cropped_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.02)

        filtered_points = np.asarray(cropped_pcd.points)

        if len(filtered_points) == 0:
            return

        # --- Publish ---
        out_msg = self.create_cloud_msg(msg.header, filtered_points.astype(np.float32))
        self.publisher.publish(out_msg)

    def create_cloud_msg(self, header, points):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud_xyz32(header, points)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
