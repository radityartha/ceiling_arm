#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os
from datetime import datetime


class PcdSaver(Node):
    def __init__(self):
        super().__init__("pcd_saver")

        # Listen to the FILTERED topic (clean data)
        self.subscription = self.create_subscription(
            PointCloud2, "/livox/lidar", self.listener_callback, 10
        )

        self.latest_cloud = None
        self.save_folder = os.path.expanduser("~/lidar_dataset")

        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        self.get_logger().info(f"Ready to save to {self.save_folder}")
        self.get_logger().info("Press 'Enter' in the terminal to save a snapshot!")

    def listener_callback(self, msg):
        self.latest_cloud = msg

    def save_snapshot(self):
        if self.latest_cloud is None:
            self.get_logger().warn("No data received yet...")
            return

        # Convert ROS -> Numpy
        gen = pc2.read_points(
            self.latest_cloud, skip_nans=True, field_names=("x", "y", "z")
        )
        xyz = [[p[0], p[1], p[2]] for p in gen]

        if not xyz:
            return

        # Create Open3D Cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(xyz, dtype=np.float64))

        # Save to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.save_folder}/cloud_{timestamp}.pcd"
        o3d.io.write_point_cloud(filename, pcd)

        self.get_logger().info(f"Saved: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = PcdSaver()

    # Run the node in a non-blocking way so we can check for keyboard input
    import threading

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        while rclpy.ok():
            input()  # Wait for Enter key
            node.save_snapshot()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
