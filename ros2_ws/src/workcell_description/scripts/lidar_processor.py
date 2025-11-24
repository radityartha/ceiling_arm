#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import open3d as o3d
import struct


class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        # 1. Subscribe to Raw LiDAR
        self.subscription = self.create_subscription(
            PointCloud2, "/livox/lidar", self.listener_callback, 10
        )

        # 2. Publish Filtered Cloud (For MoveIt Octomap)
        self.pub_filtered = self.create_publisher(
            PointCloud2, "/livox/lidar_filtered", 10
        )

        # 3. Publish Detected Object Pose (For your Picking Script)
        self.pub_object_pose = self.create_publisher(
            PoseStamped, "/detected_object_pose", 10
        )

        self.get_logger().info(
            "Python LiDAR Processor Started: Filtering & Clustering..."
        )

    def listener_callback(self, msg):
        # --- A. Convert ROS -> Open3D ---
        # Read points (x, y, z) from the message
        # We skip 'intensity' here for speed, but you can keep it if needed
        field_names = [field.name for field in msg.fields]
        cloud_data = list(
            pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        )

        if not cloud_data:
            return

        # Convert to Numpy Array
        points = np.array(cloud_data)

        # Create Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # ---------------------------------------------------------
        # STAGE 1: FILTERING (Cleaning the Data)
        # ---------------------------------------------------------

        # 1. CropBox (Remove Floor and Ceiling)
        # Adjust these bounds to match your "World" coordinates
        # Min/Max bounds: [min_x, min_y, min_z], [max_x, max_y, max_z]
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array([-2.0, -2.0, 0.1]),  # Cut floor < 0.1m
            max_bound=np.array([2.0, 2.0, 1.8]),  # Cut ceiling > 1.8m
        )
        pcd = pcd.crop(bbox)

        # 2. Voxel Downsampling (Speed up processing)
        # 0.02 = 2cm voxel size
        pcd = pcd.voxel_down_sample(voxel_size=0.02)

        # 3. Statistical Outlier Removal (Remove "Ghost" noise)
        # nb_neighbors=20, std_ratio=2.0 means:
        # Look at 20 neighbors, remove points further than 2-sigma from mean distance
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # --- PUBLISH FILTERED CLOUD (For MoveIt) ---
        # Convert back to ROS msg
        filtered_points = np.asarray(pcd.points)
        if len(filtered_points) > 0:
            out_msg = self.create_cloud_msg(msg.header, filtered_points)
            self.pub_filtered.publish(out_msg)

        # ---------------------------------------------------------
        # STAGE 2: OBJECT DETECTION (Clustering)
        # ---------------------------------------------------------

        # DBSCAN Clustering
        # eps = distance between points to be in same cluster (0.05 = 5cm)
        # min_points = minimum points to form a cluster (e.g. 10)
        labels = np.array(
            pcd.cluster_dbscan(eps=0.05, min_points=20, print_progress=False)
        )

        if len(labels) == 0:
            return

        max_label = labels.max()
        # self.get_logger().info(f"Point cloud has {max_label + 1} clusters")

        # Iterate through clusters to find the best "Groceries" candidate
        # For simplicity, let's pick the cluster closest to the center of the room
        best_cluster_center = None
        min_dist_to_center = float("inf")

        for i in range(max_label + 1):
            # Extract points for this cluster
            cluster_indices = np.where(labels == i)[0]
            cluster_pcd = pcd.select_by_index(cluster_indices)

            # Calculate Centroid (Center of the object)
            center = cluster_pcd.get_center()  # returns [x, y, z]

            # Filter logic: Ignore clusters that are too big (walls) or too small (noise)
            # You can use cluster_pcd.get_axis_aligned_bounding_box().get_extent() to check size

            # Find closest to (0,0)
            dist = np.linalg.norm(center[:2])  # Distance in X/Y plane
            if dist < min_dist_to_center:
                min_dist_to_center = dist
                best_cluster_center = center

        # --- PUBLISH TARGET POSE ---
        if best_cluster_center is not None:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = best_cluster_center[0]
            pose_msg.pose.position.y = best_cluster_center[1]
            pose_msg.pose.position.z = best_cluster_center[2]

            # Orientation: Point gripper DOWN (standard for picking)
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0

            self.pub_object_pose.publish(pose_msg)
            # self.get_logger().info(f"Object detected at: {best_cluster_center}")

    def create_cloud_msg(self, header, points):
        """Helper to convert Numpy points back to ROS PointCloud2"""
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud_xyz32(header, points)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
