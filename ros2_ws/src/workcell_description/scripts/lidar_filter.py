#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import matplotlib.pyplot as plt


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter_py")

        self.input_topic = "/livox/points"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.listener_callback, qos_profile
        )

        self.pub_filtered = self.create_publisher(PointCloud2, "/livox/filtered", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/detected_object_markers", 10
        )

        # --- CROP SETTINGS ---
        # Keep floor visible for clearing ghosts (-0.5)
        # Cut ceiling (1.75)
        self.min_bound = np.array([-4.0, -4.0, 0.1])

        # Z Max: 5.0 (Far enough to see the floor)
        self.max_bound = np.array([4.0, 4.0, 3.0])

        self.bbox = o3d.geometry.AxisAlignedBoundingBox(self.min_bound, self.max_bound)

        # --- TARGET OBJECT SETTINGS (Paper Bag) ---
        # Size: ~32cm x 39cm. We allow some tolerance.
        # We check the "Diagonal" or Max Dimension to be safe.
        self.min_size = 0.15  # Min 15cm (Ignore tiny noise)
        self.max_size = 0.60  # Max 60cm (Ignore walls/tables)

        # --- CLUSTERING SETTINGS ---
        self.voxel_size = 0.02  # 1cm detail
        self.cluster_eps = 0.06  # 6cm gap allowed between points
        self.min_points = 15  # Needs 30 points to be a real object

        self.get_logger().info("Lidar Filter & Bag Detector Started.")

    def listener_callback(self, msg):
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        xyz = [[p[0], p[1], p[2]] for p in gen]
        if not xyz:
            return

        points = np.array(xyz, dtype=np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 1. Crop
        cropped_pcd = pcd.crop(self.bbox)
        if len(cropped_pcd.points) == 0:
            return

        # 2. Downsample
        down_pcd = cropped_pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # 3. Publish Clean Cloud (for MoveIt)
        filtered_points = np.asarray(down_pcd.points)
        out_msg = self.create_cloud_msg(msg.header, filtered_points.astype(np.float32))
        self.pub_filtered.publish(out_msg)

        # 4. CLUSTERING
        labels = np.array(
            down_pcd.cluster_dbscan(
                eps=self.cluster_eps, min_points=self.min_points, print_progress=False
            )
        )

        if len(labels) == 0:
            return
        max_label = labels.max()

        marker_array = MarkerArray()
        best_dist = float("inf")
        best_pose = None

        # Loop through every detected cluster
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            cluster_pcd = down_pcd.select_by_index(cluster_indices)

            # --- SIZE FILTERING (The Logic) ---
            # Get the bounding box of this specific cluster
            aabb = cluster_pcd.get_axis_aligned_bounding_box()
            extent = aabb.get_extent()  # Returns [length, width, height]

            # Check dimensions:
            # Is it too small? (e.g., noise)
            if max(extent) < self.min_size:
                continue

            # Is it too big? (e.g., wall or table)
            if max(extent) > self.max_size:
                continue

            # Calculate Center
            center = cluster_pcd.get_center()

            # Create Visual Marker (Green Box for valid objects)
            marker = Marker()
            marker.header = msg.header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]

            # Orientation (Aligned with world)
            marker.pose.orientation.w = 1.0

            # Scale to match the actual object size
            marker.scale.x = extent[0]
            marker.scale.y = extent[1]
            marker.scale.z = extent[2]

            # Color: Green = Valid Bag
            marker.color.a = 0.6
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

            # Find the object closest to the center of the room (0,0)
            dist = np.linalg.norm(center[:2])
            if dist < best_dist:
                best_dist = dist
                best_pose = center

        # Publish Markers
        self.pub_markers.publish(marker_array)

        # Publish Pose
        if best_pose is not None:
            target_msg = PoseStamped()
            target_msg.header = msg.header
            target_msg.pose.position.x = best_pose[0]
            target_msg.pose.position.y = best_pose[1]
            target_msg.pose.position.z = best_pose[2]
            target_msg.pose.orientation.x = 1.0  # Point Down
            self.pub_pose.publish(target_msg)

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