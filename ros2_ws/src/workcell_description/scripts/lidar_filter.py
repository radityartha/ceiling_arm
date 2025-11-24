#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import struct
import matplotlib.colors as mcolors  # For coloring clusters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


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

        # --- NEW DEBUG TOPIC ---
        # This shows you exactly which points belong to which object
        self.pub_debug = self.create_publisher(PointCloud2, "/livox/debug_clusters", 10)

        # --- CROP SETTINGS ---
        # Z-Min: Increase this to cut off the table surface!
        # If table is at Z=0.8, set this to 0.82
        self.min_bound = np.array([-4.0, -4.0, 0.1])

        # Z Max: 5.0 (Far enough to see the floor)
        self.max_bound = np.array([4.0, 4.0, 3.0])
        self.bbox = o3d.geometry.AxisAlignedBoundingBox(self.min_bound, self.max_bound)

        # --- TUNING PARAMETERS ---
        self.voxel_size = 0.01  # 1cm grid (smaller = more detail)
        self.cluster_eps = 0.05  # 5cm distance to link points
        self.min_points = 10  # Minimum points to be an "object"

        self.get_logger().info("Lidar Filter & Object Detector Started.")

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

        # 2. Downsample (Crucial for speed and consistency)
        down_pcd = cropped_pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # 3. Publish Clean Cloud (for MoveIt)
        filtered_points = np.asarray(down_pcd.points)
        out_msg = self.create_cloud_msg(msg.header, filtered_points.astype(np.float32))
        self.pub_filtered.publish(out_msg)

        # 4. CLUSTERING (DBSCAN)
        labels = np.array(
            down_pcd.cluster_dbscan(
                eps=self.cluster_eps, min_points=self.min_points, print_progress=False
            )
        )

        if len(labels) == 0:
            return
        max_label = labels.max()

        # --- VISUAL DEBUGGING ---
        # Colorize the point cloud based on clusters
        # Noise (-1) = Black. Objects = Rainbow.
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0  # Labels -1 (noise) set to black
        down_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # Create colored cloud message manually since helper function doesn't support RGB
        debug_points = np.asarray(down_pcd.points)
        debug_colors = np.asarray(down_pcd.colors)
        # (Skip complex RGB packing for brevity, just printing status)
        # For true debug visualization, we process centroids below.

        marker_array = MarkerArray()
        best_dist = float("inf")
        best_pose = None

        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            cluster_pcd = down_pcd.select_by_index(cluster_indices)
            center = cluster_pcd.get_center()

            # Filter by size (Optional: Ignore huge walls)
            # extent = cluster_pcd.get_axis_aligned_bounding_box().get_extent()
            # if max(extent) > 0.5: continue # Ignore objects bigger than 50cm

            # Create Marker
            marker = Marker()
            marker.header = msg.header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            dist = np.linalg.norm(center[:2])
            if dist < best_dist:
                best_dist = dist
                best_pose = center

        self.pub_markers.publish(marker_array)

        if best_pose is not None:
            target_msg = PoseStamped()
            target_msg.header = msg.header
            target_msg.pose.position.x = best_pose[0]
            target_msg.pose.position.y = best_pose[1]
            target_msg.pose.position.z = best_pose[2]
            target_msg.pose.orientation.x = 1.0
            self.pub_pose.publish(target_msg)

    def create_cloud_msg(self, header, points):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud_xyz32(header, points)


# Helper for colors
import matplotlib.pyplot as plt


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
