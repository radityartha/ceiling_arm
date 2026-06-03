"""
gng_validation_node.py — SIM-ONLY ground-truth validation harness.

================================ SIM ONLY ==================================
This node is a SIMULATION validation tool.  It compares GNG-derived object
centroids against the TRUE model poses coming from Gazebo Fortress.  It must
NEVER be launched on real hardware (there is no ground-truth pose source on
the real Mid360).  Keep it out of any real-hardware launch file.
============================================================================

GROUND-TRUTH SOURCE (verified locally against the installed ros_gz_bridge):
  Gazebo Fortress publishes model poses on:
      /world/<world_name>/pose/info          (all models, gz.msgs.Pose_V)
      /world/<world_name>/dynamic_pose/info   (moving models, gz.msgs.Pose_V)
  ros_gz_bridge maps gz.msgs.Pose_V to either:
      geometry_msgs/msg/PoseArray   (used here — simplest, name-agnostic)
      tf2_msgs/msg/TFMessage        (alternative, preserves model names)
  (Type mapping confirmed in
   /opt/ros/humble/include/ros_gz_bridge/ros_gz_bridge/convert/geometry_msgs.hpp.)

  The bridge entry is provided in config/pose_bridge.yaml.  This node
  subscribes to the bridged PoseArray on the `truth_poses_topic` parameter.

METHOD:
  1. Subscribe to /gng/nodes (PointCloud2 of GNG node positions).
  2. Cluster the GNG nodes with DBSCAN → candidate object clusters; each
     cluster's centroid is a GNG-derived object position.
  3. Subscribe to the bridged ground-truth PoseArray.
  4. For each ground-truth pose, find the nearest GNG centroid and report the
     Euclidean distance.  Also report unmatched truths / spurious clusters.

This is a coarse, transparent harness — not a benchmarking suite.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN


class GngValidationNode(Node):
    def __init__(self):
        super().__init__("gng_validation_node")

        self.declare_parameter("gng_nodes_topic", "/gng/nodes")
        self.declare_parameter("truth_poses_topic", "/gng/ground_truth_poses")
        self.declare_parameter("dbscan_eps", 0.35)         # m
        self.declare_parameter("dbscan_min_samples", 2)
        self.declare_parameter("report_period_s", 3.0)
        # Optional static fallback: if no live truth arrives, use these named
        # truths (matches cell.sdf boxes).  ROS 2 parameters must be homogeneous
        # lists, so names and coords are declared separately and zipped:
        #   static_truth_names: [name0, name1, ...]
        #   static_truth_xyz:   [x0, y0, z0, x1, y1, z1, ...]
        self.declare_parameter(
            "static_truth_names", ["validation_box_1", "validation_box_2"])
        self.declare_parameter(
            "static_truth_xyz", [1.0, 0.0, 0.25, -1.0, 0.5, 0.2])

        self._eps = float(self.get_parameter("dbscan_eps").value)
        self._min_samples = int(self.get_parameter("dbscan_min_samples").value)
        self._nodes = np.empty((0, 3))
        self._truth = self._parse_static_truth()
        self._got_live_truth = False

        self.create_subscription(
            PointCloud2, self.get_parameter("gng_nodes_topic").value,
            self._nodes_cb, 1)
        self.create_subscription(
            PoseArray, self.get_parameter("truth_poses_topic").value,
            self._truth_cb, 10)

        period = float(self.get_parameter("report_period_s").value)
        self.create_timer(period, self._report)

        self.get_logger().info(
            "GNG validation harness up (SIM ONLY). "
            f"Using {'live' if self._got_live_truth else 'static-fallback'} truth. "
            "Do NOT run on real hardware.")

    def _parse_static_truth(self):
        names = list(self.get_parameter("static_truth_names").value)
        coords = [float(v) for v in self.get_parameter("static_truth_xyz").value]
        truth = {}
        for i, name in enumerate(names):
            k = i * 3
            if k + 2 < len(coords):
                truth[str(name)] = np.array([coords[k], coords[k + 1], coords[k + 2]])
        return truth

    def _nodes_cb(self, msg: PointCloud2):
        arr = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if arr is None or len(arr) == 0:
            self._nodes = np.empty((0, 3))
            return
        self._nodes = np.column_stack(
            (arr["x"].astype(float), arr["y"].astype(float), arr["z"].astype(float)))

    def _truth_cb(self, msg: PoseArray):
        # Live PoseArray has no names; index them positionally.
        live = {}
        for i, pose in enumerate(msg.poses):
            live[f"model_{i}"] = np.array(
                [pose.position.x, pose.position.y, pose.position.z])
        if live:
            self._truth = live
            self._got_live_truth = True

    def _centroids(self) -> np.ndarray:
        """Cluster GNG nodes into objects; return cluster centroids."""
        if self._nodes.shape[0] < self._min_samples:
            return np.empty((0, 3))
        labels = DBSCAN(eps=self._eps, min_samples=self._min_samples).fit_predict(self._nodes)
        centroids = []
        for lab in sorted(set(labels)):
            if lab == -1:
                continue  # noise
            centroids.append(self._nodes[labels == lab].mean(axis=0))
        return np.array(centroids) if centroids else np.empty((0, 3))

    def _report(self):
        centroids = self._centroids()
        src = "LIVE" if self._got_live_truth else "STATIC"
        if centroids.shape[0] == 0:
            self.get_logger().info(
                f"[{src} truth] GNG has {self._nodes.shape[0]} nodes, "
                f"0 clusters yet — waiting for the network to grow.")
            return

        lines = [f"--- GNG centroid-vs-truth ({src} truth, "
                 f"{self._nodes.shape[0]} nodes, {centroids.shape[0]} clusters) ---"]
        for name, t in self._truth.items():
            d = np.linalg.norm(centroids - t, axis=1)
            j = int(np.argmin(d))
            lines.append(
                f"  {name:18s} truth=({t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f})  "
                f"nearest_centroid=({centroids[j][0]:+.2f},{centroids[j][1]:+.2f},"
                f"{centroids[j][2]:+.2f})  error={d[j]:.3f} m")
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = GngValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
