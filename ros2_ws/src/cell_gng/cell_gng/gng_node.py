"""
gng_node.py — ROS 2 wrapper around the GrowingNeuralGas core.

INPUT CONTRACT (defined FIRST, before any processing):
  Topic:  configurable via the `input_cloud_topic` parameter.
          Default: /livox/points  (the sim LiDAR; see cell_gazebo_sim).
          To run on the REAL Mid360 later, set this parameter to the real
          driver's cloud topic (e.g. /livox/points from livox_ros_driver2).
          NO CODE CHANGE is required — only the parameter.
  Type:   sensor_msgs/msg/PointCloud2
  Fields REQUIRED:  x, y, z   (float32)   — Cartesian point coordinates, metres.
  Fields OPTIONAL:  rgb / intensity       — read but currently ignored by the
                    GNG (the network operates on 3-D geometry only).  They are
                    documented here so a future colour/intensity-aware variant
                    knows they may be present.
  Frame:  whatever the cloud's header.frame_id is.  Output markers are
          published in that same frame so RViz overlays them on the cloud.

OUTPUTS:
  /gng/graph  visualization_msgs/MarkerArray
              - marker id 0: SPHERE_LIST of GNG node positions
              - marker id 1: LINE_LIST of GNG edges
  /gng/nodes  sensor_msgs/PointCloud2
              - the GNG node positions as a structured, downstream-consumable
                point cloud (one point per node).  The validation node and any
                downstream planner can subscribe to this without a custom msg.

RESEARCH NOTE (guidance, not code behaviour): do NOT auto-tune the GNG
hyperparameters against the sim cloud — it is a raster approximation of the
Mid360.  See gng_core.py for the full rationale.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from cell_gng.gng_core import GrowingNeuralGas


class GngNode(Node):
    def __init__(self):
        super().__init__("gng_node")

        # ---- Parameters (declared, never hardcoded in the logic) -----------
        self.declare_parameter("input_cloud_topic", "/livox/points")
        self.declare_parameter("output_frame_override", "")  # "" = use cloud frame
        self.declare_parameter("voxel_size", 0.10)           # m; <=0 disables
        self.declare_parameter("max_nodes", 200)
        self.declare_parameter("samples_per_cloud", 300)     # random draws/callback
        self.declare_parameter("eps_b", 0.05)
        self.declare_parameter("eps_n", 0.0006)
        self.declare_parameter("max_age", 50)
        self.declare_parameter("lambda_insert", 100)
        self.declare_parameter("alpha", 0.5)
        self.declare_parameter("beta", 0.0005)
        self.declare_parameter("utility_k", 3.0)
        # ROI bounds [xmin, xmax, ymin, ymax, zmin, zmax] in the cloud frame.
        self.declare_parameter("roi_bounds", [-3.0, 3.0, -3.0, 3.0, -0.05, 2.5])
        self.declare_parameter("publish_rate_hz", 5.0)

        p = self.get_parameter
        self._input_topic = p("input_cloud_topic").value
        self._frame_override = p("output_frame_override").value
        self._voxel = float(p("voxel_size").value)
        self._samples_per_cloud = int(p("samples_per_cloud").value)
        self._roi = [float(v) for v in p("roi_bounds").value]
        self._rng = np.random.default_rng(0)

        self._gng = GrowingNeuralGas(
            dim=3,
            max_nodes=int(p("max_nodes").value),
            eps_b=float(p("eps_b").value),
            eps_n=float(p("eps_n").value),
            max_age=int(p("max_age").value),
            lambda_insert=int(p("lambda_insert").value),
            alpha=float(p("alpha").value),
            beta=float(p("beta").value),
            utility_k=float(p("utility_k").value),
        )

        self._last_frame = "world"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_cb, sensor_qos)
        self._graph_pub = self.create_publisher(MarkerArray, "/gng/graph", 1)
        self._nodes_pub = self.create_publisher(PointCloud2, "/gng/nodes", 1)

        rate = float(p("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish)

        self.get_logger().info(
            f"GNG node up. Subscribing PointCloud2 on '{self._input_topic}'. "
            f"voxel={self._voxel} max_nodes={p('max_nodes').value} "
            f"ROI={self._roi}. (STUB GNG-U2 — see gng_core.py)")

    # ------------------------------------------------------------------ input
    def _cloud_cb(self, msg: PointCloud2):
        self._last_frame = msg.header.frame_id or "world"
        pts = self._extract_xyz(msg)
        if pts.shape[0] == 0:
            return
        pts = self._crop_roi(pts)
        if pts.shape[0] == 0:
            return
        if self._voxel > 0:
            pts = self._voxel_downsample(pts, self._voxel)
        if pts.shape[0] == 0:
            return
        # Feed a random subset to the GNG (insertion cadence is in gng_core).
        n = min(self._samples_per_cloud, pts.shape[0])
        idx = self._rng.choice(pts.shape[0], size=n, replace=False)
        for s in pts[idx]:
            self._gng.step(s)

    def _extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        # read_points handles arbitrary field layouts; we only need xyz.
        arr = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if arr is None:
            return np.empty((0, 3))
        xyz = np.column_stack(
            (arr["x"].astype(float), arr["y"].astype(float), arr["z"].astype(float)))
        return xyz

    def _crop_roi(self, pts: np.ndarray) -> np.ndarray:
        x0, x1, y0, y1, z0, z1 = self._roi
        m = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1) &
             (pts[:, 1] >= y0) & (pts[:, 1] <= y1) &
             (pts[:, 2] >= z0) & (pts[:, 2] <= z1))
        return pts[m]

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        keys = np.floor(pts / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    # ----------------------------------------------------------------- output
    def _publish(self):
        nodes = self._gng.nodes
        edges = self._gng.edges
        stamp = self.get_clock().now().to_msg()
        frame = self._frame_override or self._last_frame

        self._graph_pub.publish(self._make_markers(nodes, edges, stamp, frame))
        self._nodes_pub.publish(self._make_node_cloud(nodes, stamp, frame))

    def _make_markers(self, nodes, edges, stamp, frame) -> MarkerArray:
        ma = MarkerArray()

        node_m = Marker()
        node_m.header.stamp = stamp
        node_m.header.frame_id = frame
        node_m.ns = "gng_nodes"
        node_m.id = 0
        node_m.type = Marker.SPHERE_LIST
        node_m.action = Marker.ADD
        node_m.scale.x = node_m.scale.y = node_m.scale.z = 0.05
        node_m.color.r, node_m.color.g, node_m.color.b, node_m.color.a = 0.1, 0.8, 1.0, 1.0
        node_m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in nodes]
        ma.markers.append(node_m)

        edge_m = Marker()
        edge_m.header.stamp = stamp
        edge_m.header.frame_id = frame
        edge_m.ns = "gng_edges"
        edge_m.id = 1
        edge_m.type = Marker.LINE_LIST
        edge_m.action = Marker.ADD
        edge_m.scale.x = 0.01
        edge_m.color.r, edge_m.color.g, edge_m.color.b, edge_m.color.a = 1.0, 0.6, 0.0, 0.8
        for (i, j) in edges:
            if i < len(nodes) and j < len(nodes):
                edge_m.points.append(Point(x=float(nodes[i][0]), y=float(nodes[i][1]), z=float(nodes[i][2])))
                edge_m.points.append(Point(x=float(nodes[j][0]), y=float(nodes[j][1]), z=float(nodes[j][2])))
        ma.markers.append(edge_m)
        return ma

    def _make_node_cloud(self, nodes, stamp, frame) -> PointCloud2:
        header = Header(stamp=stamp, frame_id=frame)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = [(float(p[0]), float(p[1]), float(p[2])) for p in nodes]
        return point_cloud2.create_cloud(header, fields, data)


def main(args=None):
    rclpy.init(args=args)
    node = GngNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
