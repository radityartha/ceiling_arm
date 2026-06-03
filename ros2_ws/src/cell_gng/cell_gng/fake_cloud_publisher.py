"""
fake_cloud_publisher.py — TEST/VERIFICATION helper (NOT for real hardware).

Publishes a synthetic sensor_msgs/PointCloud2 with points sampled on the
surfaces of the two validation boxes defined in cell_gazebo_sim/worlds/cell.sdf
(box_1 at (1.0, 0.0, 0.25), box_2 at (-1.0, 0.5, 0.2)), plus a ground plane.

Why this exists:
  The Gazebo Fortress gpu_lidar/depth_camera require an Ogre render context.
  On GPUs where that render context is unavailable (e.g. some Intel iGPUs),
  the sensors crash, so the sim cloud cannot be produced locally.  This node
  produces an equivalent PointCloud2 on the SAME topic and type, letting us
  verify the GNG input contract, network growth, MarkerArray output, and the
  ground-truth validation maths independently of the GPU.

  On a machine with working sensor rendering, you DO NOT need this node — the
  GNG node simply subscribes to the real bridged /livox/points instead.  The
  GNG node code is identical either way (topic is a parameter).

  The `box_pose` argument lets you shift box_1 to mimic "moving an object"
  for the dynamic-update part of the Phase 3 verify.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


def _box_surface_points(center, size, n, rng):
    """Sample n points roughly on the surface of an axis-aligned box."""
    cx, cy, cz = center
    sx, sy, sz = size
    pts = rng.uniform(-0.5, 0.5, size=(n, 3)) * np.array([sx, sy, sz])
    # Snap each point to the nearest face to make it look like a surface.
    face = rng.integers(0, 3, size=n)
    sign = rng.choice([-0.5, 0.5], size=n)
    for k in range(n):
        ax = face[k]
        pts[k, ax] = sign[k] * (sx, sy, sz)[ax]
    pts += np.array([cx, cy, cz])
    return pts


class FakeCloudPublisher(Node):
    def __init__(self):
        super().__init__("fake_cloud_publisher")
        self.declare_parameter("topic", "/livox/points")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("points_per_box", 400)
        self.declare_parameter("ground_points", 600)
        # box_1 pose can be overridden to simulate a moved object.
        self.declare_parameter("box1_xyz", [1.0, 0.0, 0.25])
        self.declare_parameter("box2_xyz", [-1.0, 0.5, 0.2])

        self._frame = self.get_parameter("frame_id").value
        self._ppb = int(self.get_parameter("points_per_box").value)
        self._gp = int(self.get_parameter("ground_points").value)
        self._rng = np.random.default_rng(1)

        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter("topic").value, 5)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f"Fake cloud publisher up on '{self.get_parameter('topic').value}' "
            f"(frame={self._frame}). TEST ONLY — not for real hardware.")

    def _tick(self):
        b1 = [float(v) for v in self.get_parameter("box1_xyz").value]
        b2 = [float(v) for v in self.get_parameter("box2_xyz").value]
        clouds = [
            _box_surface_points(b1, (0.3, 0.3, 0.5), self._ppb, self._rng),
            _box_surface_points(b2, (0.4, 0.2, 0.4), self._ppb, self._rng),
        ]
        # Ground plane patch.
        ground = self._rng.uniform(-3, 3, size=(self._gp, 3))
        ground[:, 2] = self._rng.normal(0.0, 0.005, size=self._gp)
        clouds.append(ground)
        pts = np.vstack(clouds).astype(np.float32)
        # Light sensor noise.
        pts += self._rng.normal(0.0, 0.005, size=pts.shape).astype(np.float32)

        header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id=self._frame)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, pts.tolist())
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
