"""One-shot mapping of the STATIC work table into a saved collision box.

The work table is static, so (like the GNG capability map) it is captured ONCE
offline and reused at runtime instead of being re-sensed every frame. This tool
listens to the RGBD collision clouds for a few seconds, fuses them into `world`,
detects the dominant horizontal plane (the table top) and fits an axis-aligned
box from that surface down to the floor, then saves the box to a file. At runtime
table_collision.py loads that file and publishes it as a static MoveIt
CollisionObject -- reliable, occlusion-free collision geometry for the table,
leaving the live octomap for genuinely unknown/dynamic stuff.

Run with the cameras + world->camera TFs up (launch_workcell.sh provides both),
the table CLEAR of objects (so the dominant plane is the bare table top):

    ros2 run reachability_gng map_table
    ros2 run reachability_gng map_table --ros-args -p output:=/tmp/work_table.npz

Saved npz: center (3,), size (3,), frame (str). Re-run whenever the table moves.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


class MapTable(Node):
    def __init__(self):
        super().__init__('map_table')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('cloud_topic_suffix', 'collision_cloud')
        self.declare_parameter('capture_seconds', 3.0)
        self.declare_parameter('min_table_z', 0.3)   # search band for the top
        self.declare_parameter('max_table_z', 1.9)
        self.declare_parameter('plane_thickness', 0.03)  # inlier band around top
        self.declare_parameter('z_bin', 0.01)        # histogram bin for the top
        self.declare_parameter('min_inliers', 500)
        self.declare_parameter('xy_percentile', 1.0)  # robust extent (reject px)
        self.declare_parameter('floor_z', 0.0)
        self.declare_parameter('to_floor', True)     # box spans floor->top
        self.declare_parameter('output', '/tmp/work_table.npz')

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('cloud_topic_suffix').value
        self.capture_s = float(self.get_parameter('capture_seconds').value)
        self.min_z = float(self.get_parameter('min_table_z').value)
        self.max_z = float(self.get_parameter('max_table_z').value)
        self.thick = float(self.get_parameter('plane_thickness').value)
        self.z_bin = float(self.get_parameter('z_bin').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.pct = float(self.get_parameter('xy_percentile').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        self.to_floor = bool(self.get_parameter('to_floor').value)
        self.output = self.get_parameter('output').value
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pts = []          # accumulated world points
        self._t0 = None
        self._done = False
        for ns in nss:
            self.create_subscription(
                PointCloud2, f'/{ns}/{self.suffix}',
                self._on_cloud, 1)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'map_table capturing {self.capture_s}s from '
            f'{[f"/{n}/{self.suffix}" for n in nss]} ... keep the table CLEAR')

    def _on_cloud(self, msg):
        if self._done:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
        a = a[np.isfinite(a).all(axis=1)]
        if a.size == 0:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        self._pts.append(a @ R.T + np.array([t.x, t.y, t.z]))
        if self._t0 is None:
            self._t0 = self.get_clock().now()

    def _tick(self):
        if self._done or self._t0 is None:
            return
        if (self.get_clock().now() - self._t0).nanoseconds < self.capture_s * 1e9:
            return
        self._done = True
        self._fit_and_save()
        rclpy.shutdown()

    def _fit_and_save(self):
        pts = np.concatenate(self._pts, axis=0)
        band = pts[(pts[:, 2] >= self.min_z) & (pts[:, 2] <= self.max_z)]
        if len(band) < self.min_inliers:
            self.get_logger().error(
                f'only {len(band)} pts in z=[{self.min_z},{self.max_z}]; '
                'cannot find a table plane -- check cameras / band params')
            return
        # dominant horizontal layer = densest z bin (the bare table top)
        edges = np.arange(self.min_z, self.max_z + self.z_bin, self.z_bin)
        hist, _ = np.histogram(band[:, 2], bins=edges)
        top_z = float(edges[hist.argmax()] + self.z_bin / 2)
        inliers = band[np.abs(band[:, 2] - top_z) <= self.thick]
        if len(inliers) < self.min_inliers:
            self.get_logger().error(
                f'dominant plane at z={top_z:.3f} has only {len(inliers)} '
                f'inliers (< {self.min_inliers}); aborting')
            return
        # robust xy extent (percentile clip rejects stray points)
        xmin, xmax = np.percentile(inliers[:, 0], [self.pct, 100 - self.pct])
        ymin, ymax = np.percentile(inliers[:, 1], [self.pct, 100 - self.pct])
        if self.to_floor:
            zmin, zmax = self.floor_z, top_z
        else:
            zmin, zmax = top_z - self.thick, top_z + self.thick
        center = np.array([(xmin + xmax) / 2, (ymin + ymax) / 2,
                           (zmin + zmax) / 2], float)
        size = np.array([xmax - xmin, ymax - ymin, zmax - zmin], float)
        np.savez(self.output, center=center, size=size, frame=self.world_frame)
        self.get_logger().info(
            f'work table mapped: top_z={top_z:.3f} m, {len(inliers)} inliers; '
            f'box center={center.round(3).tolist()} size={size.round(3).tolist()} '
            f'-> saved {self.output}')


def main():
    rclpy.init()
    node = MapTable()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
