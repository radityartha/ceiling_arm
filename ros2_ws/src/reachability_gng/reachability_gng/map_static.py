"""One-shot mapping of STATIC known geometry into saved collision boxes.

Anything static and rigid -- the work table, a cabinet, a fridge, a wall,
fixtures -- is captured ONCE offline (like the GNG capability map) and reused at
runtime instead of being re-sensed every frame. Mapping it once as an exact box
gives reliable, occlusion-free collision geometry, so the live octomap is left
for genuinely unknown / dynamic obstacles (and no occlusion "shadows" punch holes
in the static surfaces).

This tool listens to the RGBD collision clouds for a few seconds, fuses them into
`world`, optionally restricts to a region of interest, detects the dominant
horizontal surface there (the piece's top) and fits an axis-aligned box from that
surface down to the floor. The box is saved (appended) under a NAME into a shared
file; static_collision.py loads every saved box and publishes them as MoveIt
CollisionObjects.

Map one piece at a time: give it a `name` and, if several pieces are in view, an
`roi` [xmin xmax ymin ymax] so the dominant surface is that piece's top. Re-running
with the same name updates it; a new name appends. The default (no roi) maps the
single dominant surface in the whole scene -- run with that piece CLEAR of movable
objects (and the arm tucked away) so the top is the densest horizontal layer.

    ros2 run reachability_gng map_static                        # dominant piece -> 'work_table'
    ros2 run reachability_gng map_static --ros-args \
        -p name:=cabinet -p roi:="[2.0, 3.0, -1.5, -0.5]"

Saved npz: names (N,), centers (N,3), sizes (N,3), frame (str).
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


class MapStatic(Node):
    def __init__(self):
        super().__init__('map_static')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('cloud_topic_suffix', 'collision_cloud')
        self.declare_parameter('capture_seconds', 3.0)
        self.declare_parameter('min_z', 0.3)          # search band for the top
        self.declare_parameter('max_z', 1.9)
        self.declare_parameter('plane_thickness', 0.03)  # inlier band around top
        self.declare_parameter('z_bin', 0.01)         # histogram bin for the top
        self.declare_parameter('min_inliers', 500)
        self.declare_parameter('xy_percentile', 1.0)  # robust extent (reject px)
        self.declare_parameter('floor_z', 0.0)
        self.declare_parameter('to_floor', True)      # box spans floor->top
        self.declare_parameter('output', '/tmp/static_geometry.npz')
        self.declare_parameter('name', 'work_table')  # id of the mapped piece
        self.declare_parameter('append', True)        # keep other saved pieces
        # [xmin, xmax, ymin, ymax] in world; disabled when xmax<=xmin (default).
        self.declare_parameter('roi', [0.0, 0.0, 0.0, 0.0])

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('cloud_topic_suffix').value
        self.capture_s = float(self.get_parameter('capture_seconds').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.thick = float(self.get_parameter('plane_thickness').value)
        self.z_bin = float(self.get_parameter('z_bin').value)
        self.min_inliers = int(self.get_parameter('min_inliers').value)
        self.pct = float(self.get_parameter('xy_percentile').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        self.to_floor = bool(self.get_parameter('to_floor').value)
        self.output = self.get_parameter('output').value
        self.name = str(self.get_parameter('name').value)
        self.append = bool(self.get_parameter('append').value)
        self.roi = [float(v) for v in self.get_parameter('roi').value]
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
        roi_note = (f'roi={self.roi}' if self._roi_enabled()
                    else 'no roi (dominant surface in scene)')
        self.get_logger().info(
            f"map_static capturing {self.capture_s}s from "
            f"{[f'/{n}/{self.suffix}' for n in nss]} for '{self.name}' "
            f"({roi_note}) ... keep it CLEAR of movable objects")

    def _roi_enabled(self):
        return len(self.roi) == 4 and self.roi[1] > self.roi[0]

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
        if self._roi_enabled():
            xmn, xmx, ymn, ymx = self.roi
            pts = pts[(pts[:, 0] >= xmn) & (pts[:, 0] <= xmx)
                      & (pts[:, 1] >= ymn) & (pts[:, 1] <= ymx)]
        band = pts[(pts[:, 2] >= self.min_z) & (pts[:, 2] <= self.max_z)]
        if len(band) < self.min_inliers:
            self.get_logger().error(
                f'only {len(band)} pts in z=[{self.min_z},{self.max_z}]'
                + (' within roi' if self._roi_enabled() else '')
                + f'; cannot find a surface for "{self.name}" -- check '
                'cameras / band / roi params')
            return
        # dominant horizontal layer = densest z bin (the piece's bare top)
        edges = np.arange(self.min_z, self.max_z + self.z_bin, self.z_bin)
        hist, _ = np.histogram(band[:, 2], bins=edges)
        top_z = float(edges[hist.argmax()] + self.z_bin / 2)
        inliers = band[np.abs(band[:, 2] - top_z) <= self.thick]
        if len(inliers) < self.min_inliers:
            self.get_logger().error(
                f'dominant plane at z={top_z:.3f} has only {len(inliers)} '
                f'inliers (< {self.min_inliers}); aborting "{self.name}"')
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
        self._save_entry(center, size)
        self.get_logger().info(
            f'"{self.name}" mapped: top_z={top_z:.3f} m, {len(inliers)} inliers; '
            f'box center={center.round(3).tolist()} size={size.round(3).tolist()} '
            f'-> saved {self.output}')

    def _save_entry(self, center, size):
        names, centers, sizes = [], [], []
        if self.append and os.path.exists(self.output):
            d = np.load(self.output, allow_pickle=True)
            names = [str(x) for x in d['names']]
            centers = [np.asarray(c, float) for c in d['centers']]
            sizes = [np.asarray(s, float) for s in d['sizes']]
        if self.name in names:                 # re-map replaces the same piece
            i = names.index(self.name)
            centers[i], sizes[i] = center, size
        else:
            names.append(self.name)
            centers.append(center)
            sizes.append(size)
        np.savez(self.output, names=np.array(names, dtype=object),
                 centers=np.array(centers, float),
                 sizes=np.array(sizes, float), frame=self.world_frame)


def main():
    rclpy.init()
    node = MapStatic()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
