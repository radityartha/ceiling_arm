"""One-shot GNG mapping of the STATIC scene structure into a saved graph.

The env_gng topological map is online and never persisted, so it re-grows from
scratch every run and its node/edge layout differs each time (sample order,
insert/prune churn). For anything static and rigid -- walls, the work table, the
floor, fixtures -- that variability is unwanted noise. This tool captures the
EMPTY scene ONCE (arm tucked away, movable objects/people cleared), trains a GNG
on the fused RGBD cloud, and saves the graph (reusing GNG.save). At runtime:

    topo_static_pub.py   loads it -> /topo_map/static/markers (reproducible)
    env_gng.py           loads it as a background model: cloud points near a
                         static node are subtracted, so the LIVE GNG maps only
                         genuinely dynamic obstacles (objects, people, arms).

Mirrors map_static.py (exact static boxes) vs collision_cloud (live octomap),
one level up: static GNG mapped once, live GNG for the unknown/dynamic rest.

    ros2 run reachability_gng map_topo_static                 # -> /tmp/topo_static.npz
    ros2 run reachability_gng map_topo_static --ros-args -p capture_seconds:=5.0
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.env_gng import (_cloud_xyz, _radius_outlier_removal,
                                      _voxel_downsample, apply_self_filter)
from reachability_gng.gng import GNG, GNGParams


class MapTopoStatic(Node):
    def __init__(self):
        super().__init__('map_topo_static')
        p = self.declare_parameter
        p('camera_namespaces', ['rgbd', 'rgbd2'])
        # depth_cloud (geometry only, no segmentation gate) so the static capture
        # is independent of seg_source; pass seg_cloud to fall back to the old
        # colour-by-instance cloud.
        p('cloud_topic_suffix', 'depth_cloud')
        p('min_z', 0.02)
        p('max_z', 1.75)              # match topo_fusion.launch.py default (crops
        #                              the overhead gantry/arm-mount)
        # Bound each camera's contribution to its own immediate work area:
        # drop points farther than this from THAT camera's own world-X position
        # (not a fixed world-X band -- rgbd and rgbd2 sit at very different X).
        # Keeps far-room clutter (walls, furniture, people outside the work
        # cell) out of the static map. <=0 disables.
        p('max_x_from_camera', 2.5)
        p('optical_frame_suffix', '_camera_optical')
        p('leaf_size', 0.02)
        p('outlier_radius', 0.05)
        p('outlier_min_neighbors', 3)
        p('capture_seconds', 8.0)      # accumulate clouds this long, then fit
        p('max_nodes', 1800)           # dense static backbone (spacing ~0.085 m)
        p('lam', 100)
        # epochs: 0 = AUTO (recommended). GNG needs ~max_nodes*lam samples to
        # fill the graph; auto sets epochs so epochs*points hits that target with
        # margin, so the node count matches max_nodes regardless of pool size.
        # Set >0 to force a fixed epoch count.
        p('epochs', 0)
        # Cap the training pool: GNG.fit is O(epochs*points*nodes) in pure
        # Python, so a 27k-point pool * 30 epochs * 1800 nodes took ~2.5 min.
        # Subsampling to <=fit_max_points bounds per-epoch cost (nodes still tile
        # the whole surface -- the extra points were redundant). <=0 disables.
        p('fit_max_points', 12000)
        p('output', '/tmp/topo_static.npz')
        # Self-filter the robot arms out of the static capture via TF, so the
        # arms need NOT be physically moved away -- their points are dropped, not
        # baked into the static map. Same capsule/sphere filter env_gng uses.
        p('world_frame', 'world')
        p('self_filter', True)
        p('arm_prefixes', ['t1_a1', 't1_a2', 't2_a1', 't2_a2'])
        p('self_filter_radius', 0.07)
        p('finger_radius', 0.05)
        g = lambda k: self.get_parameter(k).value
        self.suffix = g('cloud_topic_suffix')
        self.world_frame = g('world_frame')
        self.min_z, self.max_z = float(g('min_z')), float(g('max_z'))
        self.max_x_from_camera = float(g('max_x_from_camera'))
        self.optical_suffix = g('optical_frame_suffix')
        self.leaf = float(g('leaf_size'))
        self.outlier_r = float(g('outlier_radius'))
        self.outlier_min = int(g('outlier_min_neighbors'))
        self.capture_s = float(g('capture_seconds'))
        self.max_nodes = int(g('max_nodes'))
        self.lam = int(g('lam'))
        self.epochs = int(g('epochs'))
        self.fit_max_points = int(g('fit_max_points'))
        self.output = g('output')
        self.self_filter = bool(g('self_filter'))
        self.arm_prefixes = list(g('arm_prefixes'))
        self.filter_r = float(g('self_filter_radius'))
        self.finger_r = float(g('finger_radius'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pts = []               # accumulated downsampled world points
        self._t0 = None
        self._done = False
        # Put the cloud subscription in its own callback group and spin on a
        # MultiThreadedExecutor (see main): the TransformListener subscribes to
        # the /tf firehose (~19 Hz), which on a single-threaded executor starves
        # this cloud callback so the capture never accumulates / never saves
        # (intermittent). A separate group lets the cloud run on its own thread.
        self._cb_group = MutuallyExclusiveCallbackGroup()
        for ns in list(g('camera_namespaces')):
            self.create_subscription(PointCloud2, f'/{ns}/{self.suffix}',
                                     lambda m, ns=ns: self._on_cloud(ns, m),
                                     qos_profile_sensor_data,
                                     callback_group=self._cb_group)
        self.get_logger().info(
            f'map_topo_static capturing {self.capture_s}s of static scene -- '
            'keep it CLEAR of movable objects and tuck the arms away')

    def _on_cloud(self, ns, msg):
        # The capture deadline is checked HERE, not on a timer: the per-cloud
        # processing keeps the single-threaded executor busy on this callback, so
        # a timer would be starved and never fire (the node would capture forever
        # and never save). Since clouds arrive continuously, checking elapsed
        # time inside the callback is reliable.
        if self._done:
            return
        pts = _cloud_xyz(msg)          # seg_cloud is already world-frame
        if len(pts) == 0:
            return
        if self.max_x_from_camera > 0:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, f'{ns}{self.optical_suffix}', rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException):
                return  # can't bound without knowing this camera's position -- drop the frame
            cam_x = tf.transform.translation.x
            pts = pts[np.abs(pts[:, 0] - cam_x) <= self.max_x_from_camera]
            if len(pts) == 0:
                return
        z = pts[:, 2]
        pts = _voxel_downsample(pts[(z >= self.min_z) & (z <= self.max_z)],
                                self.leaf)
        if len(pts):
            pts = _radius_outlier_removal(pts, self.outlier_r, self.outlier_min)
        if self.self_filter and len(pts):
            pts = apply_self_filter(pts, self.tf_buffer, self.world_frame,
                                    self.arm_prefixes, self.filter_r, self.finger_r)
        if len(pts):
            self._pts.append(pts)
            if self._t0 is None:
                self._t0 = self.get_clock().now()
        if (self._t0 is not None
                and (self.get_clock().now() - self._t0).nanoseconds
                >= self.capture_s * 1e9):
            self._done = True
            self._fit_and_save()
            rclpy.shutdown()

    def _fit_and_save(self):
        pool = np.vstack(self._pts)
        pool = _voxel_downsample(pool, self.leaf)   # merge overlapping captures
        if len(pool) < 2:
            self.get_logger().error(
                f'only {len(pool)} static points captured; cannot fit a map -- '
                'check cameras / bands')
            return
        captured = len(pool)
        if 0 < self.fit_max_points < len(pool):     # bound per-epoch cost
            rng = np.random.default_rng(0)           # fixed seed -> reproducible
            pool = pool[rng.choice(len(pool), self.fit_max_points, replace=False)]
        # AUTO epochs: enough passes so epochs*points >= ~1.5*max_nodes*lam to
        # grow the graph to max_nodes and settle, regardless of pool size.
        epochs = self.epochs
        if epochs <= 0:
            target = int(1.5 * self.max_nodes * self.lam)
            epochs = max(3, -(-target // len(pool)))   # ceil div
        gng = GNG(dim=3, task_dim=3,
                  params=GNGParams(max_nodes=self.max_nodes, lam=self.lam))
        gng.fit(pool, epochs=epochs)
        self.epochs = epochs   # for the log line
        # freeze the whole graph: it is a fixed background model from here on.
        gng.pinned[:] = True
        gng.save(self.output)
        self.get_logger().info(
            f'static GNG mapped: {captured} captured -> fit on {len(pool)} pts '
            f'x {self.epochs} epochs -> {len(gng.W)} nodes, {len(gng._edges)} '
            f'edges -> saved {self.output}')


def main():
    rclpy.init()
    node = MapTopoStatic()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
