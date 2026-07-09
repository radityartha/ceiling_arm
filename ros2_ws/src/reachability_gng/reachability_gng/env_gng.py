"""Environment perception map as an online GNG topological map (Phase 1).

Sensei's (Kubota/Nando) method represents the environment as a Growing Neural
Gas graph (nodes + edges) from the RGBD cloud -- NOT voxels/octomap. Consumes the
Isaac `seg_cloud` (already in `world`), and per tick: crop-z, voxel downsample,
radius outlier removal, TF arm self-filter, mini-batch GNG, stale-node prune.
Reuses the tested GNG core with a pure xyz vector (no q). Publishes
/topo_map/markers (green nodes + edges).
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG, GNGParams

# Kinova Gen3 Lite chain (for self-filter capsules) + finger links (spheres).
_ARM_CHAIN = ['shoulder_link', 'arm_link', 'forearm_link',
              'lower_wrist_link', 'upper_wrist_link', 'end_effector_link']
_FINGER_LINKS = ['left_finger_prox_link', 'left_finger_dist_link',
                 'right_finger_prox_link', 'right_finger_dist_link']


def _seg_dist2(P, a, b):
    """Squared distance from every point in P (N,3) to segment a-b."""
    ab = b - a
    t = np.clip(((P - a) @ ab) / (float(ab @ ab) + 1e-9), 0.0, 1.0)
    d = P - (a + t[:, None] * ab)
    return np.einsum('ij,ij->i', d, d)


def _cloud_xyz(msg):
    """Finite (x,y,z) rows of a PointCloud2 as (N,3); seg_cloud is world-frame."""
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
    return a[np.isfinite(a).all(axis=1)].astype(np.float64)


def _voxel_downsample(pts, leaf):
    """Keep one point per `leaf`-sized voxel."""
    if len(pts) == 0 or leaf <= 0.0:
        return pts
    _, idx = np.unique(np.floor(pts / leaf).astype(np.int64), axis=0,
                       return_index=True)
    return pts[idx]


def _radius_outlier_removal(pts, radius, min_neighbors):
    """Drop sparse points (depth flying-pixel streaks): fewer than min_neighbors
    within radius. Real surfaces are dense (~16/5cm); flying pixels have <=2."""
    if len(pts) == 0 or min_neighbors <= 0:
        return pts
    from scipy.spatial import cKDTree
    counts = cKDTree(pts).query_ball_point(pts, radius, return_length=True)
    return pts[counts > min_neighbors]   # count includes self -> require >


class EnvGNG(Node):
    def __init__(self):
        super().__init__('env_gng')
        p = self.declare_parameter
        p('camera_namespaces', ['rgbd', 'rgbd2'])
        p('cloud_topic_suffix', 'seg_cloud')
        p('world_frame', 'world')
        p('min_z', 0.02)
        p('max_z', 1.9)               # crops ceiling gantry (platform z=2.05)
        p('leaf_size', 0.02)
        p('outlier_radius', 0.05)     # radius outlier removal (flying pixels)
        p('outlier_min_neighbors', 3)
        p('samples_per_tick', 800)    # online mini-batch size
        p('update_hz', 10.0)
        p('max_nodes', 800)
        p('lam', 100)                 # insert a node every lam steps
        p('prune_dist', 0.10)         # delete nodes floating > this from data
        p('prune_every', 5)           # ...every this many ticks (stale-bridge fix)
        p('self_filter', True)        # drop points on the robot's own arms (TF)
        p('arm_prefixes', ['t1_a1', 't1_a2', 't2_a1', 't2_a2'])
        p('self_filter_radius', 0.07)
        p('finger_radius', 0.05)
        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.suffix = g('cloud_topic_suffix')
        self.min_z, self.max_z = float(g('min_z')), float(g('max_z'))
        self.leaf = float(g('leaf_size'))
        self.outlier_r = float(g('outlier_radius'))
        self.outlier_min = int(g('outlier_min_neighbors'))
        self.batch = int(g('samples_per_tick'))
        self.prune_dist = float(g('prune_dist'))
        self.prune_every = int(g('prune_every'))
        self.self_filter = bool(g('self_filter'))
        self.arm_prefixes = list(g('arm_prefixes'))
        self.filter_r = float(g('self_filter_radius'))
        self.finger_r = float(g('finger_radius'))
        self._tick_i = 0
        self._latest = {}             # newest downsampled cloud per camera
        self._rng = np.random.default_rng(0)
        self.gng = GNG(dim=3, task_dim=3, params=GNGParams(
            max_nodes=int(g('max_nodes')), lam=int(g('lam'))))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for ns in list(g('camera_namespaces')):
            self.create_subscription(PointCloud2, f'/{ns}/{self.suffix}',
                                     lambda m, n=ns: self._on_cloud(n, m), 1)
        self.pub = self.create_publisher(MarkerArray, '/topo_map/markers', 1)
        self.create_timer(1.0 / max(float(g('update_hz')), 1.0), self._tick)
        self.get_logger().info(f'env_gng up; max_nodes={self.gng.params.max_nodes}')

    def _on_cloud(self, ns, msg):
        pts = _cloud_xyz(msg)
        if len(pts) == 0:
            return
        z = pts[:, 2]
        pts = _voxel_downsample(pts[(z >= self.min_z) & (z <= self.max_z)], self.leaf)
        if len(pts):
            pts = _radius_outlier_removal(pts, self.outlier_r, self.outlier_min)
        if self.self_filter and len(pts):
            pts = self._apply_self_filter(pts)
        self._latest[ns] = pts

    def _frame_pos(self, frame, cache):
        """World position of a TF frame (cached per call); None if TF absent."""
        if frame not in cache:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame, frame, rclpy.time.Time()).transform.translation
                cache[frame] = np.array([t.x, t.y, t.z])
            except (LookupException, ConnectivityException, ExtrapolationException):
                cache[frame] = None
        return cache[frame]

    def _apply_self_filter(self, pts):
        """Remove points on the robot arms (link capsules + finger spheres)."""
        cache, keep = {}, np.ones(len(pts), dtype=bool)
        r2, fr2 = self.filter_r ** 2, self.finger_r ** 2
        for pre in self.arm_prefixes:
            chain = [self._frame_pos(f'{pre}_{s}', cache) for s in _ARM_CHAIN]
            for a, b in zip(chain[:-1], chain[1:]):
                if a is not None and b is not None:
                    keep &= _seg_dist2(pts, a, b) >= r2
            for s in _FINGER_LINKS:
                c = self._frame_pos(f'{pre}_{s}', cache)
                if c is not None:
                    d = pts - c
                    keep &= np.einsum('ij,ij->i', d, d) >= fr2
        return pts[keep]

    def _pool(self):
        clouds = [c for c in self._latest.values() if len(c)]
        return np.vstack(clouds) if clouds else np.empty((0, 3))

    def _tick(self):
        pool = self._pool()
        if len(pool) < 2:
            return
        if len(self.gng.W) == 0:
            self.gng.init_two(pool)
        for i in self._rng.choice(len(pool), size=min(self.batch, len(pool)),
                                  replace=False):
            self.gng.step(pool[i])
        self._tick_i += 1
        if self.prune_dist > 0 and self._tick_i % self.prune_every == 0:
            self._prune_stale(pool)
        self._publish()

    def _prune_stale(self, pool):
        """Delete nodes floating > prune_dist from any input point (Meso Emin)."""
        if len(self.gng.W) <= 2:
            return
        from scipy.spatial import cKDTree
        far = np.where(cKDTree(pool).query(self.gng.W)[0] > self.prune_dist)[0]
        if len(far):
            self.gng.remove_nodes(far)

    def _publish(self):
        W = self.gng.W
        if len(W) == 0:
            return
        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        now = self.get_clock().now().to_msg()

        def mk(ns, mid, mtype, size):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.world_frame, now
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = size
            m.color = green
            return m

        nodes = mk('topo_nodes', 0, Marker.SPHERE_LIST, 0.02)
        nodes.points = [Point(x=float(w[0]), y=float(w[1]), z=float(w[2])) for w in W]
        edges = mk('topo_edges', 1, Marker.LINE_LIST, 0.005)
        for e in self.gng._edges:
            i, j = tuple(e)
            edges.points += [Point(x=float(W[i][0]), y=float(W[i][1]), z=float(W[i][2])),
                             Point(x=float(W[j][0]), y=float(W[j][1]), z=float(W[j][2]))]
        self.pub.publish(MarkerArray(markers=[nodes, edges]))


def main():
    rclpy.init()
    node = EnvGNG()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
