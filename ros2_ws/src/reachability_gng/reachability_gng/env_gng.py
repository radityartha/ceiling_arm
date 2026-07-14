"""Environment perception map as an online GNG topological map (Phase 1).

Sensei's (Kubota/Nando) method represents the environment as a Growing Neural
Gas graph (nodes + edges) from the RGBD cloud -- NOT voxels/octomap. Consumes the
Isaac `seg_cloud` (already in `world`), and per tick: crop-z, voxel downsample,
radius outlier removal, TF arm self-filter, mini-batch GNG, stale-node prune.
Reuses the tested GNG core with a pure xyz vector (no q). Publishes
/topo_map/markers (green nodes + edges).
"""
from __future__ import annotations

from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG, GNGParams

# Kinova Gen3 Lite arm chain (self-filter capsules along consecutive links).
_ARM_CHAIN = ['shoulder_link', 'arm_link', 'forearm_link',
              'lower_wrist_link', 'upper_wrist_link', 'end_effector_link']
# Gripper self-filter as CAPSULES: end_effector -> each finger's proximal joint
# covers the gripper BODY (no TF frame lives there, so it used to leak green),
# then proximal -> distal covers the fingers. This replaces the old point
# spheres at the finger frames, which left the gripper block uncovered.
_GRIPPER_SEGMENTS = [
    ('end_effector_link', 'left_finger_prox_link'),
    ('end_effector_link', 'right_finger_prox_link'),
    ('left_finger_prox_link', 'left_finger_dist_link'),
    ('right_finger_prox_link', 'right_finger_dist_link'),
]
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
    """Drop sparse points (depth flying-pixel streaks): keep only points sharing
    a `radius`-sized grid cell with more than `min_neighbors` points. Real
    surfaces fill a cell densely; flying pixels sit alone.

    Grid-count (pure numpy, O(N)) instead of a KDTree radius query: on a dense
    frame (a clumped object pile) scipy's query_ball_point degrades badly, and
    because env_gng runs on a single-threaded executor ONE such frame freezes
    the whole node forever (0 nodes ever built). The grid count is
    distribution-independent (~0.03 s for 27k pts) so every frame stays bounded.
    Cell membership is a coarser proxy than a true radius ball, but for
    flying-pixel rejection that difference is immaterial."""
    if len(pts) == 0 or min_neighbors <= 0:
        return pts
    keys = np.floor(pts / radius).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True,
                               return_counts=True)
    return pts[counts[inv] > min_neighbors]   # cell count includes self


def _grid_near(query, refs, radius):
    """Per query row (N,3): is any `refs` point within ~`radius`? Voxel-grid hash
    (cell = radius), checking the query cell + its 26 neighbours, so a ref within
    [radius, sqrt(3)*radius] flags the point. O(N) pure-numpy, deterministic.

    Replaces scipy cKDTree.query, which HANGS nondeterministically on dense
    frames here (a zombie test sat in .query for 66 min), and a plain numpy
    brute-force (N*M) which is ~2.6 s/frame (worse under OMP=1) -- both stall
    env_gng's single-threaded executor so it never publishes. This is ~ms."""
    n = len(query)
    if n == 0:
        return np.zeros(0, dtype=bool)
    if len(refs) == 0 or radius <= 0:
        return np.zeros(n, dtype=bool)
    inv = 1.0 / radius
    rk = np.floor(refs * inv).astype(np.int64)
    qk = np.floor(query * inv).astype(np.int64)
    mn = np.minimum(rk.min(0), qk.min(0)) - 2
    rk -= mn
    qk -= mn
    base = int(max(int(rk.max()), int(qk.max())) + 3)
    pack = lambda k: (k[:, 0] * base + k[:, 1]) * base + k[:, 2]
    occ = np.unique(pack(rk))
    near = np.zeros(n, dtype=bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                shifted = qk + np.array([dx, dy, dz], dtype=np.int64)
                near |= np.isin(pack(shifted), occ)
    return near


def _frame_pos(tf_buffer, world_frame, frame, cache):
    """World position of a TF frame (cached per call); None if TF absent."""
    if frame not in cache:
        try:
            t = tf_buffer.lookup_transform(
                world_frame, frame, rclpy.time.Time()).transform.translation
            cache[frame] = np.array([t.x, t.y, t.z])
        except (LookupException, ConnectivityException, ExtrapolationException):
            cache[frame] = None
    return cache[frame]


def arm_positions(tf_buffer, world_frame, arm_prefixes):
    """Snapshot the world position of every arm link + finger frame (dict
    frame->xyz or None). Sampled from LATEST TF."""
    cache = {}
    for pre in arm_prefixes:
        for s in _ARM_CHAIN + _FINGER_LINKS:
            _frame_pos(tf_buffer, world_frame, f'{pre}_{s}', cache)
    return cache


def filter_by_positions(pts, caches, arm_prefixes, filter_r, finger_r):
    """Remove points on the robot arms, given one OR MORE position snapshots
    (`caches`, each from arm_positions). A point is dropped if it is inside an
    arm-link OR gripper capsule in ANY snapshot -- so passing the last few
    frames' snapshots removes the arm along its recent SWEPT path.

    That matters because the Isaac depth cloud carries a SIM-time stamp while TF
    is wall-clock (use_sim_time:=false), so the filter can't look up the arm pose
    at the cloud's capture instant -- it only has LATEST TF, which leads the
    (pipeline-delayed) cloud during motion, leaving the fast-moving end of a
    MOVING arm unfiltered (green nodes on the arm). Filtering against a short
    history of poses covers that lag for every arm. `finger_r` is the gripper
    capsule radius (end_effector->fingers)."""
    keep = np.ones(len(pts), dtype=bool)
    r2, gr2 = filter_r ** 2, finger_r ** 2
    for cache in caches:
        for pre in arm_prefixes:
            chain = [cache.get(f'{pre}_{s}') for s in _ARM_CHAIN]
            for a, b in zip(chain[:-1], chain[1:]):
                if a is not None and b is not None:
                    keep &= _seg_dist2(pts, a, b) >= r2
            for fa, fb in _GRIPPER_SEGMENTS:
                a, b = cache.get(f'{pre}_{fa}'), cache.get(f'{pre}_{fb}')
                if a is not None and b is not None:
                    keep &= _seg_dist2(pts, a, b) >= gr2
    return pts[keep]


def apply_self_filter(pts, tf_buffer, world_frame, arm_prefixes,
                      filter_r, finger_r):
    """Single-snapshot self-filter (for the static capture, where arms are
    stationary). env_gng uses filter_by_positions with a pose history instead."""
    return filter_by_positions(
        pts, [arm_positions(tf_buffer, world_frame, arm_prefixes)],
        arm_prefixes, filter_r, finger_r)


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
        # Filter the arm against the last N pose snapshots (its recent swept path)
        # so a MOVING arm is fully removed despite cloud/TF lag (sim-time cloud
        # stamp vs wall-clock TF -> can't filter at capture time). 1 = old single
        # snapshot behaviour.
        p('self_filter_frames', 6)
        # Static-background subtraction: if a saved static GNG (map_topo_static)
        # is given, cloud points within bg_dist of any static node are dropped
        # BEFORE stepping, so this live map grows only over genuinely dynamic
        # obstacles (objects/people/arms) instead of re-mapping the fixed scene
        # from scratch each run. Empty path (default) = original whole-scene map.
        p('static_map', '')           # path to /tmp/topo_static.npz, or ''
        p('bg_dist', 0.08)            # subtract points this close to a static node
        p('max_edge_len', 0.15)       # don't draw edges longer than this (m); <=0 = off
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
        self._arm_hist = deque(maxlen=max(1, int(g('self_filter_frames'))))
        self.bg_dist = float(g('bg_dist'))
        # Don't draw edges longer than this: GNG leaves a few long "bridge" edges
        # spanning empty space (between sparse dynamic nodes, or across the
        # self-filtered arm gap) that look like structure where there is none.
        # Collision uses NODES not edges, so this is purely a display cleanup.
        # <=0 disables the cap.
        self.max_edge = float(g('max_edge_len'))
        self._bg_nodes = self._load_static_bg(str(g('static_map')))
        self._tick_i = 0
        self._pending = {}            # newest RAW msg per camera, processed in _update
        self._latest = {}             # newest downsampled cloud per camera
        self._rng = np.random.default_rng(0)
        self.gng = GNG(dim=3, task_dim=3, params=GNGParams(
            max_nodes=int(g('max_nodes')), lam=int(g('lam'))))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PointCloud2 is a large (~1.5 MB) sensor stream: a RELIABLE reader
        # (the default from an int-depth QoS) stalls and delivers NOTHING for
        # these samples, so use SensorDataQoS (BEST_EFFORT) as sensor topics
        # require -- otherwise env_gng never sees a cloud and stays at 0 nodes.
        for ns in list(g('camera_namespaces')):
            self.create_subscription(PointCloud2, f'/{ns}/{self.suffix}',
                                     lambda m, n=ns: self._on_cloud(n, m),
                                     qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, '/topo_map/markers', 1)
        # Drive updates from the cloud callback, NOT a timer: this node's
        # TransformListener subscribes to /tf (~19 Hz here) and, together with
        # the cloud stream, keeps the single-threaded executor's wait-set always
        # ready, so a create_timer callback is starved and never fires (the GNG
        # was never stepped -> 0 nodes). The cloud callback provably runs, so we
        # throttle the update off it.
        self._update_period = 1.0 / max(float(g('update_hz')), 1.0)
        self._last_update = 0.0
        self.get_logger().info(f'env_gng up; max_nodes={self.gng.params.max_nodes}')

    def _on_cloud(self, ns, msg):
        # Stash the newest raw message, then run a throttled update. The heavy
        # per-cloud work and GNG stepping happen in _update (off this callback,
        # which provably fires); doing them inline every message would be
        # wasteful, and a timer to pace them gets starved by the /tf + cloud
        # traffic on the single-threaded executor (0 nodes ever built).
        self._pending[ns] = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_update >= self._update_period:
            self._last_update = now
            self._update()

    def _process_cloud(self, msg):
        pts = _cloud_xyz(msg)
        if len(pts) == 0:
            return pts
        z = pts[:, 2]
        pts = _voxel_downsample(pts[(z >= self.min_z) & (z <= self.max_z)], self.leaf)
        if len(pts):
            pts = _radius_outlier_removal(pts, self.outlier_r, self.outlier_min)
        if self.self_filter and len(pts):
            pts = self._apply_self_filter(pts)
        if self._bg_nodes is not None and len(pts):
            pts = self._subtract_static(pts)
        return pts

    def _load_static_bg(self, path):
        """Load the saved static GNG node positions, or None if disabled."""
        if not path:
            return None
        import os
        if not os.path.exists(path):
            self.get_logger().warn(
                f'static_map {path} not found -- running WITHOUT background '
                'subtraction (whole-scene live map); run map_topo_static first')
            return None
        g = GNG.load(path)
        self.get_logger().info(
            f'static background: {len(g.W)} nodes from {path}; '
            f'subtracting points within {self.bg_dist} m')
        return np.asarray(g.W, dtype=np.float64)

    def _subtract_static(self, pts):
        """Drop points near a static-background node (grid-approx of bg_dist)."""
        return pts[~_grid_near(pts, self._bg_nodes, self.bg_dist)]

    def _apply_self_filter(self, pts):
        # append the current arm pose, then filter against the recent swept path
        # (last N snapshots) so a moving arm is removed despite cloud/TF lag.
        self._arm_hist.append(
            arm_positions(self.tf_buffer, self.world_frame, self.arm_prefixes))
        return filter_by_positions(pts, list(self._arm_hist),
                                   self.arm_prefixes, self.filter_r, self.finger_r)

    def _pool(self):
        clouds = [c for c in self._latest.values() if len(c)]
        return np.vstack(clouds) if clouds else np.empty((0, 3))

    def _update(self):
        # process whichever cameras delivered a new cloud since the last update
        for ns, msg in list(self._pending.items()):
            self._latest[ns] = self._process_cloud(msg)
            self._pending.pop(ns, None)
        pool = self._pool()
        if len(pool) >= 2:
            if len(self.gng.W) == 0:
                self.gng.init_two(pool)
            for i in self._rng.choice(len(pool), size=min(self.batch, len(pool)),
                                      replace=False):
                self.gng.step(pool[i])
            self._tick_i += 1
            if self.prune_dist > 0 and self._tick_i % self.prune_every == 0:
                self._prune_stale(pool)
        self._publish()   # always publish (even empty) so RViz clears old markers

    def _prune_stale(self, pool):
        """Delete nodes floating > prune_dist from any input point (Meso Emin)."""
        if len(self.gng.W) <= 2:
            return
        far = np.where(~_grid_near(self.gng.W, pool, self.prune_dist))[0]
        if len(far):
            self.gng.remove_nodes(far)

    def _publish(self):
        # Always publish -- with 0 nodes we emit empty SPHERE_LIST/LINE_LIST
        # markers (same ns/id) so RViz and the collision map CLEAR any stale live
        # geometry when the scene has no dynamic content (all subtracted).
        W = self.gng.W
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
            if self.max_edge > 0 and np.linalg.norm(W[i] - W[j]) > self.max_edge:
                continue          # skip bridge edges spanning empty space
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
