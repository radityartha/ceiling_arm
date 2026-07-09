"""Environment perception map as an online GNG topological map (Phase 1).

Sensei's (Kubota/Nando) perceiving-acting method represents the environment as a
GROWING NEURAL GAS topological map (nodes + edges) built from the RGBD point
cloud -- NOT as voxels/octomap. This node is the perception half of that: it
consumes the Isaac RGBD clouds (already deprojected to the `world` frame by
`seg_cloud`), voxel-downsamples them, and feeds points to an online GNG so the
node graph continuously tiles whatever surfaces the cameras see.

It reuses the project's tested `GNG` core (reachability_gng.gng) with a pure 3D
xyz vector (task_dim == dim == 3, no joint `q` part), and mini-batch online
stepping ("batch-learning" GNG in the thesis).

Output (visualize in RViz with a MarkerArray display, frame `world`):
    /topo_map/markers   visualization_msgs/MarkerArray   (green nodes + edges)

This is Phase 1 only: perception + visualization. No collision-free adjacency
fusion and no action coupling yet (those are later phases).
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

# Kinova Gen3 Lite link chain (parent->child order) used to build capsules for
# the robot self-filter, plus the finger links treated as extra sphere centres.
_ARM_CHAIN = ['shoulder_link', 'arm_link', 'forearm_link',
              'lower_wrist_link', 'upper_wrist_link', 'end_effector_link']
_FINGER_LINKS = ['left_finger_prox_link', 'left_finger_dist_link',
                 'right_finger_prox_link', 'right_finger_dist_link']


def _seg_dist2(P: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Squared distance from every point in P (N,3) to segment a-b."""
    ab = b - a
    denom = float(ab @ ab) + 1e-9
    t = np.clip(((P - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d = P - proj
    return np.einsum('ij,ij->i', d, d)


def _cloud_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract finite (x, y, z) rows from a PointCloud2 as an (N, 3) array.

    Same decode pattern as the other nodes in this package; `seg_cloud` already
    publishes in the `world` frame so no TF transform is needed here."""
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
    a = a[np.isfinite(a).all(axis=1)]
    return a.astype(np.float64)


def _voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    """Keep one point per `leaf`-sized voxel (grid-quantise + unique)."""
    if len(pts) == 0 or leaf <= 0.0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _radius_outlier_removal(pts: np.ndarray, radius: float,
                            min_neighbors: int) -> np.ndarray:
    """Drop sparse points (fewer than `min_neighbors` within `radius`).

    Removes depth-camera "flying pixels": points interpolated along an object's
    silhouette edge that hang in free space and form thin 1D chains bridging the
    foreground (e.g. a gripper) to the background surface. Real surfaces are
    dense (~16 neighbours in 5 cm here); flying pixels have <=2, so a neighbour
    count cleanly separates them."""
    if len(pts) == 0 or min_neighbors <= 0:
        return pts
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    # count includes the point itself, so require > min_neighbors
    counts = tree.query_ball_point(pts, radius, return_length=True)
    return pts[counts > min_neighbors]


class EnvGNG(Node):
    def __init__(self):
        super().__init__('env_gng')
        # cameras whose world-frame seg_cloud we consume
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('cloud_topic_suffix', 'seg_cloud')
        self.declare_parameter('world_frame', 'world')
        # workspace crop (world coords) -- drop floor/far returns before learning
        self.declare_parameter('min_z', 0.02)
        # 1.9 crops the ceiling gantry structure (platform z=2.05, rotation
        # z=2.01) out of the map -- it is above the workspace and not a pick
        # obstacle; the wall stays an obstacle below the cut. Arms above 1.9 are
        # self-filtered anyway.
        self.declare_parameter('max_z', 1.9)
        self.declare_parameter('leaf_size', 0.02)     # voxel downsample (m)
        # radius outlier removal: kill depth flying-pixel streaks (sparse points)
        self.declare_parameter('outlier_radius', 0.05)
        self.declare_parameter('outlier_min_neighbors', 3)
        # online mini-batch: N random downsampled points fed to GNG per tick
        self.declare_parameter('samples_per_tick', 800)
        self.declare_parameter('update_hz', 10.0)
        self.declare_parameter('max_nodes', 800)      # room-scale scene budget
        self.declare_parameter('lam', 100)            # insert a node every lam steps
        # adaptive node deletion: drop stale "bridge" nodes the data no longer
        # supports (nodes floating >prune_dist from any input point). Runs every
        # prune_every ticks. Fixes streaks that edge-aging alone never clears.
        self.declare_parameter('prune_dist', 0.10)
        self.declare_parameter('prune_every', 5)
        # robot self-filter: drop cloud points on the robot's own arms so the
        # moving body is not mapped as environment (MoveIt's filtered cloud is
        # throttled/dead, so we filter here via TF -> keeps env_gng standalone).
        self.declare_parameter('self_filter', True)
        self.declare_parameter('arm_prefixes',
                               ['t1_a1', 't1_a2', 't2_a1', 't2_a2'])
        self.declare_parameter('self_filter_radius', 0.07)   # arm-link capsules
        self.declare_parameter('finger_radius', 0.05)        # finger spheres

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('cloud_topic_suffix').value
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.leaf = float(self.get_parameter('leaf_size').value)
        self.outlier_r = float(self.get_parameter('outlier_radius').value)
        self.outlier_min = int(self.get_parameter('outlier_min_neighbors').value)
        self.batch = int(self.get_parameter('samples_per_tick').value)
        self.prune_dist = float(self.get_parameter('prune_dist').value)
        self.prune_every = int(self.get_parameter('prune_every').value)
        self._tick_i = 0
        hz = float(self.get_parameter('update_hz').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self.gng = GNG(dim=3, task_dim=3, params=GNGParams(
            max_nodes=int(self.get_parameter('max_nodes').value),
            lam=int(self.get_parameter('lam').value),
        ))
        self._rng = np.random.default_rng(0)
        # latest world-frame downsampled cloud per camera (dynamic scene: keep
        # only the newest reading so the map follows moved objects)
        self._latest = {}

        self.self_filter = bool(self.get_parameter('self_filter').value)
        self.arm_prefixes = list(self.get_parameter('arm_prefixes').value)
        self.filter_r = float(self.get_parameter('self_filter_radius').value)
        self.finger_r = float(self.get_parameter('finger_radius').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for ns in nss:
            topic = f'/{ns}/{self.suffix}'
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, n=ns: self._on_cloud(n, msg), 1)
        self.pub = self.create_publisher(MarkerArray, '/topo_map/markers', 1)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)
        self.get_logger().info(
            f'env_gng up; cams={nss} suffix={self.suffix} '
            f'leaf={self.leaf} batch={self.batch} max_nodes={self.gng.params.max_nodes}')

    def _on_cloud(self, ns: str, msg: PointCloud2):
        pts = _cloud_xyz(msg)
        if len(pts) == 0:
            return
        z = pts[:, 2]
        pts = pts[(z >= self.min_z) & (z <= self.max_z)]
        pts = _voxel_downsample(pts, self.leaf)
        if len(pts):
            pts = _radius_outlier_removal(pts, self.outlier_r, self.outlier_min)
        if self.self_filter and len(pts):
            pts = self._apply_self_filter(pts)
        self._latest[ns] = pts

    def _frame_pos(self, frame: str, cache: dict):
        """World position of a TF frame (cached per filter call); None if TF absent."""
        if frame in cache:
            return cache[frame]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, frame, rclpy.time.Time())
            t = tf.transform.translation
            cache[frame] = np.array([t.x, t.y, t.z])
        except (LookupException, ConnectivityException, ExtrapolationException):
            cache[frame] = None
        return cache[frame]

    def _apply_self_filter(self, pts: np.ndarray) -> np.ndarray:
        """Remove points lying on the robot's own arms (capsules + finger spheres)."""
        cache = {}
        keep = np.ones(len(pts), dtype=bool)
        r2 = self.filter_r * self.filter_r
        fr2 = self.finger_r * self.finger_r
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

    def _pool(self) -> np.ndarray:
        clouds = [c for c in self._latest.values() if len(c)]
        if not clouds:
            return np.empty((0, 3))
        return np.vstack(clouds)

    def _tick(self):
        pool = self._pool()
        if len(pool) < 2:
            return
        if len(self.gng.W) == 0:
            self.gng.init_two(pool)
        k = min(self.batch, len(pool))
        sel = self._rng.choice(len(pool), size=k, replace=False)
        for i in sel:
            self.gng.step(pool[i])
        self._tick_i += 1
        if self.prune_dist > 0 and self._tick_i % self.prune_every == 0:
            self._prune_stale(pool)
        self._publish()

    def _prune_stale(self, pool: np.ndarray):
        """Delete nodes floating farther than prune_dist from any input point."""
        if len(self.gng.W) <= 2 or len(pool) == 0:
            return
        from scipy.spatial import cKDTree
        d, _ = cKDTree(pool).query(self.gng.W)
        far = np.where(d > self.prune_dist)[0]
        if len(far):
            self.gng.remove_nodes(far)

    def _publish(self):
        W = self.gng.W
        if len(W) == 0:
            return
        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        now = self.get_clock().now().to_msg()

        nodes = Marker()
        nodes.header.frame_id = self.world_frame
        nodes.header.stamp = now
        nodes.ns = 'topo_nodes'
        nodes.id = 0
        nodes.type = Marker.SPHERE_LIST
        nodes.action = Marker.ADD
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.02
        nodes.color = green
        nodes.points = [Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                        for w in W]

        edges = Marker()
        edges.header.frame_id = self.world_frame
        edges.header.stamp = now
        edges.ns = 'topo_edges'
        edges.id = 1
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.scale.x = 0.005
        edges.color = green
        for e in self.gng._edges:
            i, j = tuple(e)
            edges.points.append(Point(x=float(W[i][0]), y=float(W[i][1]),
                                      z=float(W[i][2])))
            edges.points.append(Point(x=float(W[j][0]), y=float(W[j][1]),
                                      z=float(W[j][2])))

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
