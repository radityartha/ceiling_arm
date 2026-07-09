"""Collision-free fusion of the reach-map (action) and env-map (perception).

Phase 2 of sensei's perceiving-acting method: fuse each arm's REACH-MAP GNG
(the free configuration graph, action map, world-frame [xyz|q]) with the
ENV-MAP GNG obstacle nodes (perception map from env_gng) to find the
collision-free reachable region toward a target -- the Meso adjacency-matrix
approach (potential_adMat / collision_adMat) run ON the reach graph, which is the
arm analog of Meso's buggy navigation graph.

2a (done): danger-source marking -- reach nodes near an env obstacle = danger.
2b (this): + TARGET CARVING and MULTI-SCALE PROPAGATION on the reach graph:
  * carve: env nodes within `target_radius` of the chosen target object are NOT
    obstacles, so reach nodes near the target are allowed (the arm may touch it).
  * diffusion S = sum_l gamma^l * A_hat^l  (row-normalised adjacency powers) is
    the multi-scale reachability of the reach graph (Meso sumMat), precomputed.
  * potential  = S @ (target node one-hot)      -> attraction toward the target
    danger     = S @ (obstacle-adjacent seed)   -> spread of collision danger
    free_score = norm(potential) - norm(danger) -> Meso NodesC; >0 = the
    collision-free corridor reachable toward the target.

Both maps are in the `world` frame, so no transform is needed.

Output (RViz MarkerArray, frame `world`), per arm `armN_`:
    _danger  red spheres      (near a non-target obstacle)
    _cfree   cyan spheres     (collision-free, reachable toward the target)
    _free    arm-colour dots  (reachable, not on the corridor)
    _edges   arm-colour lines (reach graph)
  + target_obj  big yellow sphere at the chosen target object
"""

from __future__ import annotations

import json
import re

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG
from reachability_gng.object_localizer import quat_to_R

_OBJN_RE = re.compile(r'^obj_\d+$')

# distinct free-node colour per arm so the four reach graphs are separable in
# RViz. green is reserved for the env topo_map; red=danger, cyan=collision-free.
_ARM_COLORS = {
    'arm1': (0.95, 0.95, 0.95),   # white
    'arm2': (0.15, 0.45, 1.00),   # blue
    'arm3': (0.98, 0.70, 0.10),   # amber
    'arm4': (0.85, 0.25, 0.90),   # magenta
}
_FALLBACK_COLOR = (0.6, 0.6, 0.6)


def _diffusion_matrix(n, edges, gamma, levels):
    """S = sum_{l=1..levels} gamma^l * A_hat^l  (row-normalised adjacency).

    Multi-scale reachability of the graph (Meso sumMat): S[i, j] is how strongly
    node j's influence diffuses to node i over up to `levels` hops, decayed by
    gamma each hop. Precomputed once per arm (graph is static)."""
    A = np.zeros((n, n))
    for i, j in edges:
        A[i, j] = 1.0
        A[j, i] = 1.0
    deg = A.sum(1)
    deg[deg == 0] = 1.0
    A_hat = A / deg[:, None]
    S = np.zeros((n, n))
    P = np.eye(n)
    for l in range(1, levels + 1):
        P = A_hat @ P
        S += (gamma ** l) * P
    return S


class ReachFusion(Node):
    def __init__(self):
        super().__init__('reach_fusion')
        self.declare_parameter('arm_models',
                               ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz',
                                '/tmp/arm3_model.npz', '/tmp/arm4_model.npz'])
        self.declare_parameter('arm_labels', ['arm1', 'arm2', 'arm3', 'arm4'])
        self.declare_parameter('env_markers_topic', '/topo_map/markers')
        self.declare_parameter('objects_topic', '/detected_objects')
        self.declare_parameter('world_frame', 'world')
        # a reach node within this of a (non-target) env node = in collision
        self.declare_parameter('collision_radius', 0.15)
        # env nodes within this of the target object are carved out (allowed)
        self.declare_parameter('target_radius', 0.15)
        # target selection: by LABEL (preferred, stable) -- either the Isaac prim
        # name "obj_N" (ground-truth identity, resolved via instance_id) or a
        # semantic class substring e.g. "banana"; empty -> positional
        # target_index into /detected_objects (fragile: it is object_localizer's
        # temporal track order, NOT Isaac's obj_N numbering).
        self.declare_parameter('target_label', '')
        self.declare_parameter('target_index', 0)
        # "obj_N" targets are resolved by REVERSE-PROJECTING each detected
        # centroid into the raw Isaac instance-segmentation image and reading the
        # instance_id there -> obj_N (id2objn is stable; seg_router's class names
        # are NOT, so we bypass them entirely). Needs the raw seg image +
        # camera_info per camera + the world->optical TFs.
        self.declare_parameter('isaac_labels_topic',
                               '/rgbd/instance_segmentation_labels')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('seg_image_suffix', 'instance_segmentation')
        # arm can reach the target only if its nearest reach node is this close
        self.declare_parameter('reach_tol', 0.20)
        # multi-scale diffusion (Meso sumMat) knobs
        self.declare_parameter('diffusion_gamma', 0.5)
        self.declare_parameter('diffusion_levels', 4)
        # drop reach nodes above the ceiling gantry (unreachable, blocked by roof)
        self.declare_parameter('reach_max_z', 2.05)
        self.declare_parameter('publish_hz', 2.0)

        self.world_frame = self.get_parameter('world_frame').value
        self.coll_r = float(self.get_parameter('collision_radius').value)
        self.target_r = float(self.get_parameter('target_radius').value)
        self.target_label = str(self.get_parameter('target_label').value).strip().lower()
        self.target_index = int(self.get_parameter('target_index').value)
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        gamma = float(self.get_parameter('diffusion_gamma').value)
        levels = int(self.get_parameter('diffusion_levels').value)
        self.reach_max_z = float(self.get_parameter('reach_max_z').value)
        labels = list(self.get_parameter('arm_labels').value)
        paths = list(self.get_parameter('arm_models').value)

        # load each arm's reach graph: node xyz + edges + diffusion matrix S
        self.arms = []
        for lab, path in zip(labels, paths):
            try:
                g = GNG.load(path)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{lab}: cannot load {path}: {e}')
                continue
            R = g.W[:, :g.task_dim].astype(np.float64)
            edges = [tuple(e) for e in g._edges]
            R, edges = self._crop_z(R, edges, self.reach_max_z)
            S = _diffusion_matrix(len(R), edges, gamma, levels)
            self.arms.append((lab, R, edges, S))
            self.get_logger().info(f'{lab}: reach graph {len(R)} nodes, '
                                   f'{len(edges)} edges, diffusion S ready')

        self.env_pts = np.empty((0, 3))   # latest env obstacle node positions
        self.poses = np.empty((0, 3))     # all /detected_objects centroids (world)
        self.labels = []                  # [(label_lower, marker_xyz)] from markers
        self.id2objn = {}                 # instance_id(str) -> "obj_N" (raw Isaac)
        self.optical_suffix = self.get_parameter('optical_frame_suffix').value
        self.cams = list(self.get_parameter('camera_namespaces').value)
        self.K = {}                       # ns -> (fx, fy, cx, cy)
        self.seg = {}                     # ns -> int32 instance-seg image
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            MarkerArray, self.get_parameter('env_markers_topic').value,
            self._on_env, 1)
        self.create_subscription(
            PoseArray, self.get_parameter('objects_topic').value,
            self._on_objects, 1)
        # label-carrying markers (TEXT) -> resolve target by name
        self.create_subscription(
            MarkerArray, self.get_parameter('objects_topic').value + '/markers',
            self._on_obj_markers, 1)
        # raw instance_id -> obj_N + per-camera seg image & intrinsics, for
        # reverse-projection resolution of "obj_N" targets.
        self.create_subscription(
            String, self.get_parameter('isaac_labels_topic').value,
            self._on_isaac_labels, 1)
        # runtime target switch: `ros2 topic pub -1 /reach_fusion/set_target
        # std_msgs/String "{data: obj_4}"` (or a class substring, or an integer
        # index) -- no need to restart the node.
        self.create_subscription(String, '/reach_fusion/set_target',
                                 self._on_set_target, 1)
        seg_suffix = self.get_parameter('seg_image_suffix').value
        for ns in self.cams:
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, n=ns: self.K.__setitem__(
                    n, (m.k[0], m.k[4], m.k[2], m.k[5])), 1)
            self.create_subscription(
                Image, f'/{ns}/{seg_suffix}',
                lambda m, n=ns: self.seg.__setitem__(n, self._decode_seg(m)), 1)
        self.pub = self.create_publisher(MarkerArray, '/reach_fusion/markers', 1)
        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(hz, 0.5), self._tick)

    @staticmethod
    def _crop_z(R, edges, max_z):
        """Keep reach nodes with z <= max_z; remap surviving edges."""
        keep = R[:, 2] <= max_z
        if keep.all():
            return R, edges
        remap = {}
        for new, old in enumerate(np.where(keep)[0]):
            remap[int(old)] = new
        R2 = R[keep]
        edges2 = [(remap[i], remap[j]) for i, j in edges
                  if i in remap and j in remap]
        return R2, edges2

    def _on_env(self, msg: MarkerArray):
        if msg.markers:
            self.env_pts = np.array([[p.x, p.y, p.z]
                                     for p in msg.markers[0].points])

    def _on_objects(self, msg: PoseArray):
        self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                               for p in msg.poses]) if msg.poses \
            else np.empty((0, 3))

    def _on_obj_markers(self, msg: MarkerArray):
        labels = []
        for m in msg.markers:
            if m.text:  # TEXT marker carries the label (offset above the object)
                p = m.pose.position
                labels.append((m.text.strip().lower(),
                               np.array([p.x, p.y, p.z])))
        if labels:
            self.labels = labels

    def _on_set_target(self, msg: String):
        """Switch the grasp target at runtime. Value is obj_N / a class substring
        / an integer index."""
        v = msg.data.strip()
        if v.lstrip('-').isdigit():
            self.target_index = int(v)
            self.target_label = ''
            self.get_logger().info(f'target -> index {self.target_index}')
        else:
            self.target_label = v.lower()
            self.get_logger().info(f'target -> label "{self.target_label}"')

    def _on_isaac_labels(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        # instance_id -> obj_N is STABLE across frames; merge is safe & robust to
        # per-frame drops (skip BACKGROUND / non-obj / dict values).
        self.id2objn.update({k: v.rstrip('/').split('/')[-1]
                             for k, v in d.items()
                             if isinstance(v, str) and 'obj_' in v})

    @staticmethod
    def _decode_seg(msg: Image):
        a = np.frombuffer(bytes(msg.data), dtype=np.int32)
        return a.reshape(msg.height, msg.step // 4)[:, :msg.width]

    def _instance_at(self, ns, P):
        """Instance id under world point P in camera `ns`'s seg image, or None."""
        if ns not in self.K or ns not in self.seg:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                f'{ns}{self.optical_suffix}', 'world',
                rclpy.time.Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.translation, tf.rotation
        pc = quat_to_R(q.x, q.y, q.z, q.w) @ P + np.array([t.x, t.y, t.z])
        if pc[2] <= 0.05:
            return None
        fx, fy, cx, cy = self.K[ns]
        u = int(fx * pc[0] / pc[2] + cx)
        v = int(fy * pc[1] / pc[2] + cy)
        seg = self.seg[ns]
        if 0 <= v < seg.shape[0] and 0 <= u < seg.shape[1]:
            return int(seg[v, u])
        return None

    def _resolve_objn(self, objn):
        """Centroid of Isaac prim `objn` via reverse projection (ground truth,
        immune to seg_router's unstable class naming)."""
        for P in self.poses:
            for ns in self.cams:
                iid = self._instance_at(ns, P)
                if iid is not None and self.id2objn.get(str(iid)) == objn:
                    return P
        return None

    def _resolve_target(self):
        """Target = /detected_objects CENTROID. target_label may be an Isaac prim
        name "obj_N" (resolved by reverse projection -> ground truth) or a class
        substring (uses seg_router labels -- UNRELIABLE, they swap); else fall
        back to target_index. Class substring -> marker label -> nearest centroid
        (text marker is offset ~8 cm above the true centroid)."""
        if len(self.poses) == 0:
            return None
        label = self.target_label
        if label:
            if _OBJN_RE.match(label):
                return self._resolve_objn(label)
            for lab, mxyz in self.labels:
                if label in lab:
                    j = int(np.argmin(np.linalg.norm(self.poses - mxyz, axis=1)))
                    return self.poses[j]
            return None  # named target not currently detected
        if 0 <= self.target_index < len(self.poses):
            return self.poses[self.target_index]
        return None

    def _classify(self, R, S, target):
        """Return (danger, cfree) boolean masks for one arm.

        danger: reach node within coll_r of a NON-target obstacle env node.
        cfree : collision-free corridor reachable toward the target (free_score>0)
                -- only when the arm can actually reach the target."""
        from scipy.spatial import cKDTree
        n = len(R)
        if len(self.env_pts) == 0:
            return np.zeros(n, bool), np.zeros(n, bool)
        # carve: obstacles are env nodes NOT belonging to the target object
        if target is not None:
            dT = np.linalg.norm(self.env_pts - target, axis=1)
            obst = self.env_pts[dT > self.target_r]
        else:
            obst = self.env_pts
        danger = (cKDTree(obst).query(R)[0] < self.coll_r
                  if len(obst) else np.zeros(n, bool))

        cfree = np.zeros(n, bool)
        if target is not None:
            dR = np.linalg.norm(R - target, axis=1)
            it = int(np.argmin(dR))
            if dR[it] <= self.reach_tol:            # arm can reach the target
                tgt = np.zeros(n); tgt[it] = 1.0
                pot = S @ tgt
                dang = S @ danger.astype(float)
                pot = pot / pot.max() if pot.max() > 0 else pot
                dang = dang / dang.max() if dang.max() > 0 else dang
                cfree = (pot - dang > 0.0) & ~danger
        return danger, cfree

    def _tick(self):
        if not self.arms:
            return
        markers = []
        mid = 0
        now = self.get_clock().now().to_msg()
        acolor = _ARM_COLORS
        target = self._resolve_target()
        for lab, R, edges, S in self.arms:
            danger, cfree = self._classify(R, S, target)
            cr, cg, cb = acolor.get(lab, _FALLBACK_COLOR)

            def _mk(ns, i, size, color):
                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp = now
                m.ns = f'{lab}_{ns}'
                m.id = i
                m.type = Marker.SPHERE_LIST
                m.action = Marker.ADD
                m.scale.x = m.scale.y = m.scale.z = size
                m.color = color
                return m

            free = _mk('free', mid, 0.022, ColorRGBA(r=cr, g=cg, b=cb, a=1.0))
            dang = _mk('danger', mid + 1, 0.03,
                       ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0))
            cfr = _mk('cfree', mid + 2, 0.032,
                      ColorRGBA(r=0.1, g=0.95, b=0.95, a=1.0))
            for i, w in enumerate(R):
                pt = Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                if danger[i]:
                    dang.points.append(pt)
                elif cfree[i]:
                    cfr.points.append(pt)
                else:
                    free.points.append(pt)

            net = Marker()
            net.header.frame_id = self.world_frame
            net.header.stamp = now
            net.ns = f'{lab}_edges'
            net.id = mid + 3
            net.type = Marker.LINE_LIST
            net.action = Marker.ADD
            net.scale.x = 0.004
            net.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.3)
            for i, j in edges:
                net.points.append(Point(x=float(R[i][0]), y=float(R[i][1]),
                                        z=float(R[i][2])))
                net.points.append(Point(x=float(R[j][0]), y=float(R[j][1]),
                                        z=float(R[j][2])))
            markers += [free, dang, cfr, net]
            mid += 4

        # target object marker (big yellow sphere)
        if target is not None:
            t = Marker()
            t.header.frame_id = self.world_frame
            t.header.stamp = now
            t.ns = 'target_obj'
            t.id = mid
            t.type = Marker.SPHERE
            t.action = Marker.ADD
            t.scale.x = t.scale.y = t.scale.z = 0.10
            t.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            t.pose.position = Point(x=float(target[0]),
                                    y=float(target[1]),
                                    z=float(target[2]))
            markers.append(t)

        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = ReachFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
