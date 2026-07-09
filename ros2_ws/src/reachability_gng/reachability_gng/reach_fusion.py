"""Collision-free fusion of the reach-map (action) and env-map (perception).

Sensei's Meso adjacency-matrix method run ON each arm's reach graph (the free
config-space graph, world frame). For a chosen target object:
  * carve  : env nodes within target_radius of the target are not obstacles.
  * danger : reach nodes within collision_radius of a remaining obstacle.
  * S      : diffusion sum_l gamma^l A_hat^l (Meso sumMat), precomputed per arm.
  * cfree  : norm(S@target) - norm(S@danger) > 0 = collision-free corridor.
Target = /detected_objects centroid, selected by obj_N (ground truth, via
ObjnLocalizer), class substring, or index; switch live on /reach_fusion/set_target.
Publishes /reach_fusion/markers: per arm armN_{free,danger,cfree,edges} + target.
"""
from __future__ import annotations

import re

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG
from reachability_gng.objn_localizer import ObjnLocalizer

_OBJN_RE = re.compile(r'^obj_\d+$')
# green is reserved for the env topo_map; red=danger, cyan=collision-free.
_ARM_COLORS = {'arm1': (0.95, 0.95, 0.95), 'arm2': (0.15, 0.45, 1.0),
               'arm3': (0.98, 0.70, 0.10), 'arm4': (0.85, 0.25, 0.90)}


def _diffusion_matrix(n, edges, gamma, levels):
    """S = sum_{l=1..levels} gamma^l A_hat^l (row-normalised adjacency, Meso sumMat)."""
    A = np.zeros((n, n))
    for i, j in edges:
        A[i, j] = A[j, i] = 1.0
    deg = A.sum(1)
    deg[deg == 0] = 1.0
    A_hat = A / deg[:, None]
    S, P = np.zeros((n, n)), np.eye(n)
    for l in range(1, levels + 1):
        P = A_hat @ P
        S += (gamma ** l) * P
    return S


class ReachFusion(Node):
    def __init__(self):
        super().__init__('reach_fusion')
        p = self.declare_parameter
        p('arm_models', ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz',
                         '/tmp/arm3_model.npz', '/tmp/arm4_model.npz'])
        p('arm_labels', ['arm1', 'arm2', 'arm3', 'arm4'])
        p('env_markers_topic', '/topo_map/markers')
        p('objects_topic', '/detected_objects')
        p('world_frame', 'world')
        p('collision_radius', 0.15)   # reach node this close to an obstacle = danger
        p('target_radius', 0.15)      # env nodes this close to target = carved out
        p('target_label', '')         # obj_N / class substring; '' -> target_index
        p('target_index', 0)
        p('reach_tol', 0.20)          # arm reaches target if nearest node this close
        p('diffusion_gamma', 0.5)
        p('diffusion_levels', 4)
        p('reach_max_z', 2.05)        # drop reach nodes above the ceiling gantry
        p('publish_hz', 2.0)
        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.coll_r = float(g('collision_radius'))
        self.target_r = float(g('target_radius'))
        self.target_label = str(g('target_label')).strip().lower()
        self.target_index = int(g('target_index'))
        self.reach_tol = float(g('reach_tol'))
        max_z = float(g('reach_max_z'))
        gamma, levels = float(g('diffusion_gamma')), int(g('diffusion_levels'))

        self.arms = []                # (label, R, edges, S) per arm
        for lab, path in zip(list(g('arm_labels')), list(g('arm_models'))):
            try:
                gng = GNG.load(path)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{lab}: cannot load {path}: {e}')
                continue
            R = gng.W[:, :gng.task_dim].astype(np.float64)
            R, edges = self._crop_z(R, [tuple(e) for e in gng._edges], max_z)
            S = _diffusion_matrix(len(R), edges, gamma, levels)
            self.arms.append((lab, R, edges, S))
            self.get_logger().info(f'{lab}: {len(R)} nodes, {len(edges)} edges')

        self.env_pts = np.empty((0, 3))
        self.poses = np.empty((0, 3))
        self.labels = []              # [(label_lower, marker_xyz)] class-label path
        self.objn = ObjnLocalizer(self, world_frame=self.world_frame)
        obj_topic = g('objects_topic')
        self.create_subscription(MarkerArray, g('env_markers_topic'), self._on_env, 1)
        self.create_subscription(PoseArray, obj_topic, self._on_objects, 1)
        self.create_subscription(MarkerArray, obj_topic + '/markers',
                                 self._on_obj_markers, 1)
        self.create_subscription(String, '/reach_fusion/set_target',
                                 self._on_set_target, 1)
        self.pub = self.create_publisher(MarkerArray, '/reach_fusion/markers', 1)
        self.create_timer(1.0 / max(float(g('publish_hz')), 0.5), self._tick)

    @staticmethod
    def _crop_z(R, edges, max_z):
        keep = R[:, 2] <= max_z
        if keep.all():
            return R, edges
        remap = {int(o): i for i, o in enumerate(np.where(keep)[0])}
        edges = [(remap[i], remap[j]) for i, j in edges
                 if i in remap and j in remap]
        return R[keep], edges

    def _on_env(self, msg):
        if msg.markers:
            self.env_pts = np.array([[p.x, p.y, p.z] for p in msg.markers[0].points])

    def _on_objects(self, msg):
        self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                               for p in msg.poses]) if msg.poses else np.empty((0, 3))

    def _on_obj_markers(self, msg):
        lab = [(m.text.strip().lower(),
                np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))
               for m in msg.markers if m.text]
        if lab:
            self.labels = lab

    def _on_set_target(self, msg):
        v = msg.data.strip()
        if v.lstrip('-').isdigit():
            self.target_index, self.target_label = int(v), ''
        else:
            self.target_label = v.lower()
        self.get_logger().info(f'target -> {v}')

    def _resolve_target(self):
        """Target centroid by obj_N (ground truth) / class substring / index."""
        if len(self.poses) == 0:
            return None
        lbl = self.target_label
        if lbl:
            if _OBJN_RE.match(lbl):
                return self.objn.find(lbl, self.poses)
            for lab, mxyz in self.labels:      # class substring (seg -- unreliable)
                if lbl in lab:
                    return self.poses[int(np.argmin(
                        np.linalg.norm(self.poses - mxyz, axis=1)))]
            return None
        return (self.poses[self.target_index]
                if 0 <= self.target_index < len(self.poses) else None)

    def _classify(self, R, S, target):
        """(danger, cfree) masks: danger near obstacles, cfree = corridor to target."""
        from scipy.spatial import cKDTree
        n = len(R)
        if len(self.env_pts) == 0:
            return np.zeros(n, bool), np.zeros(n, bool)
        obst = (self.env_pts[np.linalg.norm(self.env_pts - target, axis=1) > self.target_r]
                if target is not None else self.env_pts)
        danger = (cKDTree(obst).query(R)[0] < self.coll_r
                  if len(obst) else np.zeros(n, bool))
        cfree = np.zeros(n, bool)
        if target is not None:
            dR = np.linalg.norm(R - target, axis=1)
            it = int(np.argmin(dR))
            if dR[it] <= self.reach_tol:
                tgt = np.zeros(n); tgt[it] = 1.0
                pot, dang = S @ tgt, S @ danger.astype(float)
                pot = pot / pot.max() if pot.max() > 0 else pot
                dang = dang / dang.max() if dang.max() > 0 else dang
                cfree = (pot - dang > 0.0) & ~danger
        return danger, cfree

    def _sphere_list(self, lab, ns, mid, size, color, now):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.world_frame, now
        m.ns, m.id, m.type, m.action = f'{lab}_{ns}', mid, Marker.SPHERE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = color
        return m

    def _tick(self):
        if not self.arms:
            return
        markers, mid, now = [], 0, self.get_clock().now().to_msg()
        target = self._resolve_target()
        for lab, R, edges, S in self.arms:
            danger, cfree = self._classify(R, S, target)
            cr, cg, cb = _ARM_COLORS.get(lab, (0.6, 0.6, 0.6))
            free = self._sphere_list(lab, 'free', mid, 0.022,
                                     ColorRGBA(r=cr, g=cg, b=cb, a=1.0), now)
            dang = self._sphere_list(lab, 'danger', mid + 1, 0.03,
                                     ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0), now)
            cfr = self._sphere_list(lab, 'cfree', mid + 2, 0.032,
                                    ColorRGBA(r=0.1, g=0.95, b=0.95, a=1.0), now)
            for i, w in enumerate(R):
                pt = Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                (dang if danger[i] else cfr if cfree[i] else free).points.append(pt)
            net = self._sphere_list(lab, 'edges', mid + 3, 0.004,
                                    ColorRGBA(r=cr, g=cg, b=cb, a=0.3), now)
            net.type, net.scale.x = Marker.LINE_LIST, 0.004
            for i, j in edges:
                net.points += [Point(x=float(R[i][0]), y=float(R[i][1]), z=float(R[i][2])),
                               Point(x=float(R[j][0]), y=float(R[j][1]), z=float(R[j][2]))]
            markers += [free, dang, cfr, net]
            mid += 4
        if target is not None:
            t = Marker()
            t.header.frame_id, t.header.stamp = self.world_frame, now
            t.ns, t.id, t.type, t.action = 'target_obj', mid, Marker.SPHERE, Marker.ADD
            t.scale.x = t.scale.y = t.scale.z = 0.10
            t.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            t.pose.position = Point(x=float(target[0]), y=float(target[1]),
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
