"""Classify each detected object as reachable or not against the GNG cloud(s).

Subscribes /detected_objects (PoseArray, frame `world`); for every object it
finds the nearest node in each arm's GNG reachability map (gng.py). An object is
"reachable" by an arm if it lies within that arm's (density-adaptive) reach
radius AND is enclosed by the arm's nodes (so a point dangling just outside the
map boundary is rejected, not ballooned in by the radius). Publishes coloured
markers (blue = reachable by some arm, red =
out of reach) + a text label with the best-arm distance and the manipulability
(capability) at the nearest node, and logs a per-object report.

    /reachability/markers   visualization_msgs/MarkerArray  (frame `world`)

    ros2 run reachability_gng reachability_check
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG


class ArmMap:
    """A loaded per-arm GNG map + its node manipulability stats."""

    def __init__(self, name, model_path, radius_factor=1.0, radius_abs=0.0,
                 enclose_thresh=0.5, enclose_k=8):
        self.name = name
        gng = GNG.load(model_path)
        self.W3 = np.asarray(gng.W[:, :3], dtype=float)   # node xyz in `world`
        base = model_path[:-4] if model_path.endswith('.npz') else model_path
        try:
            self.manip = np.load(base + '_stats.npz')['manip']
        except (OSError, KeyError):
            self.manip = None
        # Density-adaptive reach radius: the map's node spacing is set by the GNG
        # `lam` (a node every `lam` samples), so a FIXED radius makes the reach %
        # depend on map resolution, not true reachability. Scale the radius to the
        # median nearest-neighbour distance between nodes so a sparser map (larger
        # `lam`) just uses a proportionally larger radius. `radius_abs > 0`
        # overrides with an absolute metric radius (legacy behaviour).
        self.spacing = self._node_spacing()
        self.reach_radius = (radius_abs if radius_abs > 0.0
                             else radius_factor * self.spacing)
        # Boundary-aware "enclosure" gate: a radius alone balloons the reachable
        # region OUTWARD by `reach_radius`, so on a sparse map an out-of-reach
        # object near the boundary gets a node within radius and is wrongly green.
        # An INTERIOR point has nodes all around it (unit vectors to its nearest
        # nodes cancel, |mean| ~ 0); a point dangling OUTSIDE has nodes only on one
        # side (|mean| ~ 1). Require |mean unit-vector| <= enclose_thresh so only
        # genuinely-surrounded points pass. enclose_thresh >= 1 disables the gate.
        self.enclose_thresh = float(enclose_thresh)
        self.enclose_k = int(enclose_k)

    def _node_spacing(self, chunk=1024):
        """Median distance from each node to its nearest other node (m)."""
        W = self.W3
        n = len(W)
        if n < 2:
            return 0.0
        nn = np.empty(n)
        for s in range(0, n, chunk):
            blk = W[s:s + chunk]
            diff = blk[:, None, :] - W[None, :, :]
            d2 = np.einsum('nmk,nmk->nm', diff, diff)
            rows = np.arange(blk.shape[0])
            d2[rows, s + rows] = np.inf            # exclude self
            nn[s:s + blk.shape[0]] = np.sqrt(d2.min(axis=1))
        return float(np.median(nn))

    def query(self, p):
        """Return (distance to nearest node, manipulability at that node)."""
        d = self.W3 - p
        i = int(np.argmin(np.einsum('ij,ij->i', d, d)))
        dist = float(np.linalg.norm(self.W3[i] - p))
        manip = float(self.manip[i]) if self.manip is not None else float('nan')
        return dist, manip

    def dist_to_nearest(self, pts, chunk=4000):
        """Vectorised distance from each of pts (N,3) to its nearest node (N,)."""
        out = np.empty(len(pts))
        for s in range(0, len(pts), chunk):
            blk = pts[s:s + chunk]
            diff = blk[:, None, :] - self.W3[None, :, :]
            out[s:s + chunk] = np.sqrt(np.einsum('nmk,nmk->nm', diff, diff).min(axis=1))
        return out

    def reach_mask(self, pts, chunk=2048):
        """Boolean reachable mask for pts (N,3): within `reach_radius` of the
        nearest node AND enclosed by nodes (enclosure metric <= enclose_thresh)."""
        pts = np.asarray(pts, dtype=float)
        W = self.W3
        k = min(self.enclose_k, len(W))
        out = np.zeros(len(pts), dtype=bool)
        for s in range(0, len(pts), chunk):
            blk = pts[s:s + chunk]                          # (B,3)
            vec = W[None, :, :] - blk[:, None, :]           # (B,N,3) node - point
            d2 = np.einsum('bnk,bnk->bn', vec, vec)         # (B,N)
            idx = np.argpartition(d2, k - 1, axis=1)[:, :k]  # (B,k) nearest nodes
            rows = np.arange(blk.shape[0])[:, None]
            kd = np.sqrt(d2[rows, idx])                     # (B,k) distances
            within = kd[:, 0] <= self.reach_radius          # nearest within radius
            units = vec[rows, idx] / np.maximum(kd[..., None], 1e-9)
            enclosed = np.linalg.norm(units.mean(axis=1), axis=1) <= self.enclose_thresh
            out[s:s + blk.shape[0]] = within & enclosed
        return out


class ReachabilityCheck(Node):
    def __init__(self):
        super().__init__('reachability_check')
        self.declare_parameter('arm_models',
                               ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz'])
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        # reach_radius <= 0 -> density-adaptive (reach_radius_factor * node
        # spacing) so the result is independent of the GNG `lam`; > 0 -> absolute
        # metric radius (legacy).
        self.declare_parameter('reach_radius', 0.0)
        self.declare_parameter('reach_radius_factor', 1.0)
        # enclosure gate: 0..1, lower = stricter (rejects boundary bleed); >=1 off
        self.declare_parameter('enclose_thresh', 0.5)
        self.declare_parameter('enclose_k', 8)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('object_size', 0.05)   # cube edge (m) for markers
        # Per-object REACH/OUT report to the terminal, once per callback. Off by
        # default (it prints continuously); the RViz markers still show it. Turn
        # on with -p log_reach:=true when you need the numbers.
        self.declare_parameter('log_reach', False)

        models = list(self.get_parameter('arm_models').value)
        names = list(self.get_parameter('arm_names').value)
        radius_abs = float(self.get_parameter('reach_radius').value)
        radius_factor = float(self.get_parameter('reach_radius_factor').value)
        enclose_thresh = float(self.get_parameter('enclose_thresh').value)
        enclose_k = int(self.get_parameter('enclose_k').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.object_size = float(self.get_parameter('object_size').value)
        self.log_reach = bool(self.get_parameter('log_reach').value)

        self.arms = []
        for nm, mp in zip(names, models):
            try:
                arm = ArmMap(nm, mp, radius_factor=radius_factor,
                             radius_abs=radius_abs, enclose_thresh=enclose_thresh,
                             enclose_k=enclose_k)
            except OSError:
                self.get_logger().error(f'could not load {nm} model {mp}')
                continue
            self.arms.append(arm)
            self.get_logger().info(
                f'loaded {nm}: {mp} ({len(arm.W3)} nodes, '
                f'spacing={arm.spacing:.3f} m, reach_radius={arm.reach_radius:.3f} m)')
        if not self.arms:
            self.get_logger().error('no GNG maps loaded; nothing to classify')

        self.pub = self.create_publisher(MarkerArray, '/reachability/markers', 1)
        self.create_subscription(PoseArray, '/detected_objects', self._on_objects, 1)
        mode = 'absolute' if radius_abs > 0.0 else f'adaptive (x{radius_factor})'
        self.get_logger().info(
            f'reachability_check up; reach_radius={mode}, '
            f'enclose_thresh={enclose_thresh}')

    def _on_objects(self, msg):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        report = []
        for i, pose in enumerate(msg.poses):
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            per_arm = [(a.name, *a.query(p)) for a in self.arms]
            reach_by = [(a.name, d, m) for a, (_, d, m) in zip(self.arms, per_arm)
                        if a.reach_mask(p[None, :])[0]]
            reachable = bool(reach_by)
            # blue (not green) for reachable so it isn't confused with the
            # green obstacle/octomap voxels in the same scene; red = unreachable.
            color = (ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9) if reachable
                     else ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9))

            box = Marker()
            box.header.frame_id = self.world_frame
            box.ns = 'reach'
            box.id = i
            box.type = Marker.CUBE   # match the object's box shape, not a sphere
            box.action = Marker.ADD
            box.pose = pose
            box.scale.x = box.scale.y = box.scale.z = self.object_size
            box.color = color
            ma.markers.append(box)

            if reachable:
                best = min(reach_by, key=lambda t: t[1])
                txt = ('REACH ' + ','.join(nm for nm, _, _ in reach_by)
                       + f'\nd={best[1]:.3f} m={best[2]:.3f}')
            else:
                best = min(per_arm, key=lambda t: t[1])
                txt = f'OUT (nearest {best[0]} d={best[1]:.3f})'
            text = Marker()
            text.header.frame_id = self.world_frame
            text.ns = 'reach_label'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.10)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = txt
            ma.markers.append(text)

            report.append(
                f'obj{i} {"REACH" if reachable else "OUT"}: '
                + ' '.join(f'{nm}:{d:.3f}' for nm, d, _ in per_arm))

        self.pub.publish(ma)
        if report and self.log_reach:
            self.get_logger().info('; '.join(report))


def main():
    rclpy.init()
    node = ReachabilityCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
