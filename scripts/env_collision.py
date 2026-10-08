#!/usr/bin/env python3
"""Arm-vs-ENVIRONMENT collision screening against the frozen camera map (G33).

WHY THIS EXISTS. Until G33 nothing in this cell knew the environment: all four
move_group octomap updaters failed to load at every bring-up (the plugin package
is not installed), no camera ran in the bring-up, and every screen here
(interarm_collision S18, rail sweep S28, RNEA torque) is robot-vs-robot. G32-HW
R10 ev 8 passed all of them and arm_3 hit a rack (docs/p1_g32_hw.md B3).

The map is built ONCE from the two ceiling D455s with the robot standing still
(docs/results/p1_g33/build_env_map.py), robot points removed by `self_filter`
below, and frozen into `env_static_map.npz`. This module is the script-side
consumer; `env_static_map_pub` is the MoveIt-side one (same file, same padding).

STRICT by design (the G6/gng-collision lesson: a filter that silently no-ops is
worse than none): an unexpected set of robot geometries, a joint missing from a
configuration, or a missing map file RAISES -- it never degrades to "CLEAR".

Limits, stated:
  * discrete check at waypoints, like every other screen here;
  * convex hulls (true_hull) of the URDF meshes, which CONTAIN the meshes, so
    distances are slightly pessimistic -- the safe direction;
  * the map only holds what the cameras saw at capture time. Occluded regions
    and z > the crop top are NOT obstacles-free, they are UNKNOWN
    (docs/p1_g33_map.md lists them).

    python3 scripts/env_collision.py --self-test
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from interarm_collision import LIVE_URDF, _package_dirs, true_hull  # noqa: E402

# G34: 8 captures (rails 0..1.45), ONE fixed 4-DOF correction per camera
# (docs/p1_g34_rail_calib.md B2). G33's reg3 map stays at p1_g33/env_static_map.npz.
ENV_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs',
                       'results', 'p1_g34', 'env_static_map.npz')
ENV_MARGIN_M = 0.05     # = cross-camera calibration uncertainty (rgbd-extrinsic-calibration)
ARM_LINKS = ['base_link', 'shoulder_link', 'arm_link', 'forearm_link',
             'lower_wrist_link', 'upper_wrist_link', 'gripper_base_link',
             'left_finger_prox_link', 'left_finger_dist_link',
             'right_finger_prox_link', 'right_finger_dist_link']
STRUCT_LINKS = ['platform_link', 'rotation_link', 'mount_plate_left', 'mount_plate_right']
EXPECTED_GEOMS = sorted(
    [f'{g}_{a}_{l}_0' for g in ('t1', 't2') for a in ('a1', 'a2') for l in ARM_LINKS]
    + [f'{g}_{l}_0' for g in ('t1', 't2') for l in STRUCT_LINKS])
# every non-gripper joint: a configuration must give ALL of them (strict).
CONFIG_JOINTS = sorted(
    [f'{g}_{j}_joint' for g in ('t1', 't2') for j in ('linear', 'rotation')]
    + [f'{g}_{a}_joint_{k}' for g in ('t1', 't2') for a in ('a1', 'a2') for k in range(1, 7)])


class RobotGeom:
    """URDF collision geometry (true hulls) of the whole cell, posed by FK."""

    def __init__(self, urdf=LIVE_URDF):
        if not os.path.exists(urdf):
            raise FileNotFoundError(f'URDF {urdf} tidak ada -- TOLAK')
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(
            self.model, urdf, pin.GeometryType.COLLISION, _package_dirs())
        for g in self.geom.geometryObjects:
            if hasattr(g.geometry, 'buildConvexRepresentation'):
                g.geometry.buildConvexRepresentation(False)
                g.geometry = true_hull(g.geometry.convex)
            g.geometry.computeLocalAABB()
        self.names = [g.name for g in self.geom.geometryObjects]
        if sorted(self.names) != EXPECTED_GEOMS:
            miss = set(EXPECTED_GEOMS) - set(self.names)
            extra = set(self.names) - set(EXPECTED_GEOMS)
            raise ValueError(f'geometri robot != 52 yang diharapkan: hilang {sorted(miss)} '
                             f'lebih {sorted(extra)} -- TOLAK')
        self.geom_data = self.geom.createData()

    def q_from(self, joints):
        """Full q from a dict that MUST name every CONFIG_JOINTS joint."""
        miss = [j for j in CONFIG_JOINTS if j not in joints]
        if miss:
            raise KeyError(f'konfigurasi tanpa {miss} -- TOLAK')
        q = pin.neutral(self.model)
        for name, v in joints.items():
            jid = self.model.getJointId(name)
            if jid >= self.model.njoints:
                raise KeyError(f'sendi {name!r} tidak ada di URDF -- TOLAK')
            q[self.model.joints[jid].idx_q] = v
        return q

    def place(self, q):
        pin.updateGeometryPlacements(self.model, self.data, self.geom,
                                     self.geom_data, q)
        return self.geom_data.oMg

    def world_aabb(self, i, M):
        bb = self.geom.geometryObjects[i].geometry.aabb_local
        lo, hi = np.array(bb.min_), np.array(bb.max_)
        c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                      for z in (lo[2], hi[2])])
        w = c @ M.rotation.T + M.translation
        return w.min(0), w.max(0)


def point_distances(rg, q, pts, pad, only=None):
    """Distance from each point to the nearest robot geometry, computed only for
    points inside some geometry's AABB + pad (others get +inf). `only`: name
    prefixes to restrict to."""
    import coal
    oMg = rg.place(q)
    d = np.full(len(pts), np.inf)
    who = np.full(len(pts), -1)
    probe = coal.Sphere(1e-6)
    req = coal.DistanceRequest()
    for i, g in enumerate(rg.geom.geometryObjects):
        if only and not g.name.startswith(tuple(only)):
            continue
        M = oMg[i]
        lo, hi = rg.world_aabb(i, M)
        cand = np.nonzero(((pts >= lo - pad) & (pts <= hi + pad)).all(1))[0]
        Tg = coal.Transform3s(M.rotation, M.translation)
        for k in cand:
            res = coal.DistanceResult()
            dk = coal.distance(probe, coal.Transform3s(np.eye(3), pts[k]),
                               g.geometry, Tg, req, res)
            if dk < d[k]:
                d[k], who[k] = dk, i
    return d, who


def self_filter(rg, q, pts, pad):
    """Mask of points to KEEP: farther than `pad` from every robot geometry."""
    d, _ = point_distances(rg, q, pts, pad)
    return d > pad


def load_env_map(path=ENV_MAP):
    if not os.path.exists(path):
        raise FileNotFoundError(f'peta lingkungan {path} tidak ada -- TOLAK (jangan gerak)')
    m = np.load(path, allow_pickle=False)
    for k in ('centers', 'resolution', 'frame'):
        if k not in m.files:
            raise KeyError(f'{path}: kunci {k!r} hilang -- TOLAK')
    if str(m['frame']) != 'world' or len(m['centers']) == 0:
        raise ValueError(f'{path}: frame {m["frame"]} / {len(m["centers"])} voxel -- TOLAK')
    return m


class EnvChecker(RobotGeom):
    """Robot (all 52 geometries) vs the frozen environment map."""

    def __init__(self, urdf=LIVE_URDF, map_file=ENV_MAP, margin=ENV_MARGIN_M):
        import coal
        super().__init__(urdf)
        m = load_env_map(map_file)
        self.map_file = os.path.abspath(map_file)
        self.n_voxels = len(m['centers'])
        self.resolution = float(m['resolution'])
        self.octree = coal.makeOctree(m['centers'].astype(float), self.resolution)
        self.margin = margin
        self._I = coal.Transform3s()
        self._req = coal.DistanceRequest()

    def check(self, q, only=None):
        """(min_distance_m, worst_geometry_name). Negative = penetrating."""
        import coal
        oMg = self.place(q)
        best, worst = float('inf'), None
        for i, g in enumerate(self.geom.geometryObjects):
            if only and not g.name.startswith(tuple(only)):
                continue
            M = oMg[i]
            res = coal.DistanceResult()
            d = coal.distance(g.geometry, coal.Transform3s(M.rotation, M.translation),
                              self.octree, self._I, self._req, res)
            if d < best:
                best, worst = d, g.name
        return best, worst

    def screen_trajectory(self, joint_names, points, base_joints, only=None):
        """('CLEAR'|'MARGIN'|'COLLIDE', d_min, worst_geom, waypoint_index).
        base_joints must name every CONFIG_JOINTS joint (the held state);
        joint_names/points override it per waypoint. MARGIN is a REJECT on HW."""
        base = dict(base_joints)
        worst_d, worst_g, worst_k = float('inf'), None, -1
        for k, pt in enumerate(points):
            j = dict(base)
            j.update(zip(joint_names, pt))
            d, g = self.check(self.q_from(j), only)
            if d < worst_d:
                worst_d, worst_g, worst_k = d, g, k
        if worst_d <= 0.0:
            return 'COLLIDE', worst_d, worst_g, worst_k
        if worst_d < self.margin:
            return 'MARGIN', worst_d, worst_g, worst_k
        return 'CLEAR', worst_d, worst_g, worst_k


def self_test(urdf=LIVE_URDF):
    """A box placed ON arm_1's forearm must collide; the same box 1 m away must not;
    a missing joint and a missing map must raise."""
    import tempfile
    rg = RobotGeom(urdf)
    rest = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
    j = {n: 0.0 for n in CONFIG_JOINTS}
    for p in ('t1_a1', 't1_a2', 't2_a1', 't2_a2'):
        for k in range(6):
            j[f'{p}_joint_{k + 1}'] = rest[k]
    q = rg.q_from(j)
    i = rg.names.index('t1_a1_forearm_link_0')
    c = rg.place(q)[i].translation.copy()
    ok = True
    for name, off, want in (('on', 0.0, 'COLLIDE'), ('far', 1.0, 'CLEAR')):
        g = np.mgrid[-0.04:0.05:0.02, -0.04:0.05:0.02, -0.04:0.05:0.02].reshape(3, -1).T
        pts = c + g + np.array([0, 0, -off])
        with tempfile.TemporaryDirectory() as td:
            f = os.path.join(td, 'm.npz')
            np.savez(f, centers=pts, resolution=0.02, frame='world')
            ec = EnvChecker(urdf, f)
            v = ec.screen_trajectory([], [[]], j)
        print(f'  box {name:3s}: {v[0]:7s} d={v[1]:+.3f} {v[2]}  (want {want})')
        ok &= v[0] == want
        keep = self_filter(rg, q, pts, ENV_MARGIN_M)
        print(f'  self_filter keeps {keep.sum()}/{len(pts)}')
        ok &= (keep.sum() == 0) if name == 'on' else (keep.sum() == len(pts))
    for what, fn in (('missing joint', lambda: rg.q_from({k: v for k, v in j.items()
                                                           if k != 't2_rotation_joint'})),
                     ('missing map', lambda: EnvChecker(urdf, '/nonexistent.npz'))):
        try:
            fn()
            print(f'  {what}: did NOT raise'); ok = False
        except (KeyError, FileNotFoundError) as e:
            print(f'  {what}: raised ({str(e)[:60]})')
    print('SELF-TEST', 'PASS' if ok else 'FAIL')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--urdf', default=LIVE_URDF)
    a = ap.parse_args()
    if a.self_test:
        sys.exit(0 if self_test(a.urdf) else 1)
    ap.print_help()


if __name__ == '__main__':
    main()
