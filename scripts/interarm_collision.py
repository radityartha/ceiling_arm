#!/usr/bin/env python3
"""Inter-arm collision screening for the two arms that share one gantry.

WHY THIS EXISTS, and it is not redundancy. Measured 2026-08-17 on the SRDF this
cell actually loads (`workcell_moveit_config/config/trailer_workcell.srdf`):

    112 of the 121 geometry-bearing t1_a1_* <-> t1_a2_* link pairs carry
    <disable_collisions ... reason="Never"/>, and gantry 2 is the same (143
    cross entries). Everything that matters is off: arm<->arm, forearm<->
    forearm, gripper<->anything, base<->anything. Nine wrist-vs-arm pairs
    survive.

"Never" is the MoveIt Setup Assistant reporting that it never sampled the two
links in contact. It is wrong here: the mounts are 0.800 m apart on gantry 1 and
each arm reaches 1.005 m base->tool_frame, so either arm can sweep 0.2 m PAST
the other arm's base. So MoveIt will plan arm_1 straight through arm_2 and call
the plan valid -- and G16 B4.2 already measured what happens when the only
protection is a guard that WATCHES rather than one that REFUSES.

This screens a planned trajectory against the OTHER arm held at its current
measured configuration, before ExecuteTrajectory, in the same place the torque
screen already sits in reach_dwell_probe.move_to().

Its limits, stated rather than implied:
  * it screens the pair of arms on ONE gantry, not arm-vs-world or arm-vs-
    octomap -- MoveIt still does those, and they are not disabled.
  * it is a discrete check at trajectory waypoints, like every other collision
    check here, so a fast sweep between two widely spaced waypoints can pass
    through a thin obstacle. Margin exists to absorb that; it is not a proof.
  * it uses the URDF collision meshes, which are the convex-ish hulls Kinova
    ships, so it is slightly conservative on the grippers.

    python3 scripts/interarm_collision.py --self-test
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pinocchio as pin

# Arms that physically share a gantry and can therefore reach each other.
GANTRY_PAIRS = {'gantry_1': ('t1_a1_', 't1_a2_'),
                'gantry_2': ('t2_a1_', 't2_a2_')}
ARM_PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_',
              'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
DEFAULT_MARGIN_M = 0.05
LIVE_URDF = '/tmp/reach_dwell_live.urdf'


def _package_dirs():
    return [os.path.join(p, 'share')
            for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]


class InterArmChecker:
    """Distance between the two arms of one gantry, at a full configuration."""

    def __init__(self, urdf=LIVE_URDF, gantry='gantry_1',
                 margin=DEFAULT_MARGIN_M):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(
            self.model, urdf, pin.GeometryType.COLLISION, _package_dirs())
        self.margin = margin

        pa, pb = GANTRY_PAIRS[gantry]
        names = [g.name for g in self.geom.geometryObjects]
        ia = [i for i, n in enumerate(names) if n.startswith(pa)]
        ib = [i for i, n in enumerate(names) if n.startswith(pb)]
        if not ia or not ib:
            raise ValueError(f'{gantry}: geometri lengan tidak ditemukan '
                             f'({len(ia)} vs {len(ib)})')
        # ONLY cross-arm pairs. Everything else -- self-collision within one
        # arm, arm vs world, arm vs octomap -- is MoveIt's job and is NOT
        # disabled there. Duplicating it here would just make this slower and
        # invite the two checkers to disagree.
        self.geom.removeAllCollisionPairs()
        for i in ia:
            for j in ib:
                self.geom.addCollisionPair(pin.CollisionPair(i, j))
        self.geom_data = self.geom.createData()
        self.names = names
        self.n_pairs = len(self.geom.collisionPairs)

    def q_from(self, joint_positions, q=None):
        """Full configuration vector with the named joints written into it."""
        q = pin.neutral(self.model) if q is None else q.copy()
        for name, value in joint_positions.items():
            jid = self.model.getJointId(name)
            if jid < self.model.njoints:
                q[self.model.joints[jid].idx_q] = value
        return q

    def check(self, q):
        """(min_distance_m, worst_pair_names). Negative distance = penetrating."""
        pin.updateGeometryPlacements(self.model, self.data, self.geom,
                                     self.geom_data, q)
        pin.computeDistances(self.model, self.data, self.geom, self.geom_data, q)
        worst, best = None, float('inf')
        for k, res in enumerate(self.geom_data.distanceResults):
            if res.min_distance < best:
                best = res.min_distance
                cp = self.geom.collisionPairs[k]
                worst = (self.names[cp.first], self.names[cp.second])
        return best, worst

    def screen_trajectory(self, joint_names, points, other_joints):
        """Screen one arm's planned waypoints against the other arm HELD still.

        Returns (verdict, min_distance_m, worst_pair, waypoint_index) where
        verdict is 'CLEAR' | 'MARGIN' | 'COLLIDE'. MARGIN means the arms come
        closer than the margin without touching -- reported separately, because
        collapsing "nearly hit" into "fine" is how the torque guard in G16 came
        to be a detector instead of a preventer.
        """
        base = self.q_from(other_joints)
        worst_d, worst_pair, worst_k = float('inf'), None, -1
        for k, pt in enumerate(points):
            q = self.q_from(dict(zip(joint_names, pt)), q=base)
            d, pair = self.check(q)
            if d < worst_d:
                worst_d, worst_pair, worst_k = d, pair, k
        if worst_d <= 0.0:
            return 'COLLIDE', worst_d, worst_pair, worst_k
        if worst_d < self.margin:
            return 'MARGIN', worst_d, worst_pair, worst_k
        return 'CLEAR', worst_d, worst_pair, worst_k


def _ik(model, data, prefix, target, q0, iters=300):
    """Position-only damped least squares -- same solver as torque_safe_workspace."""
    names = [f'{prefix}joint_{i}' for i in range(1, 7)]
    idx = {n: (model.joints[model.getJointId(n)].idx_q,
               model.joints[model.getJointId(n)].idx_v) for n in names}
    tool = model.getFrameId(f'{prefix}tool_frame')
    q = q0.copy()
    for _ in range(iters):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        err = target - data.oMf[tool].translation
        if np.linalg.norm(err) < 2e-3:
            return q, True
        J = pin.computeFrameJacobian(model, data, q, tool,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
        Jr = np.zeros((3, 6))
        for c, n in enumerate(names):
            Jr[:, c] = J[:, idx[n][1]]
        dq = Jr.T @ np.linalg.solve(Jr @ Jr.T + 1e-6 * np.eye(3), err)
        for c, n in enumerate(names):
            q[idx[n][0]] += 0.5 * dq[c]
    return q, False


def self_test(urdf=LIVE_URDF, lin=0.550):
    """A checker that has never fired is not evidence of anything.

    Two configurations with a known answer: both arms HANGING (they are 0.8 m
    apart and must be clear), and both arms driven to the SAME world point
    (they cannot both be there, so the check must report contact). If the
    second one does not fire, the screen is useless and must not be trusted.
    """
    c = InterArmChecker(urdf=urdf, gantry='gantry_1')
    print(f'pasangan silang diperiksa: {c.n_pairs}')
    rest = dict(zip([f't1_a1_joint_{i}' for i in range(1, 7)],
                    [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]))
    rest.update(zip([f't1_a2_joint_{i}' for i in range(1, 7)],
                    [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]))
    rest['t1_linear_joint'] = lin
    d, pair = c.check(c.q_from(rest))
    ok_rest = d > 0
    print(f'  A. dua lengan MENGGANTUNG      : jarak {d * 1000:8.1f} mm  '
          f'{"BEBAS ✓" if ok_rest else "TABRAKAN ✗"}   {pair}')

    q = c.q_from(rest)
    mid = np.array([0.55, 0.36, 1.40])          # di dalam kotak aman KEDUANYA
    q, o1 = _ik(c.model, c.data, 't1_a1_', mid, q)
    q, o2 = _ik(c.model, c.data, 't1_a2_', mid, q)
    d2, pair2 = c.check(q)
    ok_hit = d2 <= 0
    print(f'  B. dua lengan ke titik SAMA {mid} : jarak {d2 * 1000:8.1f} mm  '
          f'{"TABRAKAN ✓ (benar)" if ok_hit else "BEBAS ✗ (pemeriksa MATI)"}   {pair2}')
    print(f'     (IK konvergen: arm_1 {o1}, arm_2 {o2})')

    good = ok_rest and ok_hit and o1 and o2
    print(f'\nSWA-UJI: {"LULUS" if good else "GAGAL"} -- pemeriksa '
          f'{"membedakan" if good else "TIDAK membedakan"} bebas dari tabrakan.')
    return 0 if good else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', default=LIVE_URDF)
    ap.add_argument('--lin', type=float, default=0.550)
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return self_test(a.urdf, a.lin)
    ap.error('pakai --self-test, atau impor InterArmChecker')


if __name__ == '__main__':
    sys.exit(main())
