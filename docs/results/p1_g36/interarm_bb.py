"""G36 C: EXACT branch-and-bound for InterArmChecker.screen_trajectory (S18 same-gantry meshes) and, by
inheritance, CrossGantryChecker (true hulls) -- the G35 env_collision method applied to PAIRS.

Distance is 1-Lipschitz in rigid motion of either body, so for a pair (a, b)
    d(k) >= d(last evaluated) - disp_a - disp_b,     disp = |dt| + r * ||dR||_F   (Frobenius >= spectral)
r = radius about the geometry's own origin containing it (local AABB corners; checked >= every mesh vertex).
A (waypoint, pair) is skipped only when that bound, less SLACK for round-off, is > 0 and >= the running
minimum -- it cannot change the minimum, its pair or its waypoint. Iteration order (waypoint, pair) and the
strict `<` are those of check() + the old loop, so the first minimum found is the same one. A pair whose
bodies do not move is thereby computed once. Per-pair pin.computeDistance = what computeDistances calls.

Loaded here (not yet in scripts/interarm_collision.py) for the A3 regression; ported after it passes."""
import numpy as np
import pinocchio as pin

SLACK = 1e-3


def radii(geom, used):
    """Largest |vertex| about each geometry's origin. NOT aabb_local: on the S18 BVH meshes it is never
    computed and reads inf (found here -- an inf radius never prunes; a too-small one would be unsafe)."""
    r = []
    for x_, g in enumerate(geom.geometryObjects):
        if x_ not in used:
            r.append(float('inf'))                           # in no pair: never read
            continue
        G = g.geometry
        if hasattr(G, 'num_vertices'):                       # BVHModel (S18 meshes)
            V = np.array([G.vertices(i) for i in range(G.num_vertices)])
        elif hasattr(G, 'num_points'):                       # Convex (true hulls, cross)
            V = np.array([G.points(i) for i in range(G.num_points)])
        else:
            G.computeLocalAABB()
            bb = G.aabb_local                                  # corner of the local AABB (G35)
            V = np.maximum(np.abs(np.array(bb.min_)), np.abs(np.array(bb.max_)))[None]
        x = float(np.linalg.norm(np.abs(V), axis=1).max()) if len(V) else float('inf')
        if not np.isfinite(x):
            raise ValueError(f'{g.name}: radius {x} -- refusing to prune on it')
        r.append(x)
    return r


def screen_trajectory(self, joint_names, points, other_joints, only=None):
    if not hasattr(self, '_r'):
        self._r = radii(self.geom, {x for cp in self.geom.collisionPairs for x in (cp.first, cp.second)})
    base = self.q_from(other_joints)
    pairs = [(k, cp.first, cp.second) for k, cp in enumerate(self.geom.collisionPairs)
             if not only or self.names[cp.first].startswith(only) or self.names[cp.second].startswith(only)]
    last = {}
    worst_d, worst_pair, worst_k = float('inf'), None, -1
    for w, pt in enumerate(points):
        q = self.q_from(dict(zip(joint_names, pt)), q=base)
        pin.updateGeometryPlacements(self.model, self.data, self.geom, self.geom_data, q)
        oMg = self.geom_data.oMg
        place = {}
        for k, i, j in pairs:
            for x in (i, j):
                if x not in place:
                    place[x] = (np.array(oMg[x].rotation), np.array(oMg[x].translation))
            (Ri, ti), (Rj, tj) = place[i], place[j]
            if k in last:
                d0, Ri0, ti0, Rj0, tj0 = last[k]
                lb = (d0 - np.linalg.norm(ti - ti0) - self._r[i] * np.linalg.norm(Ri - Ri0)
                      - np.linalg.norm(tj - tj0) - self._r[j] * np.linalg.norm(Rj - Rj0) - SLACK)
                if lb > 0.0 and lb >= worst_d:
                    continue
            d = pin.computeDistance(self.geom, self.geom_data, k).min_distance
            last[k] = (d, Ri, ti, Rj, tj)
            if d < worst_d:
                worst_d, worst_pair, worst_k = d, (self.names[i], self.names[j]), w
    if worst_d <= 0.0:
        return 'COLLIDE', worst_d, worst_pair, worst_k
    if worst_d < self.margin:
        return 'MARGIN', worst_d, worst_pair, worst_k
    return 'CLEAR', worst_d, worst_pair, worst_k
