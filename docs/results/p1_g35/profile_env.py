#!/usr/bin/env python3
"""G35 A1: where the environment screen's time goes (offline, no ROS graph, no motion).

(a) import + EnvChecker build, split: URDF model, geometry, true_hull, octree
(b) per-sample `check` on: the R10 ev07 rotating rect sweep (G34b HW start state), a
    rail-only sweep, and archived G34b task plans (arm trajectories).

    python3 profile_env.py --out profile_env.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

t_imp0 = time.perf_counter()
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..', '..')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'docs', 'results', 'p1_g29'))
import env_collision as E  # noqa: E402
from interarm_collision import LIVE_URDF, _package_dirs, true_hull  # noqa: E402
t_imp = time.perf_counter() - t_imp0


def rest_state(l1, r1, l2, r2):
    s = {'t1_linear_joint': l1, 't2_linear_joint': l2, 't1_rotation_joint': r1, 't2_rotation_joint': r2}
    rest = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
    for p in ('t1_a1_', 't1_a2_', 't2_a1_', 't2_a2_'):
        s.update({f'{p}joint_{i}': v for i, v in enumerate(rest, 1)})
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'profile_env.json'))
    a = ap.parse_args()
    out = dict(import_s=t_imp)
    import coal
    # ---- (a) build, split
    t0 = time.perf_counter()
    model = pin.buildModelFromUrdf(LIVE_URDF)
    t1 = time.perf_counter()
    geom = pin.buildGeomFromUrdf(model, LIVE_URDF, pin.GeometryType.COLLISION, _package_dirs())
    t2 = time.perf_counter()
    for g in geom.geometryObjects:
        if hasattr(g.geometry, 'buildConvexRepresentation'):
            g.geometry.buildConvexRepresentation(False)
            g.geometry = true_hull(g.geometry.convex)
        g.geometry.computeLocalAABB()
    t3 = time.perf_counter()
    m = E.load_env_map()
    t4 = time.perf_counter()
    coal.makeOctree(m['centers'].astype(float), float(m['resolution']))
    t5 = time.perf_counter()
    out['build'] = dict(model=t1 - t0, geom=t2 - t1, hull=t3 - t2, map_load=t4 - t3, octree=t5 - t4)
    t6 = time.perf_counter()
    ec = E.EnvChecker()
    out['build']['EnvChecker_total'] = time.perf_counter() - t6
    print('import %.2f s; build %s' % (t_imp, {k: round(v, 3) for k, v in out['build'].items()}), flush=True)

    # ---- (b) per-sample
    import g29_rot_screen as R
    cases = {}
    st = rest_state(0.9, 0.0, 0.000963, 0.0)          # R10 ev07 start (HW log: g2 0.000963, g1 0.9? see below)
    cases['rot_sweep_ev07'] = (['t2_linear_joint', 't2_rotation_joint'],
                               R.rect_points((0.000963, 0.0), (0.6, np.radians(-10.0))), st)
    cases['rail_sweep_g1_090_135'] = (['t1_linear_joint', 't1_rotation_joint'],
                                      R.rect_points((0.899438, 0.0), (1.35, 0.0)), rest_state(0.899438, 0, 0, 0))
    plans = [json.loads(l) for l in open(os.path.join(ROOT, 'docs/results/p1_g34/g34_planonly_plans_g34b_hw_R10.jsonl'))]
    for r in plans[:6]:
        cases[f"task_k{r['k']}_{r['arm']}"] = (r['traj']['joint_names'], r['traj']['pos'], r['start'])
    out['per_sample'] = {}
    for name, (jn, pts, base) in cases.items():
        t = time.perf_counter()
        v = ec.screen_trajectory(jn, pts, base)
        dt = time.perf_counter() - t
        out['per_sample'][name] = dict(n=len(pts), total_s=dt, ms_per_sample=1000 * dt / len(pts),
                                       verdict=v[0], d_min=v[1], geom=v[2], k=v[3])
        print(f'{name:28s} n {len(pts):4d}  {dt:7.2f} s  {1000 * dt / len(pts):6.1f} ms/sampel  {v[0]} '
              f'{v[1] * 1000:.1f} mm {v[2]} @{v[3]}', flush=True)
    # per-geometry cost at one config (which bodies are expensive?)
    q = ec.q_from(st)
    oMg = ec.place(q)
    per_g = []
    for i, g in enumerate(ec.geom.geometryObjects):
        M = oMg[i]
        t = time.perf_counter()
        for _ in range(5):
            res = coal.DistanceResult()
            d = coal.distance(g.geometry, coal.Transform3s(M.rotation, M.translation), ec.octree, ec._I, ec._req, res)
        per_g.append((g.name, (time.perf_counter() - t) / 5 * 1000, d))
    per_g.sort(key=lambda x: -x[1])
    out['per_geom_ms_rest'] = per_g
    print('per-geometri (REST, ms):', ', '.join(f'{n} {t:.1f} ({d * 1000:.0f} mm)' for n, t, d in per_g[:8]),
          '... total %.1f' % sum(t for _, t, _ in per_g))
    json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
