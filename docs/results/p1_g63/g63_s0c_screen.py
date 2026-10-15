#!/usr/bin/env python3
"""G63 S0c: screen the locked G63 sequence (arm + board, payload URDF) against the NEW
voxels found by g63_s0c.py (objects not in env_static_map, e.g. the moved drawer).

Goals are exact. Transit is a PROXY only: straight joint-space interpolation between
consecutive goals (G41: OMPL joint goal ~ straight line); the real HW plan may differ.

    python3 g63_s0c_screen.py --payload payload.urdf --poses poses_real_v1.json \
        --clusters hw_.../s0c_report_clusters.npy --out hw_.../s0c_screen.json
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g63_screens  # noqa: E402  (extends EXPECTED_GEOMS with the board geoms)
from g63_common import ARM_PREFIX  # noqa: E402
from g63_plancheck import sequence  # noqa: E402
from g63_poses import REST  # noqa: E402
import env_collision  # noqa: E402

STEPS = 40


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--payload', required=True)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--clusters', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--exclude', type=int, nargs='*', default=[],
                    help='cluster idx proven free space (ray test), logged in the report')
    a = ap.parse_args()
    g63_screens.install(a.payload)
    vox = np.load(a.clusters)
    vox = vox[~np.isin(vox[:, 3].astype(int), a.exclude)]
    tmp = os.path.splitext(a.out)[0] + '_newmap.npz'
    np.savez_compressed(tmp, centers=vox[:, :3], resolution=np.float64(0.02), frame='world')
    ec = env_collision.EnvChecker(map_file=tmp)
    P = json.load(open(a.poses))

    base = {}
    for pre in ARM_PREFIX.values():
        base.update({f'{pre}joint_{k + 1}': v for k, v in enumerate(REST)})
    base.update(t1_linear_joint=P['rail1'], t2_linear_joint=P['rail2'],
                t1_rotation_joint=0.0, t2_rotation_joint=0.0)
    names = sorted(base)
    rep = {'excluded_clusters': a.exclude, 'payload': a.payload, 'map': tmp, 'n_voxels': ec.n_voxels, 'rails': [P['rail1'], P['rail2']],
           'margin_m': ec.margin, 'arms': {}}
    for arm, A in P['arms'].items():
        pre = ARM_PREFIX[arm]
        seq = sequence(A['poses'], ['P0', 'T1', 'T2', 'T3'])   # P['stages'] = cumulative counts
        rest = dict(id='REST', goal_joints={f'{pre}joint_{k + 1}': v for k, v in enumerate(REST)})
        seq = [rest] + seq + [rest]
        rows, prev = [], None
        for p in seq:
            g = p['goal_joints']
            g = ast.literal_eval(g) if isinstance(g, str) else g
            cfg = dict(base, **g)
            q = np.array([cfg[n] for n in names])
            d, w = ec.check(ec.q_from(cfg), only=[pre])
            cl = int(vox[np.argmin(np.linalg.norm(vox[:, :3] - ec.place(ec.q_from(cfg))[ec.names.index(w)].translation, axis=1)), 3])
            row = dict(id=p['id'], stage=p.get('stage', '-'), goal_d=round(d, 3), goal_geom=w, goal_cl=cl)
            if prev is not None:
                pts = [prev + (q - prev) * s / STEPS for s in range(STEPS + 1)]
                dm, wm, xm = min(((*ec.check(ec.q_from(dict(zip(names, x))), only=[pre]), x) for x in pts),
                                 key=lambda t: t[0])
                tw = ec.place(ec.q_from(dict(zip(names, xm))))[ec.names.index(wm)].translation
                row.update(leg_d=round(dm, 3), leg_geom=wm,
                           leg_cl=int(vox[np.argmin(np.linalg.norm(vox[:, :3] - tw, axis=1)), 3]))
            prev = q
            rows.append(row)
        rep['arms'][arm] = rows
        print(arm)
        for r in rows:
            flag = ' <-- MARGIN' if min(r['goal_d'], r.get('leg_d', 9)) < ec.margin else ''
            print(f"  {r['id']:6s} {r['stage']:4s} goal {r['goal_d']:6.3f} ({r['goal_geom']})"
                  f" #{r['goal_cl']}  leg {r.get('leg_d', float('nan')):6.3f} ({r.get('leg_geom', '-')}) #{r.get('leg_cl', '-')}{flag}")
    json.dump(rep, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
