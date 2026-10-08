#!/usr/bin/env python3
"""G34 step 1: where does each camera see each gantry's arm pair, vs URDF, per rail s?

From rail_calib.py rows (4-DOF per gantry, yaw about the world origin): the
displacement camera - URDF AT THE ARM-PAIR CENTROID (t and yaw are coupled at the
origin, the centroid displacement is not). Then per camera x gantry a line in s:
slope = rail direction/scale error, intercept = fixed camera-world offset.
Rows need >= 300 points on BOTH arms (one-arm fits leave yaw undetermined).

    python3 rail_table.py rail_calib_diff.json
"""
import json
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g33'))
warnings.simplefilter('ignore')
from build_env_map import capture_config  # noqa: E402
from env_collision import RobotGeom  # noqa: E402

rg = RobotGeom()
rows = json.load(open(sys.argv[1]))
out, cache = [], {}
for r in rows:
    if r['group'] == 'all':
        continue
    key = (r['capture'], r['group'])
    if key not in cache:
        cfg, _ = capture_config(os.path.join(HERE, r['capture']), 'capture')
        oMg = rg.place(rg.q_from(cfg))
        pre = 't1' if r['group'] == 'g1' else 't2'
        cache[key] = np.mean([oMg[i].translation for i, n in enumerate(rg.names)
                              if n[:2] == pre and n[3] == 'a'], 0)
    c = cache[key]
    y = np.radians(r['yaw_deg'])
    R = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    disp = R.T @ (c - np.array(r['t'])) - c
    s = r['s1'] if r['group'] == 'g1' else r['s2']
    ok = min(r['per_arm']) >= 300
    out.append(dict(capture=r['capture'], cam=r['cam'], group=r['group'], s=round(s, 4),
                    disp=disp.round(4).tolist(), yaw=round(r['yaw_deg'], 2),
                    med=round(r['median_absd_after'], 4), per_arm=r['per_arm'], ok=ok))
    print(f"{r['capture']} {r['cam']:5s} {r['group']} s={s:.3f} cam-URDF {disp.round(3)} "
          f"yaw {r['yaw_deg']:+6.2f} med {r['median_absd_after']:.3f} {r['per_arm']}{'' if ok else '  (dibuang)'}")
fits = {}
for cam in ('rgbd', 'rgbd2'):
    for g in ('g1', 'g2'):
        sel = [o for o in out if o['cam'] == cam and o['group'] == g and o['ok']]
        if len(sel) < 3:
            continue
        S = np.array([o['s'] for o in sel])
        D = np.array([o['disp'] for o in sel])
        A = np.c_[np.ones_like(S), S]
        coef, *_ = np.linalg.lstsq(A, D, rcond=None)
        res = D - A @ coef
        fits[f'{cam}:{g}'] = dict(n=len(sel), s=S.tolist(), intercept=coef[0].round(4).tolist(),
                                  slope_per_m=coef[1].round(4).tolist(),
                                  resid_max=float(np.abs(res).max().round(4)),
                                  spread=(D.max(0) - D.min(0)).round(4).tolist())
        print(f'{cam:5s} {g}: n {len(sel)}  offset {coef[0].round(3)}  per m {coef[1].round(3)}  '
              f'residual maks {np.abs(res).max():.3f}  rentang {(D.max(0) - D.min(0)).round(3)}')
json.dump(dict(rows=out, fits=fits), open(os.path.splitext(sys.argv[1])[0] + '_table.json', 'w'), indent=1)
