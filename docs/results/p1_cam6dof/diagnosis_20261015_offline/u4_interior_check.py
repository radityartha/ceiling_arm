#!/usr/bin/env python3
"""U4 follow-up (not in A_locked): bottle depth residual on interior pixels only.

Interior = ray passes within 0.5 R of the GT cylinder axis in xy, so grazing/edge
(mixed) pixels are excluded. Residual = measured optical Z - GT front-surface Z.
"""
import json
import os

import numpy as np

import analyze_offline as A

HERE = os.path.dirname(os.path.abspath(__file__))
out = {}
for run in A.RUNS:
    for cam, name, a, K, R, t in A.load_frames(run):
        mask, depth = a['mask'], a['depth']
        ids, cnt = np.unique(mask[mask > 0], return_counts=True)
        if not len(ids):
            continue
        v, u = np.nonzero(mask == ids[np.argmax(cnt)])
        ok = A.valid(depth[v, u])
        u, v = u[ok], v[ok]
        dw = A.rays(u, v, K) @ R.T
        o = t[:2] - A.GT_XY
        a2 = (dw[:, :2] ** 2).sum(1)
        b = 2 * (dw[:, :2] @ o)
        smin = -b / (2 * a2)                       # closest approach to axis (xy)
        dmin = np.linalg.norm(o + smin[:, None] * dw[:, :2], axis=1)
        c = o @ o - A.R_B ** 2
        disc = b * b - 4 * a2 * c
        hit = (disc > 0) & (dmin < 0.5 * A.R_B)
        s = (-b[hit] - np.sqrt(disc[hit])) / (2 * a2[hit])
        zw = t[2] + s * dw[hit, 2]
        on = (zw > A.SURF + 0.01) & (zw < A.SURF + A.H - 0.01)
        if on.sum() < 5:
            continue
        res = depth[v[hit][on], u[hit][on]] - s[on]
        r = out.setdefault(f'{run}/{cam}', {'frame_med': [], 'frame_iqr': [], 'n': [], 'gt_Z': []})
        r['frame_med'].append(float(np.median(res)))
        r['frame_iqr'].append(float(np.subtract(*np.percentile(res, [75, 25]))))
        r['n'].append(int(on.sum()))
        r['gt_Z'].append(float(np.median(s[on])))
summ = {k: {'frames': len(r['n']), 'resid_med': float(np.median(r['frame_med'])),
            'resid_p10_p90': np.percentile(r['frame_med'], [10, 90]).tolist(),
            'iqr_med': float(np.median(r['frame_iqr'])), 'n_px_med': float(np.median(r['n'])),
            'gt_Z_med': float(np.median(r['gt_Z']))} for k, r in out.items()}
json.dump({'per_frame': out, 'summary': summ}, open(os.path.join(HERE, 'u4_interior.json'), 'w'), indent=1)
print(json.dumps(summ, indent=1))
