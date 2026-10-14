#!/usr/bin/env python3
"""Offline F5/D diagnosis (A_locked.md U1-U4) from archived snapshots only.

Pinhole without distortion and depth = optical Z, same as object_localizer.deproject.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'diagnosis_20261014_F5D_01')
RUNS = {'dual': 'capture', 'rgbd_only': 'capture_rgbd_only'}
GT_XY = np.array([0.6497, -0.4779])
SURF, H, R_B = 0.140, 0.165, 0.0325
DMIN, DMAX = 0.1, 12.0


def load_frames(run):
    d = json.load(open(os.path.join(SRC, RUNS[run], 'diagnosis.json')))
    for f in d['frames']:
        a = np.load(os.path.join(SRC, RUNS[run], f['raw_arrays']))
        K = np.array(f['camera_info']['K']).reshape(3, 3)
        R = np.array(f['TF']['R'])
        t = np.array(f['TF']['translation'])
        yield f['camera'], f['raw_arrays'], a, K, R, t


def rays(u, v, K):
    """Optical-frame direction with z=1 for pixel arrays."""
    return np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)


def ray_plane(u, v, K, R, t, z0):
    """World point and optical Z where pixel ray hits world plane z=z0."""
    dw = rays(np.atleast_1d(u), np.atleast_1d(v), K) @ R.T
    s = (z0 - t[2]) / dw[..., 2]
    return t + s[..., None] * dw, s


def valid(depth):
    return np.isfinite(depth) & (depth > DMIN) & (depth < DMAX)


def u1(depth, rgb):
    ok = valid(depth)
    gray = rgb.mean(-1)
    g = ok[:720 // 80 * 80, :].reshape(9, 80, 16, 80).mean((1, 3))
    return ok, {
        'valid_frac': float(ok.mean()),
        'valid_frac_saturated': float(ok[gray >= 250].mean()) if (gray >= 250).any() else None,
        'frac_saturated': float((gray >= 250).mean()),
        'valid_frac_dark': float(ok[gray <= 20].mean()) if (gray <= 20).any() else None,
        'frac_dark': float((gray <= 20).mean()),
        'grid16x9': np.round(g, 3).tolist(),
    }


def u2(depth, K, R, t, rng):
    ok = valid(depth)
    v, u = np.nonzero(ok)
    z = depth[v, u]
    p = rays(u, v, K) * z[:, None] @ R.T + t
    near = np.abs(p[:, 2]) < 0.20
    if near.sum() < 1000:
        return None
    q = p[near]
    best = None
    for _ in range(300):
        s = q[rng.choice(len(q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((q - s[0]) @ n) < 0.02
        if best is None or inl.sum() > best.sum():
            best = inl
    qi = q[best]
    c = qi.mean(0)
    n = np.linalg.svd(qi - c, full_matrices=False)[2][2]
    n = n if n[2] > 0 else -n
    ui, vi, zi = u[near][best], v[near][best], z[near][best]
    _, zpred = ray_plane(ui, vi, K, R, t, 0.0)
    ratio = zi / zpred
    bins = {}
    for lo in np.arange(1.0, 5.0, 0.5):
        m = (zpred >= lo) & (zpred < lo + 0.5)
        if m.sum():
            bins[f'{lo:.1f}-{lo + 0.5:.1f}'] = {'n': int(m.sum()), 'ratio_med': float(np.median(ratio[m])),
                                               'v_med': float(np.median(vi[m]))}
    return {'n_inliers': int(best.sum()), 'tilt_deg': float(np.degrees(np.arccos(n[2]))),
            'normal': n.tolist(), 'centroid': c.tolist(), 'offset_z_at_centroid': float(c[2]),
            'ratio_bins': bins}


def u3_u4(mask, depth, K, R, t):
    ids, cnt = np.unique(mask[mask > 0], return_counts=True)
    if not len(ids):
        return None
    mid = int(ids[np.argmax(cnt)])
    v, u = np.nonzero(mask == mid)
    out = {'id': mid, 'pixels': int(len(u))}
    ok = valid(depth[v, u])
    if ok.any():
        p = rays(u[ok], v[ok], K) * depth[v[ok], u[ok]][:, None] @ R.T + t
        out['M_base'] = np.median(p, 0).tolist()
    vt = np.percentile(v, 2)
    row = np.abs(v - vt) <= 1
    top, _ = ray_plane(np.median(u[row]), vt, K, R, t, SURF + H)
    away = top[0, :2] - t[:2]
    out['M_top'] = (top[0, :2] - R_B * away / np.linalg.norm(away)).tolist()
    ctr, _ = ray_plane(u.mean(), v.mean(), K, R, t, SURF + H / 2)
    out['M_ctr'] = ctr[0, :2].tolist()
    for k in ('M_base', 'M_top', 'M_ctr'):
        if k in out:
            out[k + '_err_xy'] = float(np.linalg.norm(np.array(out[k][:2]) - GT_XY))
    # U4: front-surface intersection of pixel ray with GT cylinder (axis GT_XY, z SURF..SURF+H)
    if ok.any():
        dw = rays(u[ok], v[ok], K) @ R.T
        o = t[:2] - GT_XY
        a = (dw[:, :2] ** 2).sum(1)
        b = 2 * (dw[:, :2] @ o)
        c = o @ o - R_B ** 2
        disc = b * b - 4 * a * c
        hit = disc > 0
        s = (-b[hit] - np.sqrt(disc[hit])) / (2 * a[hit])
        zw = t[2] + s * dw[hit, 2]
        on = (zw > SURF) & (zw < SURF + H)
        if on.sum() >= 10:
            res = depth[v[ok], u[ok]][hit][on] - s[on]
            out['U4_n'] = int(on.sum())
            out['U4_resid_med'] = float(np.median(res))
    return out


def main():
    rng = np.random.default_rng(0)
    result = {}
    for run in RUNS:
        for cam, name, a, K, R, t in load_frames(run):
            ok, r1 = u1(a['depth'], a['rgb'])
            r = result.setdefault(run, {}).setdefault(cam, {'frames': [], 'invalid_maps': []})
            r['invalid_maps'].append(~ok)
            r['frames'].append({'file': name, 'U1': r1, 'U2': u2(a['depth'], K, R, t, rng),
                                'U3U4': u3_u4(a['mask'], a['depth'], K, R, t)})
    for run in result:
        for cam, r in result[run].items():
            maps = np.array(r.pop('invalid_maps'))
            med = np.median(maps, 0) > 0.5
            iou = [float((m & med).sum() / max((m | med).sum(), 1)) for m in maps]
            fr = r['frames']
            s = {'n': len(fr), 'invalid_iou_vs_median_med': float(np.median(iou)),
                 'valid_frac_med': float(np.median([f['U1']['valid_frac'] for f in fr]))}
            u2s = [f['U2'] for f in fr if f['U2']]
            if u2s:
                s['tilt_deg_med'] = float(np.median([x['tilt_deg'] for x in u2s]))
                s['offset_z_med'] = float(np.median([x['offset_z_at_centroid'] for x in u2s]))
                keys = sorted({k for x in u2s for k in x['ratio_bins']})
                s['ratio_bins'] = {k: {'ratio_med': float(np.median([x['ratio_bins'][k]['ratio_med'] for x in u2s
                                                                     if k in x['ratio_bins']])),
                                       'n_total': int(sum(x['ratio_bins'][k]['n'] for x in u2s if k in x['ratio_bins']))}
                                   for k in keys}
            u3 = [f['U3U4'] for f in fr if f['U3U4']]
            for k in ('M_base', 'M_top', 'M_ctr'):
                e = [x[k + '_err_xy'] for x in u3 if k + '_err_xy' in x]
                if e:
                    s[k + '_err_xy_med'] = float(np.median(e))
                    s[k + '_xy_med'] = np.median([x[k][:2] for x in u3 if k in x], 0).tolist()
            r4 = [x['U4_resid_med'] for x in u3 if 'U4_resid_med' in x]
            if r4:
                s['U4_resid_med'] = float(np.median(r4))
                s['U4_n_frames'] = len(r4)
            r['summary'] = s
    out = os.path.join(HERE, 'offline_result.json')
    json.dump(result, open(out, 'w'), indent=1)
    for run in result:
        for cam, r in result[run].items():
            print(run, cam, json.dumps(r['summary'], indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
