#!/usr/bin/env python3
"""G63e LURING: kurva deteksi penanda sintetis vs sisi terproyeksi terkecil (px) -- PRIOR saja; angka nyata = uji kecil HW.

Pose acak di dalam bingkai tiap kamera (incumbent): datar menghadap atas (z 0 / 0.31 / 0.48 / 0.70, yaw acak) dan vertikal
(inti lembar, normal horizontal acak menghadap kamera +-70 deg, pusat z 0.2-0.9). Render g63e_render (blur 1.3-1.6, noise,
latar frame lama), detektor g63e_detect (prior = kebenaran). Per bin 5 px: laju deteksi per frame, galat sudut (px RMS vs
proyeksi benar), dipisah datar/vertikal.

    python3 g63e_detcurve.py [--n 3000] [--out g63e_detcurve.json]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402
import g63e_detect as D  # noqa: E402
import g63e_render as RN  # noqa: E402
import g63d_common as GD  # noqa: E402


def sample(rng, cp, cam):
    T_wc = cp[cam]['T_wc']
    o = T_wc[:3, 3]
    for _ in range(1000):
        if rng.random() < 0.5:
            z = rng.choice([0.0, 0.31, 0.48, 0.70])
            c = np.r_[rng.uniform(-1.2, 3.0), rng.uniform(-2.0, 2.0), z]
            R = E.R_flat(rng.uniform(0, 2 * np.pi))
            kind, core = 'datar', False
        else:
            c = np.r_[rng.uniform(-1.2, 3.0), rng.uniform(-2.0, 2.0), rng.uniform(0.2, 0.9)]
            to_cam = np.degrees(np.arctan2(o[1] - c[1], o[0] - c[0]))
            R = E.R_vert(to_cam + rng.uniform(-70, 70))
            kind, core = 'vertikal', True
        T = E.T_of(R, c)
        uv, z = E.project(T_wc, cp[cam]['K'], np.zeros(5), E.to_w(T, E.paper_t(core=core)))
        if (z <= 0.3).any() or (uv < 5).any() or (uv[:, 0] > E.IMG_W - 5).any() or (uv[:, 1] > E.IMG_H - 5).any():
            continue
        if (c - o) @ R[:, 2] <= 0:
            continue
        tu, _ = E.project(T_wc, cp[cam]['K'], np.zeros(5), E.to_w(T, E.tag_corners_t()))
        side = float(np.linalg.norm(tu - np.roll(tu, 1, 0), axis=1).min())
        if 12 <= side <= 75:
            return kind, core, T, tu, side, float(np.linalg.norm(c - o))
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, default=3000)
    ap.add_argument('--out', default=os.path.join(E.HERE, 'g63e_detcurve.json'))
    ap.add_argument('--raw', action='store_true', help='tanpa penyempurnaan tepi (sudut SUBPIX OpenCV)')
    a = ap.parse_args()
    rng = np.random.default_rng(7)
    cp = GD.cam_params()
    bgs = RN.backgrounds()
    tex = {core: RN.tag_texture(27, core=core) for core in (False, True)}
    rows = []
    for k in range(a.n):
        cam = E.CAMS[k % 2]
        s = sample(rng, cp, cam)
        if s is None:
            continue
        kind, core, T, tu, side, dist = s
        t, pap = tex[core]
        img = RN.render(bgs[cam], [(t, pap, T)], cp[cam]['T_wc'], cp[cam]['K'], rng)
        pr = D.tag_priors({27: T}, cp[cam]['T_wc'], cp[cam]['K'], np.zeros(5))
        got, why = D.detect_tags_frame(img, pr, min_px=0, refine=not a.raw)
        err = float(np.sqrt(np.mean(np.sum((got[27] - tu) ** 2, 1)))) if 27 in got else None
        rows.append(dict(cam=cam, kind=kind, side=round(side, 1), dist=round(dist, 2), ok=27 in got, err=err, why=why))
        if k % 500 == 0:
            print(k, flush=True)
    print('bin px | datar: laju n err_med err_p95 | vertikal: laju n err_med err_p95')
    summ = {}
    for lo in range(10, 75, 5):
        line = '%2d-%2d' % (lo, lo + 5)
        for kind in ('datar', 'vertikal'):
            R = [r for r in rows if r['kind'] == kind and lo <= r['side'] < lo + 5]
            if not R:
                line += ' |   -'
                continue
            e = [r['err'] for r in R if r['ok']]
            rate = np.mean([r['ok'] for r in R])
            summ['%s_%d' % (kind, lo)] = dict(n=len(R), rate=round(float(rate), 3),
                                               err_med=round(float(np.median(e)), 3) if e else None,
                                               err_p95=round(float(np.percentile(e, 95)), 3) if e else None)
            line += ' | %.2f %4d %s %s' % (rate, len(R), '%.2f' % np.median(e) if e else '  - ', '%.2f' % np.percentile(e, 95) if e else '  - ')
        print(line, flush=True)
    json.dump(dict(summary=summ, rows=rows), open(a.out, 'w'), indent=0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
