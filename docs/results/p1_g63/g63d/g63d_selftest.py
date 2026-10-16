#!/usr/bin/env python3
"""G63d uji mandiri sintetis (tugas 2): rencana papan -> citra sintetis -> detektor terkunci + penjaga ID -> solver.

Kebenaran: kamera = incumbent diganggu (rotasi acak ROT_DEG, geser acak TRANS_M) -- prior detektor tetap memakai incumbent
(seperti di HW). Papan benar = rencana; "pita" = benar + galat (sigma xy 3 mm, yaw 0.5 deg, z 3 mm). Citra: lembar A4 penuh
(pola di tengah), hitam 30 / putih 230, blur Gauss acak 1.3-1.6 px (dikalibrasi ke kegagalan P0 nyata, G63b A8.1), noise 2.3,
latar = frame lama (lokal) atau abu, N_FRAMES frame per kamera (noise independen) -> median seperti capture.
Mode:
  nominal  -> galat kamera vs kebenaran, distribusi tinggal-satu-papan (dasar ambang G3), papan bersama (G4);
  neg3cm   -> satu papan (acak, terlihat >= 1 kamera) di pita digeser 3 cm horizontal: G3 HARUS menandai papan itu.
    python3 g63d_selftest.py --plan g63d_plan.json --seeds 20 [--bg old|grey] [--out selftest.json]
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402
import g63d_solve as S  # noqa: E402

ROT_DEG, TRANS_M = 1.5, 0.05
SIG_XY, SIG_YAW, SIG_Z = 0.003, np.radians(0.5), 0.003
N_FRAMES = 5
PPM = 3000


def texture(sq):
    w, h = int(round(G.PAPER[0] * PPM)), int(round(G.PAPER[1] * PPM))
    t = np.full((h, w), 230, np.uint8)
    s = sq * PPM
    x0, y0 = (G.PAPER[0] - G.NX * sq) / 2 * PPM, (G.PAPER[1] - G.NY * sq) / 2 * PPM
    for j in range(G.NY):
        for i in range(G.NX):
            if (i + j) % 2 == 0:
                t[int(round(y0 + j * s)):int(round(y0 + (j + 1) * s)), int(round(x0 + i * s)):int(round(x0 + (i + 1) * s))] = 30
    return t


def render(bg, boards_true, T_wc, K, sq, tex, rng):
    img = bg.astype(np.float32).copy()
    h, w = tex.shape
    for b in boards_true.values():
        uv, z = G.project(T_wc, K, np.zeros(5), G.to_w(b['T'], G.paper_b(sq)))
        if (z <= 0.1).any():
            continue
        Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), uv.astype(np.float32))
        m = cv2.warpPerspective(np.full_like(tex, 255), Hm, (G.IMG_W, G.IMG_H), flags=cv2.INTER_AREA)
        p = cv2.warpPerspective(tex, Hm, (G.IMG_W, G.IMG_H), flags=cv2.INTER_AREA)
        img = np.where(m > 127, p, img)
    img = cv2.GaussianBlur(img, (0, 0), rng.uniform(1.3, 1.6))
    return np.clip(img + rng.normal(scale=2.3, size=img.shape), 0, 255).astype(np.uint8)


def perturb_board(T, rng, shift=None):
    Tn = T.copy()
    yaw = rng.normal(scale=SIG_YAW)
    Tn[:3, :3] = Rotation.from_euler('z', yaw).as_matrix() @ T[:3, :3]
    Tn[:3, 3] = T[:3, 3] + np.r_[rng.normal(scale=SIG_XY, size=2), rng.normal(scale=SIG_Z)]
    if shift is not None:
        Tn[:3, 3] += shift
    return Tn


def run(plan, seeds, bg_mode, mode):
    sq = plan['sq']
    cp = G.cam_params()
    tex = texture(sq)
    bgs = {}
    for c in G.CAMS:
        f = sorted(glob.glob(G.G63 + '/g63b/hw_20261016_finger/frames/B-R0/%s/*.png' % c))
        bgs[c] = cv2.cvtColor(cv2.imread(f[-1]), cv2.COLOR_BGR2GRAY) if (bg_mode == 'old' and f) else np.full((G.IMG_H, G.IMG_W), 90, np.uint8)
    names = [b['name'] for b in plan['boards']]
    true = {b['name']: dict(T=G.T_wb(b['K1'][:2], b['center'][2], np.arctan2((np.array(b['K2']) - b['K1'])[0], -(np.array(b['K2']) - b['K1'])[1])))
            for b in plan['boards']}
    vis = {c: [b['name'] for b in plan['boards'] if b['vis'][c]['ok']] for c in G.CAMS}
    rows = []
    for sd in range(seeds):
        rng = np.random.default_rng(1000 + sd)
        cams, cams_true, obs_all = {}, {}, {}
        bad = None
        if mode == 'neg3cm':
            bad = names[rng.integers(len(names))]
            a = rng.uniform(0, 2 * np.pi)
            shift = 0.03 * np.array([np.cos(a), np.sin(a), 0])
        tape = {n: dict(T=perturb_board(true[n]['T'], rng, shift if n == bad else None)) for n in names}
        for c in G.CAMS:
            K = cp[c]['K']
            Tt = cp[c]['T_wc'].copy()
            ax = rng.normal(size=3)
            Tt[:3, :3] = Rotation.from_rotvec(np.radians(ROT_DEG) * ax / np.linalg.norm(ax)).as_matrix() @ Tt[:3, :3]
            d = rng.normal(size=3)
            Tt[:3, 3] += TRANS_M * d / np.linalg.norm(d)
            cams_true[c] = Tt
            cams[c] = dict(K=K, dist=np.zeros(5), T_wc=cp[c]['T_wc'])
            frames = [render(bgs[c], {n: true[n] for n in vis[c]}, Tt, K, sq, tex, rng) for _ in range(N_FRAMES)]
            obs = S.detect_cam(frames, {n: tape[n] for n in names}, cams[c]['T_wc'], K, cams[c]['dist'], sq)
            obs_all[c] = {n: o for n, o in obs.items() if n in vis[c] or 'uv' in o}
        rep, fits = S.solve(obs_all, tape, cams, sq)
        row = dict(seed=sd, mode=mode, bad=bad)
        for c in G.CAMS:
            if c not in fits:
                row[c] = dict(fail=rep[c].get('why'))
                continue
            dT = np.linalg.inv(cams_true[c]) @ fits[c]['T']
            # galat titik ruang kerja (kisi terlihat) vs kebenaran
            X = np.array([[x, y, z] for x in np.arange(-0.5, 2.6, 0.25) for y in np.arange(-1.2, 1.3, 0.25) for z in (0, 0.77, 1.3)])
            uv, dep = G.project(cams_true[c], cams[c]['K'], np.zeros(5), X)
            m = (dep > 0.3) & (dep < 3) & (uv[:, 0] > 0) & (uv[:, 0] < G.IMG_W) & (uv[:, 1] > 0) & (uv[:, 1] < G.IMG_H)
            uv2, _ = G.project(fits[c]['T'], cams[c]['K'], np.zeros(5), X[m])
            pe = np.linalg.norm(uv2 - uv[m], axis=1) * dep[m] / cams[c]['K'][0, 0] * 1000
            det = {n: (o.get('n_ok', 0), o['n']) for n, o in obs_all[c].items() if n in vis[c]}
            row[c] = dict(rot_deg=round(float(np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude())), 3),
                          pos_mm=round(float(1000 * np.linalg.norm(dT[:3, 3])), 1),
                          pt_mm_med=round(float(np.median(pe)), 1), pt_mm_p95=round(float(np.percentile(pe, 95)), 1),
                          sigma_px=rep[c]['sigma_px'], lobo=rep[c]['lobo_mm'], det=det,
                          wrong_label=[n for n, o in obs_all[c].items() if 'uv' in o and n not in vis[c]])
        row['G4'] = rep['G4']['shared']
        rows.append(row)
        print(json.dumps(row, default=str), flush=True)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', default=os.path.join(G.HERE, 'g63d_plan.json'))
    ap.add_argument('--seeds', type=int, default=20)
    ap.add_argument('--bg', default='old', choices=('old', 'grey'))
    ap.add_argument('--mode', default='both', choices=('nominal', 'neg3cm', 'both'))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    rows = []
    for m in (('nominal', 'neg3cm') if a.mode == 'both' else (a.mode,)):
        rows += run(plan, a.seeds, a.bg, m)
    if a.out:
        json.dump(rows, open(a.out, 'w'), indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
