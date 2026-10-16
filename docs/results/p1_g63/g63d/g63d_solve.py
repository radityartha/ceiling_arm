#!/usr/bin/env python3
"""G63d solver: ekstrinsik D455 dari papan polos STATIS berjangkar pita (tanpa FK sendi).

Per kamera: deteksi tiap papan di tiap frame (detektor TERKUNCI g63b_detect.detect_face, prior = pose pita + incumbent) +
penjaga identitas g63c_guard.check (prior SEMUA papan) -> median uv 12 sudut bila >= 50 % frame lolos -> PnP gabungan
(semua sudut semua papan, pose dunia papan dari pita) -> T_wc 6 DOF.
  G3 tinggalkan-satu-papan: papan i dikeluarkan, kamera dipaskan dari sisanya, galat lateral papan i (mm, = |duv| x
     kedalaman / f, rata-rata 12 sudut) dibandingkan ambang terdaftar.
  G4 papan bersama: untuk papan s terlihat dua kamera, tiap kamera dipaskan TANPA s; 12 sudut s ditriangulasi dari dua
     kamera itu (baseline ~2.7 m) -> galat 3D rata-rata vs sudut dari pita (bebas pita s; menguji kedua ekstrinsik bersama).
     (PnP satu-papan pada 2+ m terlalu berderau: uji asap 29-46 mm nominal -> hanya info.)
Ambang = §A G63d (didaftarkan sebelum data; dikalibrasi di uji mandiri sintetis g63d_selftest.py).

    python3 g63d_solve.py --tape tape.json --frames DIR [--plan g63d_plan.json] [--out solve.json]
      DIR/<cam>/*.png  (30 frame per kamera) + DIR/<cam>/camera_info.json {K, dist}
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402
import g63b_detect as D  # noqa: E402  (terkunci, tidak diubah)
import g63c_guard as GU  # noqa: E402  (penjaga identitas, tidak diubah)

MIN_FRAC = 0.5
G3_MAX_MM = 20.0     # ambang awal; nilai TERDAFTAR = §A G63d (diisi dari uji mandiri sebelum data)
G4_MAX_MM = 20.0


def T_vec(v):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:]
    return T


def v_T(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def priors(boards, T_wc, K, dist, sq):
    """{papan: dict(uv 12x2, depth)} untuk penjaga + detektor."""
    out = {}
    for n, b in boards.items():
        uv, z = G.project(T_wc, K, dist, G.to_w(b['T'], G.corners_b(sq)))
        out[n] = dict(uv=uv, depth=float(np.mean(z)))
    return out


def detect_cam(frames, boards, T_wc, K, dist, sq):
    """-> {papan: dict(uv median 12x2, std px, n_ok, n, notes)}."""
    pr = priors(boards, T_wc, K, dist, sq)
    obs = {}
    for n in boards:
        if pr[n]['depth'] <= 0:
            continue
        uvs, notes = [], {}
        for f in frames:
            gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY) if isinstance(f, str) else f
            ids, uv, note = D.detect_face(gray, pr[n]['uv'], sq)
            if ids:
                ok, gnote = GU.check(n, uv, pr, gray.shape[::-1], sq)
                if not ok:
                    note = 'penjaga ' + gnote
                    ids = []
            key = note.split(' ')[0] if not ids else 'ok'
            notes[key] = notes.get(key, 0) + 1
            if ids:
                uvs.append(np.asarray(uv, float))
        rec = dict(n=len(frames), n_ok=len(uvs), notes=notes)
        if len(uvs) >= MIN_FRAC * len(frames) and uvs:
            U = np.array(uvs)
            rec.update(uv=np.median(U, 0), std_px=float(U.std(0).max()))
        obs[n] = rec
    return obs


def fit_cam(obs, boards, T0, K, dist, sq, skip=()):
    """PnP gabungan (LM pada 6 DOF) -> T_wc, sigma px, kov (6x6, rotvec/posisi)."""
    names = [n for n in obs if 'uv' in obs[n] and n not in skip]
    if not names:
        return None
    W = np.concatenate([G.to_w(boards[n]['T'], G.corners_b(sq)) for n in names])
    U = np.concatenate([obs[n]['uv'] for n in names])

    def r(v):
        return (G.project(T_vec(v), K, dist, W)[0] - U).ravel()
    sol = least_squares(r, v_T(T0), method='lm', x_scale='jac')
    dof = max(1, len(sol.fun) - 6)
    sig = float(np.sqrt(np.sum(sol.fun ** 2) / dof))
    J = sol.jac
    cov = np.linalg.pinv(J.T @ J) * sig ** 2
    return dict(T=T_vec(sol.x), sigma_px=sig, cov=cov, boards=names, rank=int(np.linalg.matrix_rank(J)))


def lateral_mm(T_wc, K, dist, Pw, uv_obs):
    uv, z = G.project(T_wc, K, dist, Pw)
    return float(np.mean(np.linalg.norm(uv - uv_obs, axis=1) * z / K[0, 0]) * 1000)


def board_pnp_center(uv, K, dist, sq, T_wc):
    """PnP satu papan di kamera -> pusat pola di dunia."""
    obj = G.corners_b(sq).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, np.asarray(uv, np.float64), K, dist, flags=cv2.SOLVEPNP_IPPE)
    R, _ = cv2.Rodrigues(rv)
    c_b = np.array([2 * sq, 2.5 * sq, 0])
    return (T_wc[:3, :3] @ (R @ c_b + tv.ravel()) + T_wc[:3, 3])


def triangulate(uvs, cams, Ts):
    """uvs {cam: 12x2}, Ts {cam: T_wc} -> 12x3 dunia (DLT pada titik ternormalisasi, dua kamera)."""
    P, X = [], []
    for c in uvs:
        n = cv2.undistortPoints(np.asarray(uvs[c], np.float64).reshape(-1, 1, 2), cams[c]['K'], cams[c]['dist']).reshape(-1, 2)
        Tcw = np.linalg.inv(Ts[c])
        P.append(Tcw[:3])
        X.append(n)
    out = cv2.triangulatePoints(P[0], P[1], X[0].T, X[1].T)
    return (out[:3] / out[3]).T


def solve(obs_all, boards, cams, sq):
    """obs_all {cam: obs}, cams {cam: dict(K, dist, T_wc incumbent)} -> laporan dict."""
    rep = {}
    fits = {}
    for c, obs in obs_all.items():
        K, d, T0 = cams[c]['K'], cams[c]['dist'], cams[c]['T_wc']
        f = fit_cam(obs, boards, T0, K, d, sq)
        if f is None:
            rep[c] = dict(ok=False, why='tidak ada papan terdeteksi')
            continue
        fits[c] = f
        lobo = {}
        for n in f['boards']:
            g = fit_cam(obs, boards, f['T'], K, d, sq, skip=(n,))
            if g is None or len(g['boards']) < 3:
                lobo[n] = None
                continue
            lobo[n] = round(lateral_mm(g['T'], K, d, G.to_w(boards[n]['T'], G.corners_b(sq)), obs[n]['uv']), 2)
        std = np.sqrt(np.diag(f['cov']))
        dT = np.linalg.inv(T0) @ f['T']
        rep[c] = dict(n_boards=len(f['boards']), boards=f['boards'], sigma_px=round(f['sigma_px'], 3), rank=f['rank'],
                      std_rot_deg=np.degrees(std[:3]).round(3).tolist(), std_pos_mm=(1000 * std[3:]).round(1).tolist(),
                      d_incumbent_deg=round(float(np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude())), 3),
                      d_incumbent_mm=round(float(1000 * np.linalg.norm(f['T'][:3, 3] - T0[:3, 3])), 1),
                      T_wc=f['T'].round(6).tolist(), lobo_mm=lobo,
                      G3=bool(all(v is not None and v <= G3_MAX_MM for v in lobo.values()) and len(lobo) >= 4),
                      detect={n: {k: v for k, v in o.items() if k != 'uv'} for n, o in obs.items()})
    shared = [n for n in boards if all(n in fits.get(c, {}).get('boards', []) for c in cams)] if len(fits) == 2 else []
    g4 = {}
    for s in shared:
        cen, Ts = {}, {}
        for c in cams:
            K, d = cams[c]['K'], cams[c]['dist']
            g = fit_cam(obs_all[c], boards, fits[c]['T'], K, d, sq, skip=(s,))
            if g is None:
                break
            Ts[c] = g['T']
            cen[c] = board_pnp_center(obs_all[c][s]['uv'], K, d, sq, g['T'])
        if len(Ts) < 2:
            g4[s] = None
            continue
        Xt = triangulate({c: obs_all[c][s]['uv'] for c in cams}, cams, Ts)
        Xp = G.to_w(boards[s]['T'], G.corners_b(sq))
        tape_c = Xp.mean(0)
        g4[s] = dict(tri_mm=round(float(1000 * np.mean(np.linalg.norm(Xt - Xp, axis=1))), 1),
                     pnp_cam_cam_mm=round(float(1000 * np.linalg.norm(cen['rgbd'] - cen['rgbd2'])), 1))
    rep['G4'] = dict(shared=g4, ok=bool(g4) and all(v is not None and v['tri_mm'] <= G4_MAX_MM for v in g4.values()))
    return rep, fits


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tape', required=True)
    ap.add_argument('--frames', required=True)
    ap.add_argument('--plan', default=os.path.join(G.HERE, 'g63d_plan.json'))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    import g63d_tape as TP
    t = TP.load(a.tape)
    plan = json.load(open(a.plan)) if os.path.exists(a.plan) else None
    boards, meta = TP.boards_world(t, plan)
    print('kotak %.2f mm; unting residu %s (ok %s)' % (meta['sq'] * 1000, meta['plumb_res'], meta['plumb_ok']))
    for n, b in boards.items():
        print(' ', n, b['surf'], 'K1', np.round(b['k1'], 3), 'cek', b['checks'], 'OK' if b['ok'] else 'GAGAL-CEK-PITA')
    bad = [n for n, b in boards.items() if not b['ok']]
    if bad or not meta['plumb_ok']:
        print('GAGAL: cek pita tidak lulus (%s) -> ukur ulang papan itu; TIDAK dibuang diam-diam' % (bad or 'unting'))
        return 2
    cp = G.cam_params()
    cams, obs_all = {}, {}
    for c in G.CAMS:
        info = json.load(open(os.path.join(a.frames, c, 'camera_info.json')))
        cams[c] = dict(K=np.array(info['K']), dist=np.asarray(info['dist'], float), T_wc=cp[c]['T_wc'])
        frames = sorted(glob.glob(os.path.join(a.frames, c, '*.png')))
        obs_all[c] = detect_cam(frames, boards, cams[c]['T_wc'], cams[c]['K'], cams[c]['dist'], meta['sq'])
    rep, _ = solve(obs_all, boards, cams, meta['sq'])
    rep['meta'] = meta
    print(json.dumps({k: v for k, v in rep.items()}, indent=1, default=str))
    if a.out:
        json.dump(rep, open(a.out, 'w'), indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
