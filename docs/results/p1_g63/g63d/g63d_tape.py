"""G63d: angka pita operator -> pose dunia papan statis (trilaterasi, tanpa FK sendi).

Berkas pita (JSON, angka HARFIAH operator, cm):
  {"sq_mm": {"4kotak": 164.0, "5kotak": 205.0},          # panjang terukur 4 / 5 kotak -> ukuran kotak
   "plumb": {"P1-P2": 80.0, "P1-P3": ..., ...},          # jarak antar tanda unting lantai (>= 5 dari 6 pasang)
   "floor_z_cm": {"P1": 182.4, ...},                     # opsional: lantai -> sambungan basis/link1 (info z lantai)
   "boards": {"S1": {"surf": "lantai", "z_cm": 0,
                     "K1": {"P2": 123.4, "P3": 98.7, "P4": 150.2}, "K2": {...}},
              "S2": {"surf": "kotak_B", "z_cm": 48.0,         # platform: E1/E2 = sudut ATAS platform, diunting ke lantai
                     "E1": {"P2": ..., "P3": ..., "P4": ...}, "E2": {...},
                     "E1E2_top": 42.3,                         # jarak E1-E2 diukur di atas (cek)
                     "K1": {"E1": 10.2, "E2": 35.1}, "K2": {"E1": ..., "E2": ...}}}}
Keluaran: titik unting di bingkai lokal -> Procrustes 2D (tanpa skala) ke URDF P1..P4; K1/K2 tiap papan; pose papan.
Cek (gagal = papan ditandai, TIDAK dibuang diam-diam): residu trilaterasi, |K1K2| vs 5 kotak, |E1E2| atas vs lantai.
"""
import json

import numpy as np
from scipy.optimize import least_squares

import g63d_common as G

TRI_RES_MAX_M = 0.005      # RMS residu jarak trilaterasi (3 acuan, 2 anu) -> salah baca terdeteksi
K12_TOL_M = 0.005          # |K1K2| terhitung vs 5 kotak terukur
E12_TOL_M = 0.005          # |E1E2| atas vs dari lantai
PLUMB_FIT_MAX_M = 0.010    # residu Procrustes titik unting vs URDF (rel = URDF <= 1 cm, G34)


def square_m(t):
    s = t['sq_mm']
    return float(np.mean([s['4kotak'] / 4, s['5kotak'] / 5])) / 1000.0


def plumb_world(t):
    """Posisi P1..P4 terukur (bingkai lokal dari jarak) dipaskan kaku ke URDF. -> {P: xy}, residu maks (m), rms jarak (m)."""
    names = sorted(G.PLUMB)
    D = {tuple(sorted(k.split('-'))): v / 100.0 for k, v in t['plumb'].items()}
    x0 = np.array([G.PLUMB[n] for n in names]).ravel()

    def r(x):
        X = x.reshape(-1, 2)
        return [np.linalg.norm(X[names.index(a)] - X[names.index(b)]) - d for (a, b), d in D.items()]
    sol = least_squares(r, x0)
    X = sol.x.reshape(-1, 2)
    U = np.array([G.PLUMB[n] for n in names])
    mx, mu = X.mean(0), U.mean(0)
    H = (X - mx).T @ (U - mu)
    W, _, Vt = np.linalg.svd(H)
    R = (W @ Vt).T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = (W @ Vt).T
    Xw = (X - mx) @ R.T + mu
    res = np.linalg.norm(Xw - U, axis=1)
    return ({n: Xw[i] for i, n in enumerate(names)}, dict(zip(names, res.round(4).tolist())),
            float(np.sqrt(np.mean(np.square(sol.fun)))) if len(sol.fun) else 0.0)


def trilat(refs, dists_m, guess):
    """Titik 2D dari >= 2 acuan. -> xy, rms residu (m) (0 bila hanya 2 acuan = tanpa cek)."""
    P = np.array(refs)
    d = np.array(dists_m)
    sol = least_squares(lambda x: np.linalg.norm(P - x, axis=1) - d, np.asarray(guess, float))
    return sol.x, float(np.sqrt(np.mean(sol.fun ** 2))) if len(d) > 2 else 0.0


def boards_world(t, plan=None):
    """-> {papan: dict(T, z, k1, k2, checks{...}, ok)}; plan (g63d_plan.json) memberi tebakan awal sisi (ambiguitas cermin)."""
    sq = square_m(t)
    Pw, pres, prms = plumb_world(t)
    guess = {b['name']: b for b in (plan or {}).get('boards', [])}
    out = {}
    for name, b in t['boards'].items():
        z = b['z_cm'] / 100.0
        chk, refs = {}, dict(Pw)
        g = guess.get(name)
        gk = {'K1': g['K1'][:2], 'K2': g['K2'][:2]} if g else {'K1': [0.8, 0.0], 'K2': [0.8, 0.0]}
        if 'E1' in b:                       # platform: sudut atas -> lantai via unting
            for e in ('E1', 'E2'):
                xy, rr = trilat([Pw[p] for p in b[e]], [v / 100 for v in b[e].values()], gk['K1'])
                refs[e] = xy
                chk['tri_' + e] = round(rr, 4)
            chk['E1E2_err'] = round(abs(np.linalg.norm(refs['E1'] - refs['E2']) - b['E1E2_top'] / 100), 4)
        k = {}
        for kk in ('K1', 'K2'):
            xy, rr = trilat([refs[p] for p in b[kk]], [v / 100 for v in b[kk].values()], gk[kk])
            k[kk] = xy
            chk['tri_' + kk] = round(rr, 4)
        T, d12 = G.pose_from_k(k['K1'], k['K2'], z)
        chk['K1K2_err'] = round(abs(d12 - 5 * sq), 4)
        ok = (all(v <= TRI_RES_MAX_M for kk, v in chk.items() if kk.startswith('tri_'))
              and chk['K1K2_err'] <= K12_TOL_M and chk.get('E1E2_err', 0) <= E12_TOL_M)
        out[name] = dict(T=T, z=z, k1=k['K1'], k2=k['K2'], surf=b['surf'], checks=chk, ok=bool(ok))
    return out, dict(sq=sq, plumb_res=pres, plumb_rms=prms, plumb_ok=max(pres.values()) <= PLUMB_FIT_MAX_M)


def load(path):
    return json.load(open(path))
