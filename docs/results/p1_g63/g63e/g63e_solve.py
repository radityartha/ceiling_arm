#!/usr/bin/env python3
"""G63e solver gabungan M3: penanda ArUco statis berjangkar pita + papan polos genggam (stereo), NOL FK sendi.

Anu: T_w,rgbd (6), T_rel = rgbd2 di bingkai rgbd (6), sisi kotak papan genggam s (1, BEBAS -> dibandingkan angka ukur
operator), pose papan B_k di bingkai rgbd per tampilan dua-kamera (6 tiap). Residu (px, bobot sama): 4 sudut tiap penanda
di tiap kamera (sudut dunia = Procrustes kotak penanda TERUKUR ke sudut pita) + 12 sudut papan di kedua kamera.
Gerbang (ambang TERDAFTAR §A G63e, dikalibrasi di uji mandiri g63e_selftest.py SEBELUM data):
  G3 tinggal-satu-penanda (LOTO): penanda i dibuang, rig dipaskan ulang (papan tetap), galat lateral sudut i (mm,
     = |duv| x kedalaman / f, rata-rata 4 sudut, maks atas kamera yang melihatnya) <= G3_MAX_MM.
  G5 sisi kotak papan terestimasi vs ukur operator: |s - s_ukur| <= G5_MAX_MM (bila angka ukur ada).
  Info: sigma px, std rig, T_rel dari papan saja vs dari penanda saja (per kamera terpisah) -> beda sudut/posisi.

    python3 g63e_solve.py --tape tape_e.json --frames DIR --board-views DIR2 [--plan g63e_plan.json] [--out solve.json]
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402
import g63d_common as GD  # noqa: E402

G3_MAX_MM = 15.0     # ambang awal; nilai TERDAFTAR = §A G63e (diisi dari uji mandiri sebelum data)
G5_MAX_MM = 0.5
SQ0 = 0.041
LBL_MAX_DEG, LBL_MAX_M = 20.0, 0.30   # B1 (incumbent ~1.5 deg / 5-15 cm off; label salah = puluhan deg)
VIEW_RMS_MAX = 1.5                     # B2 px RMS per pose papan sesudah fit


def T_vec(v):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:6]
    return T


def v_T(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def board_pnp(uv, K, dist, sq):
    import cv2
    ok, rv, tv = cv2.solvePnP(GD.corners_b(sq).astype(np.float64), np.asarray(uv, np.float64), K, dist,
                              flags=cv2.SOLVEPNP_IPPE)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = tv.ravel()
    return T


def fit(tag_obs, tags, board_views, cams, T1_0, Trel_0, skip=(), use_board=True, sq0=None):
    """tag_obs {cam: {id: uv 4x2}}, tags {id: 4 sudut dunia 4x3 (Procrustes pita, sisi terukur)}, board_views [ {cam: uv 12x2} (dua kamera) ], cams {cam: K, dist}.
    -> dict(T1, Trel, sq, B, sigma_px, cov rig 12x12, res per blok)."""
    sq0 = SQ0 if sq0 is None else sq0
    K1, d1, K2, d2 = cams['rgbd']['K'], cams['rgbd']['dist'], cams['rgbd2']['K'], cams['rgbd2']['dist']
    Pw = tags
    tw = {c: [(i, Pw[i], uv) for i, uv in tag_obs[c].items() if i in tags and i not in skip] for c in E.CAMS}
    bv = board_views if use_board else []
    x0 = np.r_[v_T(T1_0), v_T(Trel_0), sq0 if bv else []]
    for v in bv:
        x0 = np.r_[x0, v_T(board_pnp(v['rgbd'], K1, d1, sq0))]

    def unpack(x):
        T1, Tr = T_vec(x[:6]), T_vec(x[6:12])
        if not bv:
            return T1, Tr, None, []
        return T1, Tr, x[12], [T_vec(x[13 + 6 * k:19 + 6 * k]) for k in range(len(bv))]

    def r(x):
        T1, Tr, sq, B = unpack(x)
        out = []
        for i, P, uv in tw['rgbd']:
            out.append((E.project(T1, K1, d1, P)[0] - uv).ravel())
        T2 = T1 @ Tr
        for i, P, uv in tw['rgbd2']:
            out.append((E.project(T2, K2, d2, P)[0] - uv).ravel())
        for v, Tb in zip(bv, B):
            Pc = GD.to_w(Tb, GD.corners_b(sq))
            out.append((E.project(np.eye(4), K1, d1, Pc)[0] - v['rgbd']).ravel())
            out.append((E.project(Tr, K2, d2, Pc)[0] - v['rgbd2']).ravel())
        return np.concatenate(out)
    sol = least_squares(r, x0, method='lm', x_scale='jac')
    T1, Tr, sq, B = unpack(sol.x)
    dof = max(1, len(sol.fun) - len(sol.x))
    sig = float(np.sqrt(np.sum(sol.fun ** 2) / dof))
    J = sol.jac
    cov = np.linalg.pinv(J.T @ J) * sig ** 2
    return dict(T1=T1, Trel=Tr, T2=T1 @ Tr, sq=sq, B=B, sigma_px=sig, cov=cov[:13, :13] if bv else cov[:12, :12],
                rank=int(np.linalg.matrix_rank(J)), n_par=len(sol.x), ok=bool(sol.success))




def lateral_mm(T_wc, K, dist, Pw, uv):
    p, z = E.project(T_wc, K, dist, Pw)
    return float(np.mean(np.linalg.norm(p - uv, axis=1) * z / K[0, 0]) * 1000)


def label_guard(board_views, cams, Trel_inc, sq=SQ0):
    """B1 penjaga label stereo: PnP papan di tiap kamera, rgbd2 dipetakan ke bingkai rgbd lewat T_rel incumbent; beda rotasi
    > LBL_MAX_DEG atau beda pusat > LBL_MAX_M -> pose dibuang (label terbalik/cermin = puluhan-180 deg; uji mandiri dev40: SETIAP
    seed meledak = tepat satu label salah). -> (pose lolos, [(indeks, alasan)])."""
    keep, rej = [], []
    c_b = np.array([2 * sq, 2.5 * sq, 0, 1.0])
    for k, v in enumerate(board_views):
        B1 = board_pnp(v['rgbd'], cams['rgbd']['K'], cams['rgbd']['dist'], sq)
        B2 = Trel_inc @ board_pnp(v['rgbd2'], cams['rgbd2']['K'], cams['rgbd2']['dist'], sq)
        ang = float(np.degrees(Rotation.from_matrix(B1[:3, :3].T @ B2[:3, :3]).magnitude()))
        dp = float(np.linalg.norm((B1 @ c_b - B2 @ c_b)[:3]))
        if ang > LBL_MAX_DEG or dp > LBL_MAX_M:
            rej.append((k, 'label stereo %.1f deg %.3f m' % (ang, dp)))
        else:
            keep.append(v)
    return keep, rej


def view_rms(f, views, cams):
    out = []
    for v, Tb in zip(views, f['B']):
        Pc = GD.to_w(Tb, GD.corners_b(f['sq']))
        r1 = E.project(np.eye(4), cams['rgbd']['K'], cams['rgbd']['dist'], Pc)[0] - v['rgbd']
        r2 = E.project(f['Trel'], cams['rgbd2']['K'], cams['rgbd2']['dist'], Pc)[0] - v['rgbd2']
        out.append(float(np.sqrt(np.mean(np.r_[r1, r2] ** 2) * 2)))
    return out


def solve(tag_obs, tags, board_views, cams, T_inc, sq_meas=None, loto=True):
    """T_inc {cam: T_wc incumbent} -> (laporan, fit). Pose papan: B1 penjaga label, lalu B2 residu (buang RMS > VIEW_RMS_MAX,
    fit ulang sampai bersih). Semua yang dibuang dilaporkan."""
    T1_0 = T_inc['rgbd']
    Trel_0 = np.linalg.inv(T_inc['rgbd']) @ T_inc['rgbd2']
    board_views, rejected = label_guard(board_views, cams, Trel_0)
    f = fit(tag_obs, tags, board_views, cams, T1_0, Trel_0)
    while board_views:
        rms = view_rms(f, board_views, cams)
        if max(rms) <= VIEW_RMS_MAX:
            break
        k = int(np.argmax(rms))
        rejected.append(('sisa', 'residu %.2f px' % rms[k]))
        board_views = board_views[:k] + board_views[k + 1:]
        f = fit(tag_obs, tags, board_views, cams, T1_0, Trel_0)
    rep = dict(sigma_px=round(f['sigma_px'], 3), rank=f['rank'], n_par=f['n_par'], n_board_views=len(board_views),
               board_rejected=rejected,
               n_tags={c: sorted(i for i in tag_obs[c] if i in tags) for c in E.CAMS},
               sq_est_mm=None if f['sq'] is None else round(1000 * f['sq'], 3))
    std = np.sqrt(np.diag(f['cov']))
    rep['std_rgbd_rot_deg'] = np.degrees(std[:3]).round(3).tolist()
    rep['std_rgbd_pos_mm'] = (1000 * std[3:6]).round(1).tolist()
    rep['std_rel_rot_deg'] = np.degrees(std[6:9]).round(3).tolist()
    rep['std_rel_pos_mm'] = (1000 * std[9:12]).round(1).tolist()
    for c, T in (('rgbd', f['T1']), ('rgbd2', f['T2'])):
        dT = np.linalg.inv(T_inc[c]) @ T
        rep['d_incumbent_' + c] = dict(deg=round(float(np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude())), 3),
                                       mm=round(float(1000 * np.linalg.norm(T[:3, 3] - T_inc[c][:3, 3])), 1))
    if loto:
        lo = {}
        for i in tags:
            seen = [c for c in E.CAMS if i in tag_obs[c]]
            if not seen:
                continue
            g = fit(tag_obs, tags, board_views, cams, f['T1'], f['Trel'], skip=(i,))
            Pw = tags[i]
            lo[i] = round(max(lateral_mm(g['T1'] if c == 'rgbd' else g['T2'], cams[c]['K'], cams[c]['dist'], Pw, tag_obs[c][i])
                              for c in seen), 2)
        rep['loto_mm'] = lo
        rep['G3'] = bool(lo) and all(v <= G3_MAX_MM for v in lo.values())
    if sq_meas is not None and f['sq'] is not None:
        rep['G5_sq_err_mm'] = round(1000 * abs(f['sq'] - sq_meas), 3)
        rep['G5'] = rep['G5_sq_err_mm'] <= G5_MAX_MM
    # info: T_rel dari penanda saja (rig tanpa papan) vs gabungan
    if board_views:
        g = fit(tag_obs, tags, [], cams, f['T1'], f['Trel'], use_board=False)
        dT = np.linalg.inv(f['Trel']) @ g['Trel']
        rep['info_rel_tags_only_vs_joint'] = dict(deg=round(float(np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude())), 3),
                                                  mm=round(float(1000 * np.linalg.norm(dT[:3, 3])), 1))
    return rep, f
