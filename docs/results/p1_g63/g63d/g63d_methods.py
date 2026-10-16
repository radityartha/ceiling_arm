#!/usr/bin/env python3
"""G63d LURING: bandingkan metode jangkar ekstrinsik D455 di geometri sel nyata (Monte Carlo, NOL HW).

M1  papan catur polos 4x5 @ 41 mm statis (rencana dev g63d) -- hasil uji mandiri, tidak diulang di sini.
M2  penanda ArUco/AprilTag BESAR, satu per A4 (sisi hitam TAG m), statis datar di permukaan, dijangkar pita (sama dengan
    M1: trilaterasi 2 sudut + tinggi). Visibilitas: bingkai (tepi margin), jarak <= 4.0 m, sudut <= 60 deg, sisi penanda
    terproyeksi >= 30 px (dekode 6x6 sel >= 5 px/sel), masker lengan REST, sinar env (margin) -- seperti g63d_plan.
M3  M2 + pose RELATIF rgbd<->rgbd2 dari papan polos 4x5 yang DIPEGANG operator di volume tumpang-tindih (kalibrasi stereo,
    NOL gerak lengan): kedua kamera = satu rig 6 DOF ke dunia -> jangkar kedua kamera saling membantu.
Galat: pita xy 3 mm, z 3 mm, yaw 0.5 deg per papan; sudut 0.3 px (penanda) / 0.3 px (papan genggam, median frame).
Keluaran: galat rotasi/posisi kamera dan galat lateral titik ruang kerja (median / p95), per metode, per kamera.

    python3 g63d_methods.py [--margin 0.10] [--ntag 8] [--trials 300]
"""
import argparse
import itertools
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402
import g63d_plan as P  # noqa: E402

TAG = 0.180
MAX_DIST, MAX_ANG, MIN_TAG_PX = 4.0, 60.0, 30.0
SIG_XY, SIG_Z, SIG_YAW, SIG_PX = 0.003, 0.003, np.radians(0.5), 0.3


def tag_corners(T):
    c = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0.0]]) * TAG / 2
    ctr_b = np.array([G.PAPER[0] / 2, G.PAPER[1] / 2, 0]) - np.array([(G.PAPER[0] - G.NX * P.SQ) / 2, (G.PAPER[1] - G.NY * P.SQ) / 2, 0])
    return G.to_w(T, c + ctr_b)


def tag_visibility(cands, cp, env, margin):
    trees, pts = {}, G.robot_points()
    masks = {c: G.robot_mask(c, cp, pts, margin) for c in G.CAMS}
    for b in cands:
        b['tv'] = {}
        pap = G.to_w(b['T'], G.paper_b(P.SQ))
        ctr = pap.mean(0)
        for c in G.CAMS:
            K, d, T = cp[c]['K'], cp[c]['dist'], cp[c]['T_wc']
            o = T[:3, 3]
            dist = float(np.linalg.norm(ctr - o))
            ang = float(np.degrees(np.arccos(abs((ctr - o) / dist @ b['T'][:3, 2]))))
            uv, z = G.project(T, K, d, pap)
            m_px = margin * K[0, 0] / max(z.min(), 0.1)
            ok = bool((z > 0.1).all() and (uv[:, 0] >= m_px).all() and (uv[:, 0] < G.IMG_W - m_px).all()
                      and (uv[:, 1] >= m_px).all() and (uv[:, 1] < G.IMG_H - m_px).all()) and dist <= MAX_DIST and ang <= MAX_ANG
            if ok:
                tu, _ = G.project(T, K, d, tag_corners(b['T']))
                side = float(min(np.linalg.norm(tu - np.roll(tu, 1, 0), axis=1)))
                poly = np.zeros((G.IMG_H, G.IMG_W), np.uint8)
                cv2.fillConvexPoly(poly, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
                ok = side >= MIN_TAG_PX and not (poly & masks[c]).any()
            if ok:
                if b['z'] not in trees:
                    trees[b['z']] = cKDTree(env[env[:, 2] > b['z'] + margin])
                for p in np.r_[pap, [ctr]]:
                    L = np.linalg.norm(p - o)
                    s = o + np.outer(np.arange(0.05, L, 0.02), (p - o) / L)
                    if trees[b['z']].query(s)[0].min() < margin:
                        ok = False
                        break
            b['tv'][c] = ok
    return cands


def pick_tags(cands, ntag, seed=0):
    """Pilih ntag penanda: tiap kamera sebanyak mungkin + sebaran besar; jarak antar pusat >= 0.40 m; stok platform."""
    vis = [b for b in cands if any(b['tv'].values())]
    rng = np.random.default_rng(seed)
    best, best_s = None, -1
    for it in range(300):
        S = []
        for b in rng.permutation(vis):
            surf = [x['surf'] for x in S] + [b['surf']]
            if any(surf.count(k) > n for k, n in P.STOCK.items()) or any(a in surf and c in surf for a, c in P.EXCLUSIVE):
                continue
            if any(np.linalg.norm(np.r_[b['cx'] - x['cx'], b['cy'] - x['cy'], b['z'] - x['z']]) < 0.40 for x in S):
                continue
            S.append(b)
            if len(S) == ntag:
                break
        sc = 0
        for c in G.CAMS:
            Q = np.array([[x['cx'], x['cy'], x['z']] for x in S if x['tv'][c]])
            if len(Q) < 3:
                sc -= 100
                continue
            sc += min(len(Q), 6) + np.sqrt(np.linalg.det(np.cov(Q[:, :2].T)) + 1e-9) * 10
        if sc > best_s:
            best, best_s = S, sc
    return best


def stereo_rel_sigma(cp, n_views=25, seed=0):
    """Simulasi papan genggam 4x5 @ 41 mm di volume tumpang-tindih -> galat pose relatif rgbd->rgbd2 (Monte Carlo)."""
    rng = np.random.default_rng(seed)
    T1, T2 = cp['rgbd']['T_wc'], cp['rgbd2']['T_wc']
    mid = (T1[:3, 3] + T2[:3, 3]) / 2
    views = []
    tries = 0
    while len(views) < n_views and tries < 20000:
        tries += 1
        c = np.r_[rng.uniform(0.1, 1.5), rng.uniform(-0.6, 0.7), rng.uniform(0.4, 1.1)]
        n = (T1[:3, 3] - c) / np.linalg.norm(T1[:3, 3] - c) + (T2[:3, 3] - c) / np.linalg.norm(T2[:3, 3] - c)
        n = n / np.linalg.norm(n)
        n = Rotation.from_rotvec(np.radians(15) * rng.normal(size=3) / 1.7).as_matrix() @ n
        z = -n                                              # z_b menjauhi kamera
        x = np.cross([0, 0, 1.0], z)
        x /= np.linalg.norm(x)
        R = np.c_[x, np.cross(z, x), z] @ Rotation.from_euler('z', rng.uniform(0, 2 * np.pi)).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = c - R @ np.array([2 * P.SQ, 2.5 * P.SQ, 0])
        W = G.to_w(T, G.corners_b(P.SQ))
        ok = True
        for cam in G.CAMS:
            K, d, Tc = cp[cam]['K'], cp[cam]['dist'], cp[cam]['T_wc']
            uv, zz = G.project(Tc, K, d, W)
            g = uv.reshape(4, 3, 2)
            side = min(np.linalg.norm(np.diff(g, axis=0), axis=2).min(), np.linalg.norm(np.diff(g, axis=1), axis=2).min())
            ray = (c - Tc[:3, 3]) / np.linalg.norm(c - Tc[:3, 3])
            ang = np.degrees(np.arccos(abs(ray @ R[:, 2])))
            if not ((zz > 0).all() and (uv[:, 0] > 40).all() and (uv[:, 0] < G.IMG_W - 40).all() and (uv[:, 1] > 40).all()
                    and (uv[:, 1] < G.IMG_H - 40).all() and side >= 9.5 and ang <= 55 and np.linalg.norm(c - Tc[:3, 3]) <= 2.2):
                ok = False
        if ok:
            views.append(T)
    if len(views) < 8:
        return None, len(views)
    # MC: noise piksel -> fit T_rel + pose papan (bebas) -> sebaran T_rel
    K1, d1, K2, d2 = cp['rgbd']['K'], cp['rgbd']['dist'], cp['rgbd2']['K'], cp['rgbd2']['dist']
    Trel_true = np.linalg.inv(T1) @ T2          # rgbd2 di bingkai rgbd
    obs_true = [(G.project(T1, K1, d1, G.to_w(T, G.corners_b(P.SQ)))[0], G.project(T2, K2, d2, G.to_w(T, G.corners_b(P.SQ)))[0])
                for T in views]
    B = [np.linalg.inv(T1) @ T for T in views]   # papan di bingkai rgbd
    errs = []
    for t in range(40):
        obs = [(a + rng.normal(scale=SIG_PX, size=a.shape), b + rng.normal(scale=SIG_PX, size=b.shape)) for a, b in obs_true]

        def r(x):
            Tr = np.eye(4)
            Tr[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
            Tr[:3, 3] = x[3:6]
            res = []
            for i, (oa, ob) in enumerate(obs):
                Tb = np.eye(4)
                Tb[:3, :3] = Rotation.from_rotvec(x[6 + 6 * i:9 + 6 * i]).as_matrix()
                Tb[:3, 3] = x[9 + 6 * i:12 + 6 * i]
                Pc = G.to_w(Tb, G.corners_b(P.SQ))
                res.append((G.project(np.eye(4), K1, d1, Pc)[0] - oa).ravel())
                res.append((G.project(Tr, K2, d2, Pc)[0] - ob).ravel())
            return np.concatenate(res)
        x0 = np.r_[Rotation.from_matrix(Trel_true[:3, :3]).as_rotvec(), Trel_true[:3, 3]]
        for Tb in B:
            x0 = np.r_[x0, Rotation.from_matrix(Tb[:3, :3]).as_rotvec(), Tb[:3, 3]]
        sol = least_squares(r, x0, method='lm')
        Tr = np.eye(4)
        Tr[:3, :3] = Rotation.from_rotvec(sol.x[:3]).as_matrix()
        Tr[:3, 3] = sol.x[3:6]
        dT = np.linalg.inv(Trel_true) @ Tr
        errs.append((np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude()), 1000 * np.linalg.norm(dT[:3, 3])))
    e = np.array(errs)
    return dict(n_views=len(views), rot_med=float(np.median(e[:, 0])), rot_p95=float(np.percentile(e[:, 0], 95)),
                pos_med=float(np.median(e[:, 1])), pos_p95=float(np.percentile(e[:, 1], 95)), Trel=Trel_true), len(views)


def mc(tags, cp, trials, stereo=None, seed=0):
    """MC galat kamera. stereo=None: per kamera bebas; stereo=dict: rig (T_w,rgbd + T_rel prior)."""
    rng = np.random.default_rng(seed)
    X = np.array([[x, y, z] for x in np.arange(-0.5, 2.6, 0.25) for y in np.arange(-1.2, 1.3, 0.25) for z in (0, 0.77, 1.3)])
    out = {c: [] for c in G.CAMS}
    Tt = {c: cp[c]['T_wc'] for c in G.CAMS}
    true_c = {c: [tag_corners(b['T']) for b in tags if b['tv'][c]] for c in G.CAMS}
    for t in range(trials):
        tape = []
        for b in tags:
            Tn = b['T'].copy()
            Tn[:3, :3] = Rotation.from_euler('z', rng.normal(scale=SIG_YAW)).as_matrix() @ b['T'][:3, :3]
            Tn[:3, 3] += np.r_[rng.normal(scale=SIG_XY, size=2), rng.normal(scale=SIG_Z)]
            tape.append(Tn)
        W = {c: [tag_corners(Tn) for Tn, b in zip(tape, tags) if b['tv'][c]] for c in G.CAMS}
        U = {c: [G.project(Tt[c], cp[c]['K'], cp[c]['dist'], w)[0] + rng.normal(scale=SIG_PX, size=(4, 2)) for w in true_c[c]]
             for c in G.CAMS}
        est = {}
        if stereo is None:
            for c in G.CAMS:
                if len(W[c]) < 2:
                    continue
                Wc, Uc = np.concatenate(W[c]), np.concatenate(U[c])
                x0 = np.r_[Rotation.from_matrix(Tt[c][:3, :3]).as_rotvec(), Tt[c][:3, 3]]
                sol = least_squares(lambda x: (G.project(_T(x), cp[c]['K'], cp[c]['dist'], Wc)[0] - Uc).ravel(), x0, method='lm')
                est[c] = _T(sol.x)
        else:
            # T_rel terukur = benar diganggu galat stereo (sampel)
            dR = Rotation.from_rotvec(np.radians(stereo['rot_med'] / 0.83) * rng.normal(size=3) / np.sqrt(3)).as_matrix()
            Trel = stereo['Trel'].copy()
            Trel[:3, :3] = Trel[:3, :3] @ dR
            Trel[:3, 3] += stereo['pos_med'] / 0.83 / 1000 * rng.normal(size=3) / np.sqrt(3)
            W1, U1 = np.concatenate(W['rgbd']), np.concatenate(U['rgbd'])
            W2, U2 = (np.concatenate(W['rgbd2']), np.concatenate(U['rgbd2'])) if W['rgbd2'] else (np.zeros((0, 3)), np.zeros((0, 2)))

            def r(x):
                T1 = _T(x)
                res = [(G.project(T1, cp['rgbd']['K'], cp['rgbd']['dist'], W1)[0] - U1).ravel()]
                if len(W2):
                    res.append((G.project(T1 @ Trel, cp['rgbd2']['K'], cp['rgbd2']['dist'], W2)[0] - U2).ravel())
                return np.concatenate(res)
            x0 = np.r_[Rotation.from_matrix(Tt['rgbd'][:3, :3]).as_rotvec(), Tt['rgbd'][:3, 3]]
            T1 = _T(least_squares(r, x0, method='lm').x)
            est = {'rgbd': T1, 'rgbd2': T1 @ Trel}
        for c, Te in est.items():
            dT = np.linalg.inv(Tt[c]) @ Te
            uv, dep = G.project(Tt[c], cp[c]['K'], cp[c]['dist'], X)
            m = (dep > 0.3) & (dep < 3.5) & (uv[:, 0] > 0) & (uv[:, 0] < G.IMG_W) & (uv[:, 1] > 0) & (uv[:, 1] < G.IMG_H)
            uv2, _ = G.project(Te, cp[c]['K'], cp[c]['dist'], X[m])
            pe = np.linalg.norm(uv2 - uv[m], axis=1) * dep[m] / cp[c]['K'][0, 0] * 1000
            out[c].append((np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude()), 1000 * np.linalg.norm(dT[:3, 3]),
                           np.median(pe), np.percentile(pe, 95)))
    return {c: (dict(rot_med=round(float(np.median([v[0] for v in o])), 3), rot_p95=round(float(np.percentile([v[0] for v in o], 95)), 3),
                     pos_med=round(float(np.median([v[1] for v in o])), 1), pos_p95=round(float(np.percentile([v[1] for v in o], 95)), 1),
                     pt_med=round(float(np.median([v[2] for v in o])), 1), pt_p95=round(float(np.percentile([v[3] for v in o], 95)), 1))
                if o else None) for c, o in out.items()}


def _T(x):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
    T[:3, 3] = x[3:6]
    return T


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--margin', type=float, default=0.10)
    ap.add_argument('--ntag', type=int, default=8)
    ap.add_argument('--trials', type=int, default=300)
    ap.add_argument('--out', default=os.path.join(G.HERE, 'g63d_methods.json'))
    a = ap.parse_args()
    cp = G.cam_params()
    env = P.env_points()
    cands = P.candidates(env)
    tag_visibility(cands, cp, env, a.margin)
    for c in G.CAMS:
        print(c, 'penanda terlihat', sum(b['tv'][c] for b in cands), {s: sum(b['tv'][c] for b in cands if b['surf'] == s) for s in P.SURF},
              flush=True)
    print('dua kamera', sum(all(b['tv'].values()) for b in cands), flush=True)
    res = dict(margin=a.margin, ntag=a.ntag)
    st, nv = stereo_rel_sigma(cp)
    print('stereo papan genggam: pose layak', nv, st and {k: round(v, 3) for k, v in st.items() if k != 'Trel'}, flush=True)
    res['stereo'] = st and {k: v for k, v in st.items() if k != 'Trel'}
    for n in (a.ntag, a.ntag + 4):
        tags = pick_tags(cands, n)
        print('\nM2 %d penanda:' % n, [(b['surf'], b['cx'], b['cy'], b['z'], {c: b['tv'][c] for c in G.CAMS}) for b in tags], flush=True)
        r2 = mc(tags, cp, a.trials)
        print('  M2 per kamera:', r2, flush=True)
        res['M2_%d' % n] = dict(tags=[(b['surf'], b['cx'], b['cy'], b['z'], b['tv']) for b in tags], err=r2)
        if st:
            r3 = mc(tags, cp, a.trials, stereo=st)
            print('  M3 rig (M2 + stereo):', r3, flush=True)
            res['M3_%d' % n] = r3
    json.dump(res, open(a.out, 'w'), indent=1, default=str)
    return 0


if __name__ == '__main__':
    sys.exit(main())
