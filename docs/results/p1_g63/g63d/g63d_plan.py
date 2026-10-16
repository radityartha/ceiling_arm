#!/usr/bin/env python3
"""G63d tugas 1 (LURING): rencana penempatan papan polos statis.

Kandidat: pusat lembar tiap 5 cm di empat permukaan (lantai z 0, laci ~0.70, meja kerja 0.77, meja tinggi g2 0.98 [operator]),
yaw 0/90 deg. Per kamera (incumbent G34 + K/dist camera_info sesi jari) papan TERLIHAT bila SEMUA:
  V1 4 sudut lembar di bingkai dengan tepi = MARGIN m (galat incumbent) pada kedalaman papan;
  V2 jarak kamera-pusat <= 2.5 m;  V3 sudut pandang (normal papan vs sinar) <= 55 deg;
  V4 lembar tidak menyentuh masker lengan REST/gantry (dilatasi MARGIN m);
  V5 sinar kamera -> 4 sudut + pusat lembar berjarak >= MARGIN m dari voksel peta env (G34, z >= 0.30; voksel laci LAMA
     dibuang, laci SEKARANG ditambah sebagai kotak); voksel z <= permukaan + MARGIN diabaikan (penyangga + galat z peta ~5 cm; benda kecil di atas permukaan dibersihkan operator; di bawah
     bidang papan tidak bisa menghalangi kamera dari atas);
  V6 sisi kotak terproyeksi terkecil >= 9.5 px (HW jari 10-16, detektor terkunci pada frame tersimpan: sukses terkecil
     9.2 px, SEMUA <= 8.8 px gagal 'SB tidak menemukan grid' -> V2 2.5 m saja tidak cukup).
Pilih 6 papan: tiap kamera >= 4, >= 2 dilihat keduanya, jarak pusat antar papan >= 0.40 m, sebaran per kamera >= 1.0 m;
tujuan = minimalkan galat kamera terprediksi (PnP linear: 0.5 px + pita 3 mm / 0.5 deg per papan), terburuk dari dua kamera.

    python3 g63d_plan.py [--margin 0.10] [--out g63d_plan.json] [--overlay-dir DIR]
"""
import argparse
import itertools
import json
import os
import sys

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402

SQ = 0.041
MAX_DIST, MAX_ANG, MIN_SQ_PX = 2.5, 55.0, 9.5
SIG_PX, SIG_TAPE, SIG_YAW, SIG_Z = 0.5, 0.003, np.radians(0.5), 0.003
OLD_DRAWER = ((0.25, 0.95), (-0.10, 0.55), 0.80)      # voksel basi (laci dipindah sesudah peta 10-07; frame 10-16: lantai)
DRAWER = ((0.65, 0.98), (0.40, 0.72), 0.70)           # triangulasi dua kamera frame B-R0 10-16 (+-5 cm), tinggi DUGAAN
SURF = {   # nama: (x-range, y-range, z, catatan)
    'lantai': (None, None, 0.0, 'z 0'),
    'laci': (DRAWER[0], DRAWER[1], DRAWER[2], 'atas lemari laci, tinggi diukur operator'),
    'meja_kerja': (None, None, 0.77, 'operator 0.770'),
    'meja_tinggi': (None, None, 0.98, 'operator 0.98'),
    # kotak CAM-6DOF di lantai (operator 10-14: A 310 / B 480 mm +-2, atas 470x350 / 423x268 mm, kokoh, rata, stabil)
    'kotak_A': ('kotak', (0.470, 0.350), 0.31, 'kotak A di lantai, papan di tengah atas'),
    'kotak_B': ('kotak', (0.423, 0.268), 0.48, 'kotak B di lantai, papan di tengah atas'),
    'laci_pindah': ('kotak', (0.33, 0.32), 0.70, 'lemari laci DIPINDAH sementara (izin operator), papan di atas'),
}
STOCK = {'kotak_A': 1, 'kotak_B': 1, 'laci_pindah': 1}   # tiap platform bergerak hanya satu
EXCLUSIVE = [('laci', 'laci_pindah')]                    # laci di tempat ATAU dipindah


def rest_with_rails(rails):
    """(model, q) REST dengan rel (g1, g2) m; None = keadaan G63d (1.40 / 0.0009)."""
    model, q = G.rest_q()
    if rails is not None:
        for n, v in zip(('t1_linear_joint', 't2_linear_joint'), rails):
            q[model.joints[model.getJointId(n)].idx_q] = v
    return q, model


def env_points():
    m = np.load(G.ENV_MAP)
    c = m['centers'].astype(float)
    (x0, x1), (y0, y1), z1 = OLD_DRAWER
    c = c[~((c[:, 0] > x0) & (c[:, 0] < x1) & (c[:, 1] > y0) & (c[:, 1] < y1) & (c[:, 2] < z1))]
    (x0, x1), (y0, y1), zt = DRAWER
    g = np.mgrid[x0:x1:0.02, y0:y1:0.02, 0.30:zt:0.02].reshape(3, -1).T
    return np.r_[c, g]


def surface_rect(c, band, box):
    """Kotak permukaan dari kolom voksel yang puncaknya di band z, dalam kotak cari; susut 3 cm."""
    s = c[(c[:, 0] > box[0]) & (c[:, 0] < box[1]) & (c[:, 1] > box[2]) & (c[:, 1] < box[3]) & (c[:, 2] < 1.45)]
    k = {}
    for p in s:
        key = (round(p[0] / 0.04), round(p[1] / 0.04))
        k[key] = max(k.get(key, -1), p[2])
    xy = np.array([(a * 0.04, b * 0.04) for (a, b), z in k.items() if band[0] <= z <= band[1]])
    lo, hi = np.percentile(xy, 5, axis=0) + 0.03, np.percentile(xy, 95, axis=0) - 0.03
    return (lo[0], hi[0]), (lo[1], hi[1])


def board_T(cx, cy, z, psi):
    R = G.R_board(psi)
    k1 = np.array([cx, cy]) - (R[:2, :2] @ np.array([2 * SQ, 2.5 * SQ]))
    return G.T_wb(k1, z, psi)


def candidates(env):
    SURF['meja_kerja'] = (*surface_rect(env, (0.72, 0.86), (1.0, 2.6, 0.2, 1.1)), 0.77, SURF['meja_kerja'][3])
    SURF['meja_tinggi'] = (*surface_rect(env, (0.92, 1.15), (0.5, 1.5, -1.35, -0.7)), 0.98, SURF['meja_tinggi'][3])
    floor_tree = cKDTree(env[env[:, 2] < 1.5][:, :2])
    out = []
    for name, (xr, yr, z, _) in SURF.items():
        box = xr == 'kotak'
        xs = np.arange(-1.2, 3.0, 0.05) if xr is None or box else np.arange(xr[0], xr[1] + 1e-9, 0.05)
        ys = np.arange(-2.0, 2.0, 0.05) if yr is None or box else np.arange(yr[0], yr[1] + 1e-9, 0.05)
        for cx, cy in itertools.product(xs, ys):
            for psi in (0.0, np.pi / 2):
                T = board_T(cx, cy, z, psi)
                pap = G.to_w(T, G.paper_b(SQ))
                if box:             # kotak: tapak kotak (sumbu panjang = sumbu panjang lembar) bebas kolom perabot
                    R2 = G.R_board(psi)[:2, :2]
                    hx, hy = yr[1] / 2, yr[0] / 2     # x_b = sisi pendek lembar -> sisi pendek kotak
                    pp = np.array([[cx, cy]]) + np.array([[sx * hx, sy * hy] for sx in (-1, 0, 1) for sy in (-1, 0, 1)]) @ R2.T
                    if (floor_tree.query(pp)[0] < 0.05).any():
                        continue
                elif xr is None:    # lantai: lembar + 5 cm bebas kolom perabot
                    pp = np.r_[pap[:, :2], [[cx, cy]], (pap[:, :2] + [cx, cy]) / 2]
                    if (floor_tree.query(pp)[0] < 0.05).any():
                        continue
                elif not ((pap[:, 0] >= xr[0] - 0.01).all() and (pap[:, 0] <= xr[1] + 0.01).all()
                          and (pap[:, 1] >= yr[0] - 0.01).all() and (pap[:, 1] <= yr[1] + 0.01).all()):
                    continue
                out.append(dict(surf=name, cx=round(float(cx), 3), cy=round(float(cy), 3), z=z, psi=psi, T=T))
    return out


def visibility(cands, cp, env, margin, rails=None):
    trees = {}
    pts = G.robot_points(*rest_with_rails(rails))
    masks = {c: G.robot_mask(c, cp, pts, margin) for c in G.CAMS}
    for b in cands:
        b['vis'] = {}
        pap = G.to_w(b['T'], G.paper_b(SQ))
        ctr = pap.mean(0)
        crn = G.to_w(b['T'], G.corners_b(SQ))
        for c in G.CAMS:
            K, d, T = cp[c]['K'], cp[c]['dist'], cp[c]['T_wc']
            o = T[:3, 3]
            v = dict(dist=float(np.linalg.norm(ctr - o)))
            ray = (ctr - o) / v['dist']
            v['ang'] = float(np.degrees(np.arccos(abs(ray @ b['T'][:3, 2]))))
            uv, z = G.project(T, K, d, pap)
            m_px = margin * K[0, 0] / max(z.min(), 0.1)
            v['in_img'] = bool((z > 0.1).all() and (uv[:, 0] >= m_px).all() and (uv[:, 0] < G.IMG_W - m_px).all()
                               and (uv[:, 1] >= m_px).all() and (uv[:, 1] < G.IMG_H - m_px).all())
            ok = v['in_img'] and v['dist'] <= MAX_DIST and v['ang'] <= MAX_ANG
            if v['in_img']:
                poly = np.zeros((G.IMG_H, G.IMG_W), np.uint8)
                cv2.fillConvexPoly(poly, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
                v['arm_occ'] = bool((poly & masks[c]).any())
                cuv, _ = G.project(T, K, d, crn)
                g = cuv.reshape(4, 3, 2)
                v['sq_px'] = float(min(np.linalg.norm(np.diff(g, axis=0), axis=2).min(),
                                       np.linalg.norm(np.diff(g, axis=1), axis=2).min()))
                ok = ok and not v['arm_occ'] and v['sq_px'] >= MIN_SQ_PX
            if ok:
                if b['z'] not in trees:
                    trees[b['z']] = cKDTree(env[env[:, 2] > b['z'] + margin])
                clr = np.inf
                for p in np.r_[pap, [ctr]]:
                    L = np.linalg.norm(p - o)
                    s = o + np.outer(np.arange(0.05, L, 0.02), (p - o) / L)
                    clr = min(clr, float(trees[b['z']].query(s)[0].min()))
                v['env_clear'] = clr
                ok = clr >= margin
            v['ok'] = bool(ok)
            b['vis'][c] = v
    return cands


def cam_cov(boards, cam, cp, full=False):
    """Kovarians linear pose kamera (rotvec rad, posisi pusat m) dari papan terpilih: piksel + galat pita per papan."""
    K, d, T0 = cp[cam]['K'], cp[cam]['dist'], cp[cam]['T_wc']

    def uv_all(Twc, Ts):
        return np.concatenate([G.project(Twc, K, d, G.to_w(Tb, G.corners_b(SQ)))[0].ravel() for Tb in Ts])

    def pert(Tw, dv):
        T = Tw.copy()
        T[:3, :3] = Rotation.from_rotvec(dv[:3]).as_matrix() @ Tw[:3, :3]
        T[:3, 3] = Tw[:3, 3] + dv[3:]
        return T
    Ts = [b['T'] for b in boards]
    u0 = uv_all(T0, Ts)
    e = 1e-6
    J = np.stack([(uv_all(pert(T0, e * np.eye(6)[k]), Ts) - u0) / e for k in range(6)], 1)
    cols, sig = [], []
    for i, Tb in enumerate(Ts):          # papan: dx, dy, dz, dyaw
        for k, s in ((3, SIG_TAPE), (4, SIG_TAPE), (5, SIG_Z), (2, SIG_YAW)):
            dv = np.zeros(6)
            dv[k] = e
            Tp = list(Ts)
            Tc = Tb.copy()
            Tc[:3, :3] = Rotation.from_rotvec(dv[:3]).as_matrix() @ Tb[:3, :3]
            Tc[:3, 3] = Tb[:3, 3] + dv[3:]
            Tp[i] = Tc
            cols.append((uv_all(T0, Tp) - u0) / e)
            sig.append(s)
    Jb = np.stack(cols, 1)
    S = SIG_PX ** 2 * np.eye(len(u0)) + Jb @ np.diag(np.square(sig)) @ Jb.T
    A = np.linalg.pinv(J)
    C = A @ S @ A.T
    if full:
        return C
    return float(np.degrees(np.sqrt(np.trace(C[:3, :3])))), float(1000 * np.sqrt(np.trace(C[3:, 3:])))


def platform_box(b):
    """Volume platform bergerak (kotak/laci) di bawah papan b: (lo, hi) AABB dunia, atau None (lantai/permukaan tetap)."""
    xr, yr, z, _ = SURF[b['surf']]
    if xr != 'kotak':
        return None
    R2 = G.R_board(b['psi'])[:2, :2]
    hx, hy = yr[1] / 2, yr[0] / 2
    xy = np.array([[b['cx'], b['cy']]]) + np.array([[sx * hx, sy * hy] for sx in (-1, 1) for sy in (-1, 1)]) @ R2.T
    return np.r_[xy.min(0), 0.0], np.r_[xy.max(0), z]


def seg_hits_box(o, p, lo, hi, pad):
    """Segmen o->p memotong AABB yang digembungkan pad (metode slab)."""
    lo, hi = lo - pad, hi + pad
    d = p - o
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if o[k] < lo[k] or o[k] > hi[k]:
                return False
            continue
        a, b = (lo[k] - o[k]) / d[k], (hi[k] - o[k]) / d[k]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def occluded_by(b, plat, cam, cp, margin):
    """Papan b tertutup platform lain (volume kotak) untuk kamera cam."""
    lo, hi = platform_box(plat)
    o = cp[cam]['T_wc'][:3, 3]
    pap = G.to_w(b['T'], G.paper_b(SQ))
    return any(seg_hits_box(o, q, lo, hi, margin) for q in np.r_[pap, [pap.mean(0)]])


def point_err(boards, cam, cp, n=400, seed=1):
    """Galat lateral (mm) titik dunia di ruang kerja (kisi z 0 / 0.77 / 1.3 yang terlihat) akibat galat pose kamera
    (kovarians penuh, Monte Carlo): median dan p95 jarak pergeseran sinar pada kedalaman titik."""
    K, d, T0 = cp[cam]['K'], cp[cam]['dist'], cp[cam]['T_wc']
    C = cam_cov(boards, cam, cp, full=True)
    rng = np.random.default_rng(seed)
    X = np.array([[x, y, z] for x in np.arange(-0.5, 2.6, 0.25) for y in np.arange(-1.2, 1.3, 0.25) for z in (0, 0.77, 1.3)])
    uv, dep = G.project(T0, K, d, X)
    m = (dep > 0.3) & (uv[:, 0] > 0) & (uv[:, 0] < G.IMG_W) & (uv[:, 1] > 0) & (uv[:, 1] < G.IMG_H) & (dep < 3.0)
    X, dep = X[m], dep[m]
    e = []
    for dv in rng.multivariate_normal(np.zeros(6), C, n):
        T = T0.copy()
        T[:3, :3] = Rotation.from_rotvec(dv[:3]).as_matrix() @ T0[:3, :3]
        T[:3, 3] = T0[:3, 3] + dv[3:]
        uv2, _ = G.project(T, K, d, X)
        e.append(np.linalg.norm(uv2 - uv[m], axis=1) * dep / K[0, 0])
    e = 1000 * np.array(e)
    return round(float(np.median(e)), 1), round(float(np.percentile(e, 95)), 1), int(m.sum())


def select(cands, cp, n=6, seed=0, margin=0.10, min_both=2, restarts=150):
    vis = [b for b in cands if any(v['ok'] for v in b['vis'].values())]
    # satu wakil per sel 0.25 m x permukaan x set kamera (paling dekat ke tengah sel) -> ruang cari kecil
    cell = {}
    for b in vis:
        key = (b['surf'], round(b['cx'] / 0.25), round(b['cy'] / 0.25), tuple(b['vis'][c]['ok'] for c in G.CAMS))
        cell.setdefault(key, []).append(b)
    pool = [min(v, key=lambda b: (b['cx'] - round(b['cx'] / 0.25) * 0.25) ** 2 + (b['cy'] - round(b['cy'] / 0.25) * 0.25) ** 2)
            for v in cell.values()]

    occ_cache = {}

    def eff(S):
        """Visibilitas efektif di set S: vis papan dan tidak tertutup platform papan lain dalam S."""
        out = []
        for b in S:
            e = {}
            for c in G.CAMS:
                ok = b['vis'][c]['ok']
                for q in S:
                    if ok and q is not b and platform_box(q) is not None:
                        key = (id(b), id(q), c)
                        if key not in occ_cache:
                            occ_cache[key] = occluded_by(b, q, c, cp, margin)
                        ok = not occ_cache[key]
                e[c] = ok
            out.append(e)
        return out

    def violation(S):
        """0 = layak; > 0 = jumlah pelanggaran berbobot (memandu pencarian lokal)."""
        v = 0.0
        surf = [b['surf'] for b in S]
        v += sum(max(0, surf.count(k) - n) for k, n in STOCK.items())
        v += sum(a in surf and b in surf for a, b in EXCLUSIVE)
        P = np.array([[b['cx'], b['cy'], b['z']] for b in S])
        v += sum(max(0.0, 0.40 - np.linalg.norm(P[i] - P[j])) / 0.1 for i, j in itertools.combinations(range(len(S)), 2))
        E = eff(S)
        for c in G.CAMS:
            B = [b for b, e in zip(S, E) if e[c]]
            Q = np.array([[b['cx'], b['cy'], b['z']] for b in B]) if B else np.zeros((0, 3))
            spread = max((np.linalg.norm(Q[i] - Q[j]) for i, j in itertools.combinations(range(len(B)), 2)), default=0)
            v += max(0, 4 - len(B)) + max(0.0, 1.0 - spread)
        v += max(0, min_both - sum(all(e.values()) for e in E))
        return v

    def score(S):
        vio = violation(S)
        if vio > 0:
            return 1e3 + vio, None
        out = {}
        E = eff(S)
        for c in G.CAMS:
            B = [b for b, e in zip(S, E) if e[c]]
            Q = np.array([[b['cx'], b['cy'], b['z']] for b in B])
            spread = max(np.linalg.norm(Q[i] - Q[j]) for i, j in itertools.combinations(range(len(B)), 2))
            r, t = cam_cov(B, c, cp)
            out[c] = dict(n=len(B), spread=round(float(spread), 2), rot_deg=round(r, 3), pos_mm=round(t, 1))
        n_elev = sum(b['surf'] != 'lantai' for b in S)
        return max(o['rot_deg'] / 0.1 + o['pos_mm'] / 5.0 for o in out.values()) + 0.05 * n_elev, out

    both = [b for b in pool if all(b['vis'][c]['ok'] for c in G.CAMS)]
    rng = np.random.default_rng(seed)
    best = (np.inf, None, None)
    best_eff = None
    for it in range(restarts):                 # mulai acak (2 papan bersama + 4 sisanya), lalu tukar-satu lokal
        if len(both) < 2:
            break
        S = list(rng.choice(both, 2, replace=False)) + list(rng.choice(pool, n - 2, replace=False))
        sc, info = score(S)
        improved = True
        while improved:
            improved = False
            for i in range(n):
                for b in rng.permutation(pool)[:60]:
                    T = S[:i] + [b] + S[i + 1:]
                    s2, inf2 = score(T)
                    if s2 < sc - 1e-9:
                        S, sc, info, improved = T, s2, inf2, True
        if sc < best[0]:
            best = (sc, S, info)
            best_eff = eff(S)
    best_eff = eff(best[1]) if best[1] is not None else None
    global LAST_BEST
    LAST_BEST = (best[1], best_eff if best[1] is not None else None)
    if best[0] >= 1e3:
        return (best[0], None, None), pool, both
    for b, e in zip(best[1], best_eff):      # visibilitas efektif (oklusi platform) menggantikan 'ok' per papan
        for c in G.CAMS:
            b['vis'][c]['occ_platform'] = bool(b['vis'][c]['ok'] and not e[c])
            b['vis'][c]['ok'] = bool(e[c])
    return best, pool, both


def overlay(sel, cp, out_dir, margin):
    import glob
    pts = G.robot_points()
    for c in G.CAMS:
        f = sorted(glob.glob(G.G63 + '/g63b/hw_20261016_finger/frames/B-R0/%s/*.png' % c))[-1]
        im = cv2.imread(f)
        m = G.robot_mask(c, cp, pts, margin)
        im[m > 0] = (0.65 * im[m > 0] + [0, 0, 90]).astype(np.uint8)
        K, d, T = cp[c]['K'], cp[c]['dist'], cp[c]['T_wc']
        for b in sel:
            uv, z = G.project(T, K, d, G.to_w(b['T'], G.paper_b(SQ)))
            if (z <= 0).any():
                continue
            col = (0, 255, 0) if b['vis'][c]['ok'] else (128, 128, 128)
            cv2.polylines(im, [uv.astype(np.int32)], True, col, 2)
            k, _ = G.project(T, K, d, G.k_points(b['T'], SQ))
            for j, p in enumerate(k):
                cv2.circle(im, tuple(p.astype(int)), 4, (0, 255, 255), -1)
            cv2.putText(im, b['name'], tuple(uv.mean(0).astype(int)), 0, 0.8, col, 2)
        for n, (x, y) in G.PLUMB.items():
            p, z = G.project(T, K, d, np.array([[x, y, 0.0]]))
            if z[0] > 0:
                cv2.drawMarker(im, tuple(p[0].astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(im, n, tuple(p[0].astype(int) + 6), 0, 0.6, (255, 0, 255), 2)
        cv2.imwrite(os.path.join(out_dir, 'plan_%s.jpg' % c), im)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--margin', type=float, default=0.10)
    ap.add_argument('--out', default=os.path.join(G.HERE, 'g63d_plan.json'))
    ap.add_argument('--overlay-dir', default=None, help='frame lama berisi orang -> LOKAL, jangan commit')
    a = ap.parse_args()
    cp = G.cam_params()
    env = env_points()
    cands = candidates(env)
    print('permukaan:', {k: v[:3] for k, v in SURF.items()})
    print('kandidat', len(cands))
    visibility(cands, cp, env, a.margin)
    for c in G.CAMS:
        print(c, 'terlihat', sum(b['vis'][c]['ok'] for b in cands), 'per permukaan',
              {s: sum(b['vis'][c]['ok'] for b in cands if b['surf'] == s) for s in SURF})
    (sc, sel, info), pool, both = select(cands, cp, margin=a.margin)
    print('pool', len(pool), 'bersama', len(both), 'skor', sc)
    if sel is None or sc >= 1e3:
        print('GAGAL: tidak ada set 6 papan yang memenuhi syarat')
        return 1
    sel = sorted(sel, key=lambda b: (b['surf'], b['cx']))
    rows = []
    for i, b in enumerate(sel):
        b['name'] = 'S%d' % (i + 1)
        k = G.k_points(b['T'], SQ)
        near = sorted(G.PLUMB, key=lambda n: np.hypot(*(np.array(G.PLUMB[n]) - k[0, :2])))[:3]
        rows.append(dict(name=b['name'], surf=b['surf'], center=[b['cx'], b['cy'], b['z']],
                         yaw_deg=round(float(np.degrees(b['psi'])), 1), K1=np.round(k[0], 3).tolist(),
                         K2=np.round(k[1], 3).tolist(),
                         tape_cm={n: [round(100 * float(np.hypot(*(k[j, :2] - G.PLUMB[n]))), 1) for j in (0, 1)]
                                  for n in near},
                         vis={c: {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in b['vis'][c].items()}
                              for c in G.CAMS}))
        print(b['name'], b['surf'], rows[-1]['center'], 'yaw', rows[-1]['yaw_deg'],
              {c: (b['vis'][c]['ok'], round(b['vis'][c]['dist'], 2), round(b['vis'][c]['ang'], 1),
                   round(b['vis'][c].get('sq_px', 0), 1)) for c in G.CAMS})
    for c in G.CAMS:
        info[c]['pt_err_mm_med_p95_n'] = point_err([b for b in sel if b['vis'][c]['ok']], c, cp)
    print('prediksi galat kamera (rot/pos = sqrt-trace, berkorelasi; pt_err = galat lateral titik ruang kerja):', info)
    json.dump(dict(margin=a.margin, sq=SQ, surf={k: list(v) for k, v in SURF.items()}, boards=rows,
                   pred=info, n_candidates=len(cands)), open(a.out, 'w'), indent=1, default=float)
    if a.overlay_dir:
        os.makedirs(a.overlay_dir, exist_ok=True)
        overlay(sel, cp, a.overlay_dir, a.margin)
    return 0


if __name__ == '__main__':
    sys.exit(main())
