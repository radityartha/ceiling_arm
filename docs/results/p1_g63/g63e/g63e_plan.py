#!/usr/bin/env python3
"""G63e tugas 1 (LURING): rencana penanda ArUco BESAR statis (datar + VERTIKAL).

Kandidat:
  datar   = kandidat papan g63d_plan (lantai, atas kotak A/B, atas laci dipindah; footprint lembar A4 sama) -> pusat penanda;
  vertikal = sisi platform bergerak (kotak A 470x350x310, B 423x268x480, laci dipindah 330x320x700 [operator]) di lantai
             bebas (kisi 0.10 m, yaw 0/90, 4 sisi), lembar digunting jadi inti 210 x (TAG+30) mm, tepi atas inti 1 cm di bawah
             tepi atas platform; normal luar = arah sisi.
  Bidang TETAP (dinding, muka meja tinggi, lemari) TIDAK direncanakan di sini: posisinya dari awan G33 10-07 (ruangan sudah
  berubah) -> ditentukan dari awan depth BARU saat uji kecil.
Terlihat per kamera bila SEMUA: inti lembar di bingkai (tepi MARGIN pada kedalaman), jarak <= MAX_DIST, sudut normal-sinar
<= MAX_ANG, sisi penanda terproyeksi >= MIN_TAG_PX (40), inti lembar tidak kena masker lengan REST/gantry (dilatasi MARGIN),
sinar ke 4 sudut + pusat inti bebas voksel env sejauh MARGIN (voksel di dalam platform penanda sendiri / di bawah bidang
penanda datar diabaikan), tidak tertutup platform lain dalam set (volume kotak).
Pilih N penanda: stok platform 1 tiap jenis (maks 2 penanda per platform: atas + satu sisi), jarak pusat >= MIN_SEP,
rgbd >= MIN_RGBD, rgbd2 >= MIN_RGBD2; tujuan = galat kamera terprediksi terburuk (kovarians linear: sudut 0.3 px,
pita per sudut xy SIG_XY / z SIG_Z, Procrustes 4 sudut).

    python3 g63e_plan.py [--n 10] [--margin 0.10] [--out g63e_plan.json] [--overlay-dir DIR]
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
import g63e_common as E  # noqa: E402
import g63d_common as GD  # noqa: E402
import g63d_plan as PD  # noqa: E402

MAX_DIST, MAX_ANG = 4.0, 60.0
SIG_PX, SIG_XY, SIG_Z = 0.3, 0.003, 0.003
MIN_SEP = 0.30
MIN_RGBD, MIN_RGBD2 = 5, 5
MIN_PX = E.MIN_TAG_PX           # --min-px hanya untuk peta kelayakan; syarat terdaftar = 40
PLAT = {   # nama: (panjang x, lebar y pada yaw 0, tinggi)
    'kotak_A': (0.470, 0.350, 0.31),
    'kotak_B': (0.423, 0.268, 0.48),
    'laci_pindah': (0.33, 0.32, 0.70),
}
FLAT_SURF = ('lantai', 'kotak_A', 'kotak_B', 'laci_pindah')
PAPER_OFF = (E.PAPER[1] / 2) - (E.MARGIN_TOP + E.TAG_NOMINAL / 2)    # pusat lembar -> pusat penanda (sepanjang -y_t)


def flat_candidates(env):
    """Dari kandidat g63d (footprint lembar sudah dicek) -> penanda datar. Platform: ukuran tapak dari PLAT."""
    out = []
    for b in PD.candidates(env):
        if b['surf'] not in FLAT_SURF:
            continue
        pap = GD.to_w(b['T'], GD.paper_b(PD.SQ))
        c = pap.mean(0)
        R = E.R_flat(b['psi'])
        ct = c - R @ np.array([0, PAPER_OFF, 0])
        d = dict(kind='datar', surf=b['surf'], c=ct.round(4), T=E.T_of(R, ct), face=None, core=False)
        if b['surf'] in PLAT:
            d['plat'] = (b['surf'], round(b['cx'], 3), round(b['cy'], 3), 0 if abs(b['psi']) < 1e-6 else 90)
        out.append(d)
    return out


def plat_box(plat):
    """(nama, cx, cy, yaw) -> AABB (lo, hi)."""
    n, cx, cy, yaw = plat
    L, W, H = PLAT[n]
    if yaw == 90:
        L, W = W, L
    return np.array([cx - L / 2, cy - W / 2, 0.0]), np.array([cx + L / 2, cy + W / 2, H])


def vert_candidates(env):
    floor_tree = cKDTree(env[env[:, 2] < 1.5][:, :2])
    out = []
    for n, (L0, W0, H) in PLAT.items():
        for yaw in (0, 90):
            L, W = (L0, W0) if yaw == 0 else (W0, L0)
            for cx, cy in itertools.product(np.arange(-1.2, 3.0, 0.10), np.arange(-2.0, 2.0, 0.10)):
                pp = np.array([[cx + sx * L / 2, cy + sy * W / 2] for sx in (-1, 0, 1) for sy in (-1, 0, 1)])
                if (floor_tree.query(pp)[0] < 0.05).any():
                    continue
                for face, (dx, dy, half) in ((0, (L / 2, 0, W / 2)), (90, (0, W / 2, L / 2)),
                                             (180, (-L / 2, 0, W / 2)), (270, (0, -W / 2, L / 2))):
                    if half * 2 < E.PAPER[0] + 0.02:
                        continue
                    R = E.R_vert(face)
                    ztop_core = H - 0.01
                    ct = np.array([cx + dx, cy + dy, ztop_core - E.MARGIN_TOP - E.TAG_NOMINAL / 2])
                    if ct[2] - E.TAG_NOMINAL / 2 - E.MARGIN_TOP < 0.0:
                        continue
                    out.append(dict(kind='vertikal', surf=n + '_sisi', c=ct.round(4), T=E.T_of(R, ct), face=face, core=True,
                                    plat=(n, round(float(cx), 3), round(float(cy), 3), yaw)))
    return out


def seg_box(o, p, lo, hi, pad):
    return PD.seg_hits_box(o, p, lo, hi, pad)


def visibility(cands, cp, env, margin):
    pts = GD.robot_points()
    masks = {c: GD.robot_mask(c, cp, pts, margin) for c in E.CAMS}
    tree_hi = {}
    tree_all = cKDTree(env[env[:, 2] > 0.05])
    for b in cands:
        b['tv'] = {}
        core = E.to_w(b['T'], E.paper_t(core=True))
        ctr = core.mean(0)
        for c in E.CAMS:
            K, d, T = cp[c]['K'], cp[c]['dist'], cp[c]['T_wc']
            o = T[:3, 3]
            v = dict(dist=float(np.linalg.norm(b['c'] - o)))
            # z_t masuk permukaan: menghadap kamera bila sinar o->c searah z_t (cos > 0) -> sudut = arccos(ray . z_t)
            v['ang'] = float(np.degrees(np.arccos(np.clip((b['c'] - o) / v['dist'] @ b['T'][:3, 2], -1, 1))))
            uv, z = E.project(T, K, d, core)
            m_px = margin * K[0, 0] / max(z.min(), 0.1)
            ok = bool((z > 0.1).all() and (uv[:, 0] >= m_px).all() and (uv[:, 0] < E.IMG_W - m_px).all()
                      and (uv[:, 1] >= m_px).all() and (uv[:, 1] < E.IMG_H - m_px).all())
            v['in_img'] = ok
            ok = ok and v['dist'] <= MAX_DIST and v['ang'] <= MAX_ANG
            if ok:
                tu, _ = E.project(T, K, d, E.to_w(b['T'], E.tag_corners_t()))
                v['side_px'] = float(np.linalg.norm(tu - np.roll(tu, 1, 0), axis=1).min())
                poly = np.zeros((E.IMG_H, E.IMG_W), np.uint8)
                cv2.fillConvexPoly(poly, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
                v['arm_occ'] = bool((poly & masks[c]).any())
                ok = v['side_px'] >= MIN_PX and not v['arm_occ']
            if ok:
                if b['kind'] == 'datar':
                    zz = round(float(b['c'][2]), 3)
                    if zz not in tree_hi:
                        tree_hi[zz] = cKDTree(env[env[:, 2] > zz + margin])
                    tr = tree_hi[zz]
                else:                        # tapak platform bebas voksel (cek lantai) -> tidak ada voksel di dalamnya
                    tr = tree_all
                clr = np.inf
                for p in np.r_[core, [ctr]]:
                    L = np.linalg.norm(p - o)
                    s = o + np.outer(np.arange(0.05, L - margin, 0.02), (p - o) / L)
                    if len(s):
                        clr = min(clr, float(tr.query(s)[0].min()))
                v['env_clear'] = clr
                ok = clr >= margin
            v['ok'] = bool(ok)
            b['tv'][c] = v
    return cands


def cam_cov(tags, cam, cp):
    """Kovarians linear pose kamera (rotvec, posisi) dari penanda: sudut SIG_PX + pita per sudut (Procrustes)."""
    K, d, T0 = cp[cam]['K'], cp[cam]['dist'], cp[cam]['T_wc']
    Pw = [E.to_w(b['T'], E.tag_corners_t()) for b in tags]

    def uv_all(Twc, Ts):
        return np.concatenate([E.project(Twc, K, d, E.to_w(Tt, E.tag_corners_t()))[0].ravel() for Tt in Ts])

    def pert(Tw, dv):
        T = Tw.copy()
        T[:3, :3] = Rotation.from_rotvec(dv[:3]).as_matrix() @ Tw[:3, :3]
        T[:3, 3] = Tw[:3, 3] + dv[3:]
        return T
    Ts = [b['T'] for b in tags]
    u0 = uv_all(T0, Ts)
    e = 1e-6
    J = np.stack([(uv_all(pert(T0, e * np.eye(6)[k]), Ts) - u0) / e for k in range(6)], 1)
    cols, sig = [], []
    for i in range(len(Ts)):
        for k in range(4):
            for ax, s in ((0, SIG_XY), (1, SIG_XY), (2, SIG_Z)):
                P = Pw[i].copy()
                P[k, ax] += e
                Tp = list(Ts)
                Tp[i] = E.procrustes_pose(P)[0]
                cols.append((uv_all(T0, Tp) - u0) / e)
                sig.append(s)
    Jb = np.stack(cols, 1)
    S = SIG_PX ** 2 * np.eye(len(u0)) + Jb @ np.diag(np.square(sig)) @ Jb.T
    A = np.linalg.pinv(J)
    C = A @ S @ A.T
    return C


def cam_err(tags, cam, cp):
    C = cam_cov(tags, cam, cp)
    return float(np.degrees(np.sqrt(np.trace(C[:3, :3])))), float(1000 * np.sqrt(np.trace(C[3:, 3:])))


def eff_vis(S, cp, margin, cache):
    out = []
    for b in S:
        e = {}
        for c in E.CAMS:
            ok = b['tv'][c]['ok']
            if ok:
                o = cp[c]['T_wc'][:3, 3]
                core = E.to_w(b['T'], E.paper_t(core=True))
                for q in S:
                    if q is b or 'plat' not in q or ('plat' in b and q['plat'] == b['plat']):
                        continue
                    key = (id(b), q['plat'], c)
                    if key not in cache:
                        lo, hi = plat_box(q['plat'])
                        cache[key] = any(seg_box(o, p, lo, hi, margin) for p in np.r_[core, [core.mean(0)]])
                    if cache[key]:
                        ok = False
                        break
            e[c] = ok
        out.append(e)
    return out


def violation(S, cp, margin, cache):
    v = 0.0
    plats = {}
    for b in S:
        if 'plat' in b:
            plats.setdefault(b['plat'][0], set()).add(b['plat'])
    v += sum(len(s) - 1 for s in plats.values())                       # stok 1 per jenis (satu posisi)
    v += sum(max(0, sum(1 for b in S if b.get('plat') == p) - 2) for s in plats.values() for p in s)
    for a, b in itertools.combinations(S, 2):                            # platform saling tumpang
        if 'plat' in a and 'plat' in b and a['plat'] != b['plat']:
            la, ha = plat_box(a['plat'])
            lb, hb = plat_box(b['plat'])
            if (la[:2] < hb[:2] + 0.05).all() and (lb[:2] < ha[:2] + 0.05).all():
                v += 1
        if a.get('plat') == b.get('plat') and 'plat' in a and a['kind'] == b['kind']:
            v += 1                                                       # dua penanda sejenis di platform sama
        # penanda lantai tidak boleh di bawah platform
        for p, q in ((a, b), (b, a)):
            if p['surf'] == 'lantai' and 'plat' in q:
                lo, hi = plat_box(q['plat'])
                core = E.to_w(p['T'], E.paper_t())
                if ((core[:, :2] > lo[:2] - 0.03) & (core[:, :2] < hi[:2] + 0.03)).all(1).any() or \
                        ((p['c'][:2] > lo[:2] - 0.15) & (p['c'][:2] < hi[:2] + 0.15)).all():
                    v += 1
        v += max(0.0, MIN_SEP - np.linalg.norm(a['c'] - b['c'])) / 0.1
    E_ = eff_vis(S, cp, margin, cache)
    for c, mn in (('rgbd', MIN_RGBD), ('rgbd2', MIN_RGBD2)):
        v += max(0, mn - sum(e[c] for e in E_))
    return v, E_


def select(cands, cp, n, margin, seed=0, restarts=60):
    vis = [b for b in cands if any(v['ok'] for v in b['tv'].values())]
    cell = {}
    for b in vis:
        key = (b['surf'], b['face'], round(b['c'][0] / 0.2), round(b['c'][1] / 0.2), tuple(b['tv'][c]['ok'] for c in E.CAMS))
        cell.setdefault(key, []).append(b)
    pool = [v[len(v) // 2] for v in cell.values()]
    cache = {}
    rng = np.random.default_rng(seed)

    def score(S):
        vio, E_ = violation(S, cp, margin, cache)
        if vio > 0:
            return 1e3 + vio, None
        info = {}
        for c in E.CAMS:
            B = [b for b, e in zip(S, E_) if e[c]]
            r, t = cam_err(B, c, cp)
            info[c] = dict(n=len(B), rot_deg=round(r, 3), pos_mm=round(t, 1))
        return max(o['rot_deg'] / 0.1 + o['pos_mm'] / 5.0 for o in info.values()), info

    best = (np.inf, None, None)
    for it in range(restarts):
        S = list(rng.choice(pool, n, replace=False))
        sc, info = score(S)
        improved = True
        while improved:
            improved = False
            for i in range(n):
                for b in rng.permutation(pool)[:50]:
                    if any(b is x for x in S):
                        continue
                    T = S[:i] + [b] + S[i + 1:]
                    s2, inf2 = score(T)
                    if s2 < sc - 1e-9:
                        S, sc, info, improved = T, s2, inf2, True
        if sc < best[0]:
            best = (sc, S, info)
        print('  restart %d skor %.2f terbaik %.2f' % (it, sc, best[0]), flush=True)
    if best[1] is not None:
        E_ = eff_vis(best[1], cp, margin, cache)
        for b, e in zip(best[1], E_):
            b['eff'] = e
    return best, pool


def overlay(sel, cp, out_dir, margin):
    import glob
    pts = GD.robot_points()
    for c in E.CAMS:
        f = sorted(glob.glob(E.G63 + '/g63b/hw_20261016_finger/frames/B-R0/%s/*.png' % c))[-1]
        im = cv2.imread(f)
        m = GD.robot_mask(c, cp, pts, margin)
        im[m > 0] = (0.65 * im[m > 0] + [0, 0, 90]).astype(np.uint8)
        K, d, T = cp[c]['K'], cp[c]['dist'], cp[c]['T_wc']
        drawn = set()
        for b in sel:
            if 'plat' in b and b['plat'] not in drawn:
                drawn.add(b['plat'])
                lo, hi = plat_box(b['plat'])
                bx = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                uv, z = E.project(T, K, d, bx)
                if (z > 0).all():
                    for i, j in ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)):
                        cv2.line(im, tuple(uv[i].astype(int)), tuple(uv[j].astype(int)), (255, 160, 0), 1)
            uv, z = E.project(T, K, d, E.to_w(b['T'], E.tag_corners_t()))
            if (z <= 0).any():
                continue
            col = (0, 255, 0) if b['eff'][c] else (128, 128, 128)
            cv2.polylines(im, [uv.astype(np.int32)], True, col, 2)
            cv2.circle(im, tuple(uv[0].astype(int)), 4, (0, 255, 255), -1)
            cv2.putText(im, '%d' % b['id'], tuple(uv.mean(0).astype(int)), 0, 0.7, col, 2)
        for n, (x, y) in GD.PLUMB.items():
            p, z = E.project(T, K, d, np.array([[x, y, 0.0]]))
            if z[0] > 0:
                cv2.drawMarker(im, tuple(p[0].astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(im, n, tuple(p[0].astype(int) + 6), 0, 0.6, (255, 0, 255), 2)
        cv2.imwrite(os.path.join(out_dir, 'plan_%s.jpg' % c), im)


def main():
    global MIN_PX, MIN_RGBD, MIN_RGBD2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, default=10)
    ap.add_argument('--margin', type=float, default=0.10)
    ap.add_argument('--restarts', type=int, default=60)
    ap.add_argument('--min-px', type=float, default=E.MIN_TAG_PX)
    ap.add_argument('--vis-cache', default=None, help='pickle visibilitas (kunci margin, min-px)')
    ap.add_argument('--min-rgbd', type=int, default=MIN_RGBD)
    ap.add_argument('--min-rgbd2', type=int, default=MIN_RGBD2)
    ap.add_argument('--out', default=os.path.join(E.HERE, 'g63e_plan.json'))
    ap.add_argument('--overlay-dir', default=None, help='frame lama berisi orang -> LOKAL, jangan commit')
    a = ap.parse_args()
    MIN_PX, MIN_RGBD, MIN_RGBD2 = a.min_px, a.min_rgbd, a.min_rgbd2
    cp = GD.cam_params()
    env = PD.env_points()
    import pickle
    key = (a.margin, MIN_PX)
    if a.vis_cache and os.path.exists(a.vis_cache) and pickle.load(open(a.vis_cache, 'rb'))[0] == key:
        cands = pickle.load(open(a.vis_cache, 'rb'))[1]
    else:
        cands = flat_candidates(env) + vert_candidates(env)
        visibility(cands, cp, env, a.margin)
        if a.vis_cache:
            pickle.dump((key, cands), open(a.vis_cache, 'wb'))
    print('kandidat', len(cands), {k: sum(b['kind'] == k for b in cands) for k in ('datar', 'vertikal')}, flush=True)
    for c in E.CAMS:
        print(c, 'terlihat', sum(b['tv'][c]['ok'] for b in cands),
              {s: sum(b['tv'][c]['ok'] for b in cands if b['surf'] == s) for s in sorted({b['surf'] for b in cands})}, flush=True)
    print('dua kamera', sum(all(v['ok'] for v in b['tv'].values()) for b in cands), flush=True)
    (sc, sel, info), pool = select(cands, cp, a.n, a.margin, restarts=a.restarts)
    print('pool', len(pool), 'skor', sc, info, flush=True)
    if sel is None or sc >= 1e3:
        print('GAGAL: tidak ada set %d penanda yang memenuhi syarat (rgbd >= %d, rgbd2 >= %d)' % (a.n, MIN_RGBD, MIN_RGBD2))
        return 1
    sel = sorted(sel, key=lambda b: (b['surf'], b['c'][0]))
    rows = []
    for i, b in zip(E.IDS, sel):
        b['id'] = i
        Pw = E.to_w(b['T'], E.tag_corners_t())
        rows.append(dict(id=i, kind=b['kind'], surf=b['surf'], center=b['c'].tolist(), face_deg=b['face'], plat=b.get('plat'),
                         T=b['T'].round(5).tolist(), corners=Pw.round(4).tolist(),
                         vis={c: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in b['tv'][c].items()} for c in E.CAMS},
                         eff=b['eff']))
        print(i, b['kind'], b['surf'], b['c'], 'face', b['face'], b.get('plat'),
              {c: (b['eff'][c], round(b['tv'][c]['dist'], 2), round(b['tv'][c]['ang'], 1), round(b['tv'][c].get('side_px', 0), 1))
               for c in E.CAMS}, flush=True)
    json.dump(dict(n=a.n, margin=a.margin, tag=E.TAG_NOMINAL, tags=rows, pred=info, n_candidates=len(cands),
                   limits=dict(MAX_DIST=MAX_DIST, MAX_ANG=MAX_ANG, MIN_PX=MIN_PX, MIN_SEP=MIN_SEP, MIN_RGBD=MIN_RGBD, MIN_RGBD2=MIN_RGBD2)),
              open(a.out, 'w'), indent=1, default=float)
    if a.overlay_dir:
        os.makedirs(a.overlay_dir, exist_ok=True)
        overlay(sel, cp, a.overlay_dir, a.margin)
    return 0


if __name__ == '__main__':
    sys.exit(main())
