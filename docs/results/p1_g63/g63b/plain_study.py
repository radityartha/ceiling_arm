#!/usr/bin/env python3
"""G63b (LURING): studi sintetis target papan catur POLOS ukuran A4 vs papan ChArUco lama.

1. Kalibrasi model blur: render papan ChArUco LAMA (4x5 @ 48.43 mm) pada jarak/kemiringan P0 nyata (A-01 rgbd 1.04 m 13 deg,
   A-R0 rgbd2 1.44 m) di atas latar frame P0 nyata; sapu blur -> blur yang mereproduksi kegagalan nyata (marker 4/10 @1.04,
   0 @1.44). Frame nyata hanya latar + noise (data pengembangan, bukan data kalibrasi; tidak disimpan).
2. Kandidat papan polos: deteksi findChessboardCornersSB + pelabelan 180 deg dari paritas (jumlah kotak ganjil x genap),
   jarak 0.8-3.0 m, kemiringan 0-55 deg, blur terkalibrasi (+margin). Metrik: laju deteksi, label SALAH (harus 0),
   galat sudut px (median/p95) dan lateral mm = px * d / f.

    python3 g63b/plain_study.py [--trials 12] > g63b/plain_study.log
"""
import argparse
import glob
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from make_g63_boards import DICT_NAME, MARKER_MM, N_MARKERS  # noqa: E402

K = np.array([[643.544, 0, 647.053], [0, 642.759, 365.949], [0, 0, 1.0]])
W, H = 1280, 720
FRAMES = os.path.join(os.path.dirname(HERE), 'hw_20261015b', 'cap', 'frames')
PPM = 3000                                       # texture px per metre (3 px/mm)
MAX_H_RMS = 1.0                                  # residu homografi 12 sudut (px): grid campuran papan lain -> tolak
MIN_PARITY = 0.5                                 # |kontras paritas| ternormalisasi minimum (aturan terdaftar, sebelum HW)

# kandidat: nama -> (kotak_x, kotak_y, kotak_m, tepi_putih_m) ; semua muat A4 (210 x 297) atau sambungan A4
CANDS = {
    'P50_4x5_A4': (4, 5, 0.050, 0.005),         # papan lama tanpa marker, tepi kertas 5 mm
    'P50_4x5_A4_tepi20': (4, 5, 0.050, 0.020),  # sama + tepi putih 20 mm (kardus berlapis kertas putih)
    # P65_3x4 (A4, kotak 65) GUGUR: 2x3 sudut dalam < minimum findChessboardCornersSB (>= 3 per sisi)
    'P70_4x5_2xA4': (4, 5, 0.070, 0.020),       # sambungan 2 lembar A4 (280 x 350), sambungan di garis kotak
    'P42_4x5_A4_tepi21': (4, 5, 0.042, 0.021),  # operator 2026-10-15 (2b): satu A4, tepi = kertas putih tanpa tinta
}


def backgrounds():
    out = []
    for p in ('A-01/rgbd', 'A-R0/rgbd2', 'A-01/rgbd2', 'A-R0/rgbd'):
        f = sorted(glob.glob(os.path.join(FRAMES, p, '*.png')))
        if f:
            out.append(cv2.imread(f[0], 0))
    if not out:
        sys.exit('GAGAL: frame P0 tidak ada di %s' % FRAMES)
    return out


def pose(d, tilt_deg, spin_deg, rng, extent):
    """Board frame -> camera: centre on a ray near image centre at range d, tilted about board x, spun about normal."""
    # konvensi OpenCV: x kanan, y bawah, z MENJAUHI kamera -> muka depan terlihat dengan det(J) > 0
    R = Rotation.from_euler('zxz', [spin_deg, tilt_deg, rng.uniform(-5, 5)], degrees=True).as_matrix()
    ray = np.array([rng.uniform(-0.25, 0.25), rng.uniform(-0.15, 0.15), 1.0])
    c = ray / np.linalg.norm(ray) * d
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = c - R @ np.array([extent[0] / 2, extent[1] / 2, 0])
    return T


def warp(tex, T, extent_m, origin_m, bg, blur, noise, rng):
    h, w = tex.shape
    x0, y0 = origin_m
    obj = np.array([[x0, y0, 0], [x0 + extent_m[0], y0, 0], [x0 + extent_m[0], y0 + extent_m[1], 0],
                    [x0, y0 + extent_m[1], 0]])
    rv, _ = cv2.Rodrigues(T[:3, :3])
    dst, _ = cv2.projectPoints(obj, rv, T[:3, 3], K, np.zeros(5))
    Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst.reshape(-1, 2).astype(np.float32))
    mask = cv2.warpPerspective(np.full_like(tex, 255), Hm, (W, H), flags=cv2.INTER_AREA)
    img = np.where(mask > 127, cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_AREA), bg)
    img = cv2.GaussianBlur(img.astype(np.uint8), (0, 0), blur)
    return np.clip(img + rng.normal(scale=noise, size=img.shape), 0, 255).astype(np.uint8)


# ----------------------------------------------------------------------------------------- 1. kalibrasi blur (papan lama)
def old_board_markers(bgs, trials):
    d = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))
    sq = 0.04843
    board = aruco.CharucoBoard((4, 5), sq, MARKER_MM / 1000 * 0.9686, d, np.arange(N_MARKERS))
    tex = board.generateImage((int(4 * sq * PPM), int(5 * sq * PPM)), marginSize=0, borderBits=1)
    pad = int(0.005 * PPM)
    tex = cv2.copyMakeBorder(tex, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    # cetakan nyata: hitam abu (~60) + putih tak penuh (~200)
    tex = (60 + tex.astype(float) / 255 * 140).astype(np.uint8)
    det = aruco.CharucoDetector(board)
    print('# 1. kalibrasi blur: papan ChArUco LAMA, cetak hitam 60 / putih 200, noise 2.3; nyata: 4/10 @1.04 m, 0/10 @1.44 m')
    print('blur_px  marker@1.04m(13deg)  marker@1.44m(30deg)')
    for blur in (0.8, 1.0, 1.3, 1.6, 2.0, 2.5):
        row = []
        for dist, tilt in ((1.04, 13), (1.44, 30)):
            n = []
            for t in range(trials):
                rng = np.random.default_rng(t)
                T = pose(dist, tilt, rng.uniform(-30, 30), rng, (4 * sq, 5 * sq))
                img = warp(tex, T, (4 * sq + 0.01, 5 * sq + 0.01), (-0.005, -0.005), bgs[t % len(bgs)], blur, 2.3, rng)
                _, _, _, mi = det.detectBoard(img)
                n.append(0 if mi is None else len(mi))
            row.append('%4.1f' % np.median(n))
        print('%5.1f    %s/10              %s/10' % (blur, row[0], row[1]))


# --------------------------------------------------------------------------------------------- 2. papan polos
def plain_tex(nx, ny, sq, border, black=30, white=230):
    s = int(round(sq * PPM))
    t = np.full((ny * s, nx * s), white, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:                   # kotak (0,0) hitam
                t[j * s:(j + 1) * s, i * s:(i + 1) * s] = black
    b = int(round(border * PPM))
    return cv2.copyMakeBorder(t, b, b, b, b, cv2.BORDER_CONSTANT, value=white)


def truth_corners(nx, ny, sq, T):
    g = np.array([[(i + 1) * sq, (j + 1) * sq, 0] for j in range(ny - 1) for i in range(nx - 1)], float)
    rv, _ = cv2.Rodrigues(T[:3, :3])
    uv, _ = cv2.projectPoints(g, rv, T[:3, 3], K, np.zeros(5))
    return uv.reshape(-1, 2)


def label_parity(img, uv, nx, ny, face=+1):
    """Board-order corners + parity score. SB returns a (nx-1)x(ny-1) row-major grid in an arbitrary one of the 4
    dihedral orders that keep its shape (nx != ny). Handedness (muka depan: det J > 0; muka belakang: < 0) removes the
    2 mirror orders; parity of ALL squares (ganjil x genap -> 180 deg changes colours) picks between the other 2.
    Score = (mean I_putih - mean I_hitam) / (max - min) for the chosen order; < MIN_PARITY = ambigu."""
    cx, cy = nx - 1, ny - 1
    grid = uv.reshape(cy, cx, 2)
    g = np.array([[(i + 1), (j + 1)] for j in range(cy) for i in range(cx)], np.float32)
    cen = np.array([[i + 0.5, j + 0.5] for j in range(ny) for i in range(nx)], np.float32)
    black = np.array([(i + j) % 2 == 0 for j in range(ny) for i in range(nx)])
    best = None
    for fx_, fy_ in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        cand = grid[::fy_, ::fx_].reshape(-1, 2).astype(np.float32)
        Hm, _ = cv2.findHomography(g, cand)
        if np.sign(np.linalg.det(Hm[:2, :2] * Hm[2, 2] - np.outer(Hm[:2, 2], Hm[2, :2]))) != face:
            continue
        pc = cv2.perspectiveTransform(cen[None], Hm)[0]
        if not np.all((pc[:, 0] >= 2) & (pc[:, 0] < W - 2) & (pc[:, 1] >= 2) & (pc[:, 1] < H - 2)):
            return None, 0.0
        vals = np.array([img[int(round(y)) - 1:int(round(y)) + 2, int(round(x)) - 1:int(round(x)) + 2].mean()
                         for x, y in pc])
        sc = (vals[~black].mean() - vals[black].mean()) / (vals.max() - vals.min() + 1e-6)
        if best is None or sc > best[1]:
            best = (cand, sc)
    return best if best else (None, 0.0)


def h_rms(uv, nx, ny):
    g = np.array([[(i + 1), (j + 1)] for j in range(ny - 1) for i in range(nx - 1)], np.float32)
    Hm, _ = cv2.findHomography(g, uv.astype(np.float32))
    return float(np.sqrt(np.mean(np.sum((cv2.perspectiveTransform(g[None], Hm)[0] - uv) ** 2, 1))))


def ring_white(img, uv, nx, ny, border_sq):
    """Penjaga sub-kisi: titik di cincin tepi putih (setengah lebar tepi di luar kotak terluar, satu per kotak tepi)
    harus SEMUA lebih terang dari tengah (hitam+putih)/2. Sub-kisi papan catur lain -> separuh cincin hitam -> tolak."""
    cx, cy = nx - 1, ny - 1
    g = np.array([[(i + 1), (j + 1)] for j in range(cy) for i in range(cx)], np.float32)
    Hm, _ = cv2.findHomography(g, uv.astype(np.float32))
    o = border_sq / 2
    ring = [[i + 0.5, -o] for i in range(nx)] + [[i + 0.5, ny + o] for i in range(nx)] + \
           [[-o, j + 0.5] for j in range(ny)] + [[nx + o, j + 0.5] for j in range(ny)]
    cen = np.array([[i + 0.5, j + 0.5] for j in range(ny) for i in range(nx)], np.float32)
    black = np.array([(i + j) % 2 == 0 for j in range(ny) for i in range(nx)])
    pr = cv2.perspectiveTransform(np.array(ring, np.float32)[None], Hm)[0]
    pc = cv2.perspectiveTransform(cen[None], Hm)[0]
    if not np.all((pr[:, 0] >= 1) & (pr[:, 0] < W - 1) & (pr[:, 1] >= 1) & (pr[:, 1] < H - 1)):
        return False
    v = lambda p: np.array([img[int(round(y)), int(round(x))] for x, y in p], float)
    vc = v(pc)
    mid = (vc[~black].mean() + vc[black].mean()) / 2
    return bool(np.all(v(pr) > mid))


def roi_crop(img, T, nx, ny, sq, border, prior_err, rng):
    """Prior FK: pusat papan + galat 3D |e| = prior_err (kasus terburuk), radius = setengah diagonal + tepi + galat + 5 cm."""
    c = T[:3, :3] @ np.array([nx * sq / 2, ny * sq / 2, 0]) + T[:3, 3]
    e = rng.normal(size=3)
    c = c + e / np.linalg.norm(e) * prior_err
    uv = K @ c
    u, v = uv[:2] / uv[2]
    r = (np.hypot(nx * sq, ny * sq) / 2 + border + prior_err + 0.05) * K[0, 0] / c[2]
    x0, x1 = int(max(0, u - r)), int(min(W, u + r))
    y0, y1 = int(max(0, v - r)), int(min(H, v + r))
    out = np.full_like(img, 128)
    out[y0:y1, x0:x1] = img[y0:y1, x0:x1]
    return out


def plain_study(bgs, trials, blurs, guards, prints):
    print('\n# 2. papan polos: SB(EXHAUSTIVE|ACCURACY) + paritas; label SALAH harus 0; |paritas| min %.2f' % MIN_PARITY)
    print('penjaga ROI-prior 0.18 m + cincin putih: %s' % ('AKTIF' if guards else 'MATI'))
    print('kandidat             cetak    blur  d(m) tilt  det  salah  ambigu  med_px  p95_px  med_mm  p95_mm  px/kotak')
    summary = {}
    for (name, (nx, ny, sq, border)), (bk, wh) in [(c, p) for c in CANDS.items() for p in prints]:
        tex = plain_tex(nx, ny, sq, border, bk, wh)
        ext = (nx * sq + 2 * border, ny * sq + 2 * border)
        for blur in blurs:
            for d in (0.8, 1.0, 1.5, 2.0, 2.5, 3.0):
                for tilt in (0, 30, 45, 55):
                    ok = wrong = amb = 0
                    errs = []
                    for t in range(trials):
                        rng = np.random.default_rng(1000 + t)
                        T = pose(d, tilt, rng.uniform(-30, 30), rng, (nx * sq, ny * sq))
                        img = warp(tex, T, ext, (-border, -border), bgs[t % len(bgs)], blur, 2.3, rng)
                        if guards:
                            img = roi_crop(img, T, nx, ny, sq, border, 0.18, rng)
                        uv = None
                        for _ in range(3 if guards else 1):   # grid ditolak -> tutup hull-nya, ulang (distraktor)
                            f, c = cv2.findChessboardCornersSB(img, (nx - 1, ny - 1),
                                                               cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
                            if not f:
                                break
                            uv, s = label_parity(img, c.reshape(-1, 2), nx, ny)
                            if uv is not None and s >= MIN_PARITY and (not guards or (
                                    h_rms(uv, nx, ny) <= MAX_H_RMS and ring_white(img, uv, nx, ny, border / sq))):
                                break
                            hull = cv2.convexHull(c.reshape(-1, 2).astype(np.int32))
                            img = img.copy()
                            cv2.fillConvexPoly(img, cv2.convexHull(((hull - hull.mean(0)) * 1.6 + hull.mean(0))
                                                                   .astype(np.int32)), 128)
                            uv = None
                        if not f:
                            continue
                        if uv is None:
                            amb += 1
                            continue
                        tr = truth_corners(nx, ny, sq, T)
                        e = np.linalg.norm(uv - tr, axis=1)
                        if e.max() > 0.5 * sq * K[0, 0] / d:   # > setengah kotak = label salah
                            wrong += 1
                            continue
                        ok += 1
                        errs += list(e)
                    errs = np.array(errs) if errs else np.array([np.nan])
                    mp, pp = np.median(errs), np.percentile(errs, 95)
                    print('%-20s %3d/%3d %4.1f  %3.1f  %3d  %2d/%-2d  %3d   %3d    %6.2f  %6.2f  %6.2f  %6.2f  %5.1f' % (
                        name, bk, wh, blur, d, tilt, ok, trials, wrong, amb, mp, pp, mp * d / K[0, 0] * 1e3,
                        pp * d / K[0, 0] * 1e3, sq * K[0, 0] / d * np.cos(np.radians(tilt))))
                    summary[(name, bk, wh, blur, d, tilt)] = (ok, wrong, amb)
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trials', type=int, default=12)
    ap.add_argument('--blurs', default='1.3,2.0')
    a = ap.parse_args()
    bgs = backgrounds()
    old_board_markers(bgs, a.trials)
    prints = [(30, 230), (60, 200)]                # cetak baik / cetak buruk (abu, seperti papan P0)
    blurs = [float(b) for b in a.blurs.split(',')]
    for guards in (False, True):
        s = plain_study(bgs, a.trials, blurs, guards, prints)
        wrong = sum(v[1] for v in s.values())
        print('\nTOTAL label SALAH (penjaga %s) = %d -> %s' % ('AKTIF' if guards else 'MATI', wrong,
              'LULUS' if wrong == 0 else 'GAGAL'))
        for name in CANDS:
            for p in prints:
                for b in blurs:
                    for dmax in (1.5, 2.0, 2.5, 3.0):
                        k = [v for kk, v in s.items() if kk[0] == name and kk[1:3] == p and kk[3] == b and kk[4] <= dmax]
                        print('  ringkas %-20s cetak %d/%d blur %.1f d<=%.1f: deteksi %d/%d salah %d ambigu %d' % (
                            name, p[0], p[1], b, dmax, sum(x[0] for x in k), len(k) * a.trials,
                            sum(x[1] for x in k), sum(x[2] for x in k)))


if __name__ == '__main__':
    main()
