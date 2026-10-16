#!/usr/bin/env python3
"""G63e detektor (berkas baru; detektor terkunci g63b TIDAK diubah, hanya dipanggil).

PENANDA ArUco (DICT_4X4_50, sudut SUBPIX), per frame, ditolak bila:
  T1 ID < 20 (papan lama di ruangan) atau ID tidak ada di rencana            -> 'ID asing'
  T2 ID sama muncul > 1 kali dalam satu frame                                -> 'ID ganda' (keduanya dibuang)
  T3 sisi terproyeksi terkecil < MIN_TAG_PX (40 px)                          -> 'kecil'
  T4 pusat > PRIOR_R_M (0.25 m pada kedalaman prior) + 20 px dari prior (pita + incumbent) -> 'jauh dari prior'
  T5 urutan sudut tidak cocok prior (C0 terdeteksi bukan yang terdekat C0 prior setelah pusat disamakan) -> 'orientasi'
Per kamera multi-frame: median tiap sudut atas frame lolos; perlu >= MIN_FRAC frame; std sudut maks dicatat (> 0.5 px = 'goyah').

PAPAN POLOS GENGGAM 4x5 (tanpa prior): findChessboardCornersSB 3x4 pada citra penuh -> label paritas/handedness
(plain_study.label_parity) -> label itu dipakai sebagai prior untuk g63b_detect.detect_face (ROI + penjaga terkunci:
residu H, paritas, cincin putih). Hasil = 12 sudut id baris*3+kolom atau tidak ada.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402

sys.path.insert(0, E.G63 + '/g63b')
import g63b_detect as GB  # noqa: E402  (terkunci)
import plain_study as PS  # noqa: E402  (terkunci)

PRIOR_R_M = 0.25
PRIOR_R_PX = 20.0
MIN_FRAC = 0.5
SHAKY_PX = 0.5


def _params():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMaxIterations = 50
    p.cornerRefinementMinAccuracy = 0.01
    return p


_DET = None


def detector():
    global _DET
    if _DET is None:
        _DET = cv2.aruco.ArucoDetector(E.dictionary(), _params())
    return _DET


def _refine_once(gray, uv, frac=0.15, step=1.0):
    """Sudut = potongan 4 garis tepi kotak hitam. Titik tepi: sepanjang tiap sisi (buang frac dekat sudut), profil
    intensitas di garis normal +-half px (bilinear), lintasan 50 % antara rata-rata sisi gelap (dalam) dan terang (luar).
    Tak bias oleh blur simetris (beda dengan cornerSubPix yang untuk titik pelana). -> uv 4x2 | None."""
    g = gray.astype(np.float32)
    ctr = uv.mean(0)
    lines = []
    for k in range(4):
        a, b = uv[k], uv[(k + 1) % 4]
        L = np.linalg.norm(b - a)
        t = (b - a) / L
        n = np.array([-t[1], t[0]])
        if n @ (a - ctr) < 0:
            n = -n                                       # n keluar (ke putih)
        half = float(np.clip(0.06 * L, 1.5, 4.0))         # tepi putih kertas = 15/180 sisi -> jangan sampai lantai
        pts = []
        for s in np.arange(frac * L, (1 - frac) * L, step):
            p = a + s * t
            off = np.arange(-int(half / 0.5), int(half / 0.5) + 1) * 0.5     # SIMETRIS (asimetri -> bias 0.1-0.2 px)
            q = (p[None] + off[:, None] * n[None]).astype(np.float32)
            prof = cv2.remap(g, q[:, 0].reshape(1, -1), q[:, 1].reshape(1, -1), cv2.INTER_LINEAR).ravel()
            lo, hi = prof[:3].mean(), prof[-3:].mean()
            if hi - lo < 20:
                continue
            m = (lo + hi) / 2
            j = np.where((prof[:-1] < m) & (prof[1:] >= m))[0]
            if len(j) != 1:
                continue
            j = j[0]
            x = off[j] + (m - prof[j]) / (prof[j + 1] - prof[j]) * 0.5
            pts.append(p + x * n)
        if len(pts) < 5:
            return None
        P = np.array(pts)
        c0 = P.mean(0)
        _, _, Vt = np.linalg.svd(P - c0)
        d = Vt[0]
        lines.append((c0, d))
    out = np.zeros((4, 2))
    for k in range(4):                                   # sudut k = potongan sisi (k-1) dan sisi k
        (p1, d1), (p2, d2) = lines[(k - 1) % 4], lines[k]
        A = np.c_[d1, -d2]
        if abs(np.linalg.det(A)) < 1e-6:
            return None
        s = np.linalg.solve(A, p2 - p1)
        out[k] = p1 + s[0] * d1
    return out


def refine_edges(gray, uv, iters=10, tol=0.01):
    """_refine_once diulang: jendela profil dipusatkan ulang di tepi hasil sebelumnya (jendela tak berpusat -> lo/hi
    asimetris -> bias; uji 1D blur 1.5: 0.46 px sekali jalan) sampai perubahan < tol. Geser > 3 px dari ArUco -> None."""
    uv0 = np.asarray(uv, float)
    cur = uv0
    for _ in range(iters):
        nxt = _refine_once(gray, cur)
        if nxt is None or np.abs(nxt - uv0).max() > 3.0:
            return None
        done = np.abs(nxt - cur).max() < tol
        cur = nxt
        if done:
            break
    return cur


def detect_tags_frame(gray, priors, min_px=E.MIN_TAG_PX, refine=True):
    """priors {id: dict(uv 4x2, depth m, f px)} -> ({id: uv 4x2}, {alasan: n})."""
    corners, ids, _ = detector().detectMarkers(gray)
    out, why = {}, {}
    if ids is None:
        return out, why
    ids = ids.ravel().tolist()
    for k, i in enumerate(ids):
        uv = corners[k].reshape(4, 2).astype(float)
        ref = None
        if i < E.ID_MIN_OLD or (i not in priors if priors is not None else i not in E.IDS):
            r = 'ID asing'
        elif ids.count(i) > 1:
            r = 'ID ganda'
        elif np.linalg.norm(uv - np.roll(uv, 1, 0), axis=1).min() < min_px:
            r = 'kecil'
        elif refine and (ref := refine_edges(gray, uv)) is None:
            r = 'tepi gagal'
        elif priors is None:                      # uji kecil tanpa pita: tanpa cek prior (T4/T5), dilaporkan apa adanya
            out[i] = ref if refine else uv
            continue
        else:
            p = priors[i]
            rad = PRIOR_R_M * p['f'] / p['depth'] + PRIOR_R_PX
            dc = uv.mean(0) - p['uv'].mean(0)
            if np.linalg.norm(dc) > rad:
                r = 'jauh dari prior'
            elif np.argmin(np.linalg.norm(uv - dc - p['uv'][0], axis=1)) != 0:
                r = 'orientasi'
            else:
                out[i] = ref if refine else uv
                continue
        why[r] = why.get(r, 0) + 1
    return out, why


def tag_priors(tags, T_wc, K, dist):
    """tags {id: T_wt} -> priors untuk penanda di depan kamera."""
    pr = {}
    for i, T in tags.items():
        uv, z = E.project(T_wc, K, dist, E.to_w(T, E.tag_corners_t()))
        if (z > 0.1).all():
            pr[i] = dict(uv=uv, depth=float(z.mean()), f=float(K[0, 0]))
    return pr


def detect_tags(frames, tags, T_wc, K, dist):
    """frames: list citra abu / path -> {id: dict(uv median 4x2 | None, n_ok, n, std_px, notes)}."""
    pr = tag_priors(tags, T_wc, K, dist)
    acc = {i: [] for i in pr}
    notes = {}
    for f in frames:
        g = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY) if isinstance(f, str) else f
        got, why = detect_tags_frame(g, pr)
        for r, n in why.items():
            notes[r] = notes.get(r, 0) + n
        for i, uv in got.items():
            acc[i].append(uv)
    out = {}
    for i, L in acc.items():
        rec = dict(n=len(frames), n_ok=len(L))
        if L and len(L) >= MIN_FRAC * len(frames):
            U = np.array(L)
            rec.update(uv=np.median(U, 0), std_px=float(U.std(0).max()))
            rec['shaky'] = rec['std_px'] > SHAKY_PX
        out[i] = rec
    return out, notes


def detect_board_free(gray, sq):
    """Papan polos genggam tanpa prior -> (uv 12x2 | None, catatan)."""
    h, w = gray.shape
    PS.W, PS.H = w, h
    f, c = cv2.findChessboardCornersSB(gray, (GB.NX - 1, GB.NY - 1), cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not f:
        return None, 'SB tidak menemukan grid'
    uv, s = PS.label_parity(gray, c.reshape(-1, 2), GB.NX, GB.NY)
    if uv is None or s < PS.MIN_PARITY:
        return None, 'paritas (tanpa prior) %s' % (None if uv is None else round(float(s), 2))
    ids, uv2, note = GB.detect_face(gray, np.asarray(uv), sq)
    if not ids:
        return None, 'terkunci: ' + note
    return np.asarray(uv2, float), note
