#!/usr/bin/env python3
"""G63b detektor papan catur POLOS 4x5 (§A3 draf G63b). Fungsi inti = plain_study.py (yang tervalidasi sintetis, 0 label
salah) -- diimpor, tidak disalin.

Per citra, per muka (A/B) dengan prior 12x2 uv (FK + ekstrinsik yang dipakai + mount):
  0. muka hanya dicari bila prior menghadap kamera: det J homografi prior > 0 (muka belakang -> < 0) dan seluruh prior
     di dalam bingkai;
  1. ROI = kotak pembatas prior + (0.18 m galat prior + 0.05 m) dalam piksel pada jarak prior; luar ROI = abu 128;
  2. findChessboardCornersSB 3x4 EXHAUSTIVE|ACCURACY; grid ditolak -> hull x1.6 ditutup, ulang maks 3x;
  3. penjaga: residu homografi <= 1.0 px RMS DAN <= 1.5 px per sudut; paritas |skor| >= 0.5 (handedness muka depan); cincin putih (tepi 21 mm);
  4. hasil = 12 sudut id = baris*3 + kolom (konvensi board_points G63) atau tidak ada sama sekali.
Aturan buang di atas DIDAFTARKAN sebelum data HW (draf §A3); tidak dilonggarkan sesudah data.

    python3 g63b_detect.py --image x.png --prior prior.json   (prior = {"A": [[u,v]x12], ...}, K nominal rgbd)
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plain_study as PS  # noqa: E402

NX, NY = 4, 5
PRIOR_ERR_M = 0.18
ROI_PAD_M = 0.05
BORDER_M = 0.021
MAX_TRIES = 3
MAX_H_CORNER_PX = 1.5   # e2e seed 4 (2026-10-15): satu sudut 3.4 px lolos RMS 1.0 (12 sudut) -> tambah batas per sudut
GRID = np.array([[(i + 1), (j + 1)] for j in range(NY - 1) for i in range(NX - 1)], np.float32)


def _h_max(uv):
    Hm, _ = cv2.findHomography(GRID, np.asarray(uv, np.float32))
    return float(np.linalg.norm(cv2.perspectiveTransform(GRID[None], Hm)[0] - uv, axis=1).max())


def _front_facing(prior):
    Hm, _ = cv2.findHomography(GRID, prior.astype(np.float32))
    if Hm is None:
        return False
    return np.linalg.det(Hm[:2, :2] * Hm[2, 2] - np.outer(Hm[:2, 2], Hm[2, :2])) > 0


def _roi(gray, prior, square_m):
    px_per_m = np.median(np.linalg.norm(np.diff(prior.reshape(NY - 1, NX - 1, 2), axis=1), axis=2)) / square_m
    pad = (PRIOR_ERR_M + ROI_PAD_M + BORDER_M + square_m) * px_per_m
    h, w = gray.shape
    x0, y0 = (prior.min(0) - pad).astype(int)
    x1, y1 = (prior.max(0) + pad).astype(int)
    out = np.full_like(gray, 128)
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
    out[y0:y1, x0:x1] = gray[y0:y1, x0:x1]
    return out


def detect_face(gray, prior, square_m):
    """-> (ids list, uv 12x2 list, note). Empty ids = rejected (note says why)."""
    prior = np.asarray(prior, float)
    h, w = gray.shape
    if not ((prior[:, 0] >= 0) & (prior[:, 0] < w) & (prior[:, 1] >= 0) & (prior[:, 1] < h)).all():
        return [], [], 'prior di luar bingkai'
    if not _front_facing(prior):
        return [], [], 'muka membelakangi kamera (prior)'
    PS.W, PS.H = w, h
    img = _roi(gray, prior, square_m)
    why = 'SB tidak menemukan grid'
    for _ in range(MAX_TRIES):
        f, c = cv2.findChessboardCornersSB(img, (NX - 1, NY - 1), cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not f:
            break
        uv, s = PS.label_parity(img, c.reshape(-1, 2), NX, NY)
        if uv is None:
            why = 'paritas: kisi keluar bingkai'
        elif s < PS.MIN_PARITY:
            why = f'paritas lemah {s:.2f}'
        elif PS.h_rms(uv, NX, NY) > PS.MAX_H_RMS:
            why = f'residu H {PS.h_rms(uv, NX, NY):.2f} px'
        elif _h_max(uv) > MAX_H_CORNER_PX:
            why = f'residu H sudut maks {_h_max(uv):.2f} px'
        elif not PS.ring_white(img, uv, NX, NY, BORDER_M / square_m):
            why = 'cincin tepi tidak putih (sub-kisi / papan lain)'
        else:
            return list(range(12)), uv.tolist(), f'ok paritas {s:.2f}'
        hull = cv2.convexHull(c.reshape(-1, 2).astype(np.int32))
        img = img.copy()
        cv2.fillConvexPoly(img, cv2.convexHull(((hull - hull.mean(0)) * 1.6 + hull.mean(0)).astype(np.int32)), 128)
    return [], [], why


def detect(gray, priors, square_m):
    """Drop-in for g63_detect.detect (capture): {face: dict(ids, uv, method, n_markers, note)}."""
    out = {}
    for name, prior in (priors or {}).items():
        ids, uv, note = detect_face(gray, prior, square_m[name] if isinstance(square_m, dict) else square_m)
        out[name] = dict(ids=ids, uv=uv, method='polos' if ids else 'none', n_markers=0, note=note)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', required=True)
    ap.add_argument('--prior', required=True)
    ap.add_argument('--square-m', type=float, default=0.042)
    a = ap.parse_args()
    gray = cv2.imread(a.image, 0)
    for k, v in detect(gray, json.load(open(a.prior)), a.square_m).items():
        print(k, len(v['ids']), 'sudut', v['note'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
