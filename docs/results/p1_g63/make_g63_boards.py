#!/usr/bin/env python3
"""G63: papan ChArUco 4x5 @ 50 mm untuk ditempel di jari gripper (arm_2 = papan A, arm_3 = papan B).

Terpisah dari charuco_common.py (papan lama 5x7 @ 35 mm, DICT_5X5_250) supaya alat kalibrasi lama tidak berubah.
DICT_4X4_250: dictionary BEDA dari papan lama -> tidak pernah tertukar; sel 4x4 lebih besar per bit -> terbaca lebih jauh.
Papan A memakai ID marker 0..9, papan B 10..19 -> kedua papan boleh terlihat bersamaan.

Raster 20 px/mm (508 DPI) -> satu kotak tepat 1000 px, tanpa resampling. PDF A4 menempatkan pola 200x250 mm
pada ukuran fisik persis. CETAK 100 % / ACTUAL SIZE, lalu UKUR (penggaris/jangka sorong) 4 kotak = 200.0 mm dan
5 kotak = 250.0 mm; angka terukur dicatat dan dipakai solver (bukan nominal).

    python3 make_g63_boards.py --out-dir docs/results/p1_g63/boards
"""
import argparse
import hashlib
import os

import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

SQUARES_X = 4          # kolom (lebar 200 mm)
SQUARES_Y = 5          # baris (tinggi 250 mm)
SQUARE_MM = 50.0
MARKER_MM = 37.5       # 0.75 x kotak
DICT_NAME = 'DICT_4X4_250'
PX_PER_MM = 20
N_MARKERS = (SQUARES_X * SQUARES_Y) // 2   # 10
BOARDS = {'A': 0, 'B': N_MARKERS}          # nama -> ID marker pertama
ARM_OF = {'A': 'arm_2', 'B': 'arm_3'}


def build_board(name):
    d = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))
    ids = np.arange(BOARDS[name], BOARDS[name] + N_MARKERS, dtype=np.int32)
    return aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_MM / 1000, MARKER_MM / 1000, d, ids)


def render(name, out_dir):
    board = build_board(name)
    w, h = int(SQUARES_X * SQUARE_MM * PX_PER_MM), int(SQUARES_Y * SQUARE_MM * PX_PER_MM)
    img = board.generateImage((w, h), marginSize=0, borderBits=1)
    png = os.path.join(out_dir, f'g63_board_{name}.png')
    Image.fromarray(img).save(png, dpi=(PX_PER_MM * 25.4, PX_PER_MM * 25.4))

    pdf = os.path.join(out_dir, f'g63_board_{name}.pdf')
    pw, ph = A4
    x0 = (pw - SQUARES_X * SQUARE_MM * mm) / 2          # 5 mm kiri/kanan
    y0 = ph - 8 * mm - SQUARES_Y * SQUARE_MM * mm       # 8 mm dari atas
    c = canvas.Canvas(pdf, pagesize=A4)
    c.setTitle(f'G63 ChArUco board {name}')
    c.drawImage(png, x0, y0, width=SQUARES_X * SQUARE_MM * mm, height=SQUARES_Y * SQUARE_MM * mm)
    # label + penggaris cek skala di bawah pola (vektor, tidak ikut raster)
    ty = y0 - 7 * mm
    c.setFont('Helvetica-Bold', 11)
    c.drawString(12 * mm, ty, f'G63 PAPAN {name}  ({ARM_OF[name]})   ChArUco {SQUARES_X}x{SQUARES_Y}  '
                 f'kotak {SQUARE_MM:.0f} mm  marker {MARKER_MM} mm  {DICT_NAME}  ID {BOARDS[name]}-{BOARDS[name] + N_MARKERS - 1}')
    c.setFont('Helvetica', 9)
    c.drawString(12 * mm, ty - 5 * mm, 'CETAK 100% / ACTUAL SIZE (jangan "fit to page"). Ukur: 4 kotak = 200.0 mm, '
                 '5 kotak = 250.0 mm; catat angka terukur.')
    c.drawString(12 * mm, ty - 10 * mm, 'Tempel rata ke alas kaku; tanpa gelembung/lipatan. Muka papan menghadap kamera.')
    # penggaris 100 mm dengan tik tiap 10 mm
    ry = ty - 18 * mm
    rx = 12 * mm
    c.setLineWidth(0.6)
    c.line(rx, ry, rx + 100 * mm, ry)
    for k in range(11):
        tl = 4 * mm if k % 5 == 0 else 2 * mm
        c.line(rx + k * 10 * mm, ry, rx + k * 10 * mm, ry + tl)
    c.drawString(rx + 102 * mm, ry, 'penggaris 100 mm (cek skala printer)')
    c.save()
    return board, png, pdf


def verify(board, png):
    """Deteksi ulang raster: semua marker + semua sudut dalam terbaca, ID sesuai papan."""
    img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    pad = cv2.copyMakeBorder(small, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
    corners, ids, mcorners, mids = aruco.CharucoDetector(board).detectBoard(pad)
    n_c = 0 if ids is None else len(ids)
    got = sorted(int(i) for i in mids.ravel()) if mids is not None else []
    return n_c, got


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out-dir', required=True)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    ok = True
    for name in BOARDS:
        board, png, pdf = render(name, a.out_dir)
        n_c, got = verify(board, png)
        exp_c = (SQUARES_X - 1) * (SQUARES_Y - 1)
        exp_m = list(range(BOARDS[name], BOARDS[name] + N_MARKERS))
        good = n_c == exp_c and got == exp_m
        ok &= good
        sha = hashlib.sha256(open(pdf, 'rb').read()).hexdigest()
        print(f'{name}: {pdf} sha256 {sha}')
        print(f'   verifikasi raster: sudut {n_c}/{exp_c}, marker {got} -> {"OK" if good else "GAGAL"}')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
