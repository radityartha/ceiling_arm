#!/usr/bin/env python3
"""G63b: papan catur POLOS 4x5 @ 42 mm di tengah satu lembar A4 (operator 2026-10-15: cetak maks A4, opsi 2b).

Pola 168 x 210 mm di tengah A4 210 x 297 -> tepi kertas putih TANPA TINTA 21 mm kiri/kanan, 43.5 mm atas/bawah (penjaga
cincin putih detektor, plain_study.py). Kotak (0,0) kiri-atas HITAM; 4 x 5 = genap x ganjil -> rotasi 180 deg mengubah
warna sudut -> pelabelan dari paritas. Label kecil hanya di tepi bawah lembar (>= 30 mm dari pola, di luar cincin).
Dua lembar identik (A, B); identitas di HW = sisi gripper (normal papan dari FK), bukan pola.

CETAK 100 % / ACTUAL SIZE, matte; lalu UKUR 4 kotak dan 5 kotak (mm) -> angka terukur dipakai solver, bukan nominal.

    python3 make_g63b_boards.py --out-dir boards --check
"""
import argparse
import hashlib
import os

import cv2
import numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

NX, NY, SQ_MM = 4, 5, 42.0
PAGE_W, PAGE_H = 210.0, 297.0
X0 = (PAGE_W - NX * SQ_MM) / 2          # 21.0 mm
Y0 = (PAGE_H - NY * SQ_MM) / 2          # 43.5 mm (dari atas)


def make_pdf(path, name):
    c = canvas.Canvas(path, pagesize=A4)
    c.setFillColorRGB(0, 0, 0)
    for j in range(NY):
        for i in range(NX):
            if (i + j) % 2 == 0:
                # reportlab: origin kiri-bawah
                c.rect((X0 + i * SQ_MM) * mm, (PAGE_H - Y0 - (j + 1) * SQ_MM) * mm, SQ_MM * mm, SQ_MM * mm,
                       stroke=0, fill=1)
    c.setFont('Helvetica', 7)
    c.setFillColorRGB(0.6, 0.6, 0.6)
    c.drawString(X0 * mm, 6 * mm, 'G63b papan %s  4x5 @ %.0f mm  CETAK 100%%  ukur 4 kotak=%.0f mm, 5 kotak=%.0f mm'
                 % (name, SQ_MM, NX * SQ_MM, NY * SQ_MM))
    c.showPage()
    c.save()


def check(pdf):
    """Rasterisasi ulang PDF (pdftoppm 300 dpi) -> SB 3x4 sudut + ukuran kotak."""
    out = pdf[:-4] + '_check'
    os.system('pdftoppm -r 300 -gray -png -singlefile "%s" "%s"' % (pdf, out))
    img = cv2.imread(out + '.png', 0)
    os.remove(out + '.png')
    f, c = cv2.findChessboardCornersSB(img, (NX - 1, NY - 1), cv2.CALIB_CB_ACCURACY)
    if not f:
        return 'GAGAL: sudut tidak terdeteksi'
    g = c.reshape(NY - 1, NX - 1, 2) / 300 * 25.4
    sx = np.linalg.norm(np.diff(g, axis=1), axis=2).mean()
    sy = np.linalg.norm(np.diff(g, axis=0), axis=2).mean()
    lo = g.reshape(-1, 2).min(0)               # urutan sudut SB sembarang -> pakai minimum, bukan sudut ke-0
    return 'OK %d sudut, kotak %.2f x %.2f mm, sudut kiri-atas pola (%.2f, %.2f) mm (harus %.1f, %.1f)' % (
        len(c), sx, sy, lo[0] - sx, lo[1] - sy, X0, Y0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out-dir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'boards'))
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    for name in ('A', 'B'):
        p = os.path.join(a.out_dir, 'g63b_board_%s.pdf' % name)
        make_pdf(p, name)
        h = hashlib.sha256(open(p, 'rb').read()).hexdigest()[:12]
        print(p, 'sha256', h, check(p) if a.check else '')


if __name__ == '__main__':
    main()
