#!/usr/bin/env python3
"""G63e tugas 1: PDF penanda ArUco BESAR (satu per A4) + cek raster.

Vektor (sel = persegi terisi, hitam 100 % K) -> tidak ada interpolasi. Halaman: kotak hitam TAG di tengah horizontal, tepi
atas MARGIN_TOP; strip bawah = ID besar, label sudut C0..C3, petunjuk ukur, batang skala 100 mm (cek cetak 100 %).
Cek raster (pdftoppm 300 dpi): tiap halaman terdeteksi dengan ID benar, satu penanda, sisi hitam = TAG +- 0.3 mm,
skala 100 mm = 100 +- 0.3 mm. Gagal -> rc 1 (fail loud).

    python3 g63e_markers.py [--out g63e_markers.pdf] [--ids 20-35] [--check-only]
"""
import argparse
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402

MM = 72 / 25.4
BAR = 100.0      # mm


def draw(out, ids, tag_mm):
    from reportlab.lib.colors import CMYKColor, Color
    from reportlab.pdfgen import canvas
    black = CMYKColor(0, 0, 0, 1)
    W, H = E.PAPER[0] * 1000, E.PAPER[1] * 1000
    c = canvas.Canvas(out, pagesize=(W * MM, H * MM))
    d = E.dictionary()
    n = 6                                  # 4x4 data + 1 sel tepi tiap sisi
    cell = tag_mm / n
    x0 = (W - tag_mm) / 2
    top = E.MARGIN_TOP * 1000
    for i in ids:
        bits = cv2.aruco.generateImageMarker(d, i, n, borderBits=1)   # 6x6 px, 1 px per sel
        c.setFillColor(black)
        c.setStrokeColor(black)
        for r in range(n):
            for k in range(n):
                if bits[r, k] < 128:
                    # PDF: asal kiri-bawah; baris r dari atas
                    c.rect((x0 + k * cell) * MM, (H - top - (r + 1) * cell) * MM, cell * MM, cell * MM, stroke=0, fill=1)
        yb = H - top - tag_mm                                 # tepi bawah kotak hitam (mm dari bawah)
        c.setFont('Helvetica-Bold', 28)
        c.drawCentredString(W / 2 * MM, (yb - 32) * MM, 'ID %d' % i)
        c.setFont('Helvetica', 9)
        c.drawString((x0) * MM, (yb - 20) * MM, 'C3')
        c.drawRightString((x0 + tag_mm) * MM, (yb - 20) * MM, 'C2')
        lines = ['G63e  DICT_4X4_50  sisi hitam %.1f mm  -  CETAK 100 %% (ukuran sebenarnya, tanpa "fit to page")' % tag_mm,
                 'Sudut ukur = sudut LUAR kotak hitam: C0 kiri-atas, C1 kanan-atas, C2 kanan-bawah, C3 kiri-bawah.',
                 'Ukur sisi hitam C0-C1 dan C0-C3 (mm). Bagian bawah garis putus boleh digunting (penanda vertikal).',
                 'Jangan menulis / menempel selotip di dalam atau 15 mm di sekitar kotak hitam.']
        for j, s in enumerate(lines):
            c.setFont('Helvetica', 8)
            c.drawString(12 * MM, (yb - 42 - 4.2 * j) * MM, s)
        # batang skala 100 mm
        yb2 = yb - 66
        c.setLineWidth(0.6)
        c.line((W / 2 - BAR / 2) * MM, yb2 * MM, (W / 2 + BAR / 2) * MM, yb2 * MM)
        for t in range(0, 101, 10):
            h = 4 if t % 50 == 0 else 2.5
            c.line((W / 2 - BAR / 2 + t) * MM, yb2 * MM, (W / 2 - BAR / 2 + t) * MM, (yb2 + h) * MM)
        c.rect((W / 2 - BAR / 2) * MM, (yb2 - 4) * MM, 1.0 * MM, 3 * MM, stroke=0, fill=1)        # penanda ujung cek raster
        c.rect((W / 2 + BAR / 2 - 1.0) * MM, (yb2 - 4) * MM, 1.0 * MM, 3 * MM, stroke=0, fill=1)
        c.setFont('Helvetica', 8)
        c.drawCentredString(W / 2 * MM, (yb2 - 7) * MM, 'skala 100 mm (cek cetak)')
        # garis gunting: core = TAG + 2 margin
        c.setDash(2, 2)
        c.setStrokeColor(Color(0.6, 0.6, 0.6))
        yc = H - (2 * E.MARGIN_TOP * 1000 + tag_mm)
        c.line(0, yc * MM, W * MM, yc * MM)
        c.setDash()
        c.showPage()
    c.save()


def check(pdf, ids, tag_mm, dpi=300):
    rc = 0
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(['pdftoppm', '-r', str(dpi), '-gray', '-png', pdf, td + '/p'], check=True)
        pages = sorted(f for f in os.listdir(td) if f.endswith('.png'))
        if len(pages) != len(ids):
            print('GAGAL: halaman %d != ID %d' % (len(pages), len(ids)))
            return 1
        det = cv2.aruco.ArucoDetector(E.dictionary(), cv2.aruco.DetectorParameters())
        px_mm = dpi / 25.4
        for f, i in zip(pages, ids):
            g = cv2.imread(os.path.join(td, f), cv2.IMREAD_GRAYSCALE)
            corners, got, _ = det.detectMarkers(g)
            got = [] if got is None else got.ravel().tolist()
            ok = got == [i]
            side = np.linalg.norm(np.diff(np.r_[corners[0][0], corners[0][0][:1]], axis=0), axis=1) / px_mm if ok else [np.nan]
            # skala: dua blok ujung di baris bar
            blk = (g < 128).astype(np.uint8)
            n, lab, st, cen = cv2.connectedComponentsWithStats(blk)
            small = [cen[k] for k in range(1, n) if abs(st[k, 2] / px_mm - 1.0) < 0.3 and abs(st[k, 3] / px_mm - 3.0) < 0.4]
            bar = None
            if len(small) == 2:
                bar = (abs(small[1][0] - small[0][0]) / px_mm) + 1.0       # pusat-ke-pusat + 2 x setengah lebar = luar-luar - 0
            # nilai hitam (pekat): median piksel di sel hitam terluar
            ok_side = ok and np.all(np.abs(np.asarray(side) - tag_mm) <= 0.3)
            ok_bar = bar is not None and abs(bar - BAR) <= 0.3
            if not (ok_side and ok_bar):
                rc = 1
            print('hal %s ID %d: deteksi %s sisi %s mm  skala %s mm  %s' % (f, i, got, np.round(side, 2).tolist(),
                  None if bar is None else round(bar, 2), 'OK' if ok_side and ok_bar else 'GAGAL'))
    return rc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(E.HERE, 'g63e_markers.pdf'))
    ap.add_argument('--ids', default='%d-%d' % (E.IDS[0], E.IDS[-1]))
    ap.add_argument('--tag-mm', type=float, default=E.TAG_NOMINAL * 1000)
    ap.add_argument('--check-only', action='store_true')
    a = ap.parse_args()
    lo, hi = map(int, a.ids.split('-'))
    ids = list(range(lo, hi + 1))
    if any(i < E.ID_MIN_OLD for i in ids):
        print('GAGAL: ID < %d bentrok dengan papan lama' % E.ID_MIN_OLD)
        return 1
    if not a.check_only:
        draw(a.out, ids, a.tag_mm)
    return check(a.out, ids, a.tag_mm)


if __name__ == '__main__':
    sys.exit(main())
