#!/usr/bin/env python3
"""Generate a printable ChArUco calibration board (PNG + A4 PDF).

Used for the world->camera extrinsic calibration of the 2 overhead RealSense
cameras (see realsense_dual.launch.py tf1_*/tf2_* placeholders): print this
board, tape it flat at a KNOWN, measured position/orientation in the `world`
frame, then solve PnP per camera against its corner coordinates.

Board: 7x5 squares, ChArUco, DICT_5X5_250, square=28mm, marker=21mm (0.75x).
28mm x 7 = 196mm wide, x 5 = 140mm tall -- fits A4 (210x297mm) with margin.

PRINTING: the PDF places the board at its exact physical size on an A4 page.
Print at 100% / "actual size" -- NOT "fit to page" or "shrink to fit", or the
square size below is wrong and every downstream PnP/extrinsic will be off.
After printing, measure one square with a ruler; it must read 28.0 mm.

The printout also has ORIGIN/+X/+Y arrows baked in at the corner
calibrate_extrinsics.py treats as (0,0,0) -- when you tape the board down,
line those printed arrows up with your chosen world axes (arrows face-up
toward the ceiling cameras) and read the position of the ORIGIN dot with a
tape measure for --board-xyz. No need to reason about ChArUco's internal
corner-numbering convention; calibrate_extrinsics.py already matches it to
what's drawn here.

    python3 generate_charuco_board.py                # writes to /tmp
    python3 generate_charuco_board.py --out-dir .

Output: charuco_board.png (300 DPI), charuco_board.pdf (A4, print this one).
"""
import argparse
import os

import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

from charuco_common import (DICT_NAME, MARKER_LENGTH_M, SQUARE_LENGTH_M,
                            SQUARES_X, SQUARES_Y, build_board)

DPI = 300
_MARGIN_SQUARES = 1.3  # room for the ORIGIN/+X/+Y annotation


def _annotate_axes(board, gray_img):
    """Draw ORIGIN/+X/+Y arrows at charuco corner id0, matching the frame
    calibrate_extrinsics.py's solve_camera_pose() actually solves in (raw
    OpenCV corner Y/Z run backwards from this -- that function flips them to
    match this drawing, so this picture is the ground truth to tape up by)."""
    rgb = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    detector = aruco.CharucoDetector(board)
    corners, ids, _mc, _mi = detector.detectBoard(gray_img)
    idx = {int(i): j for j, i in enumerate(ids.ravel())}
    p0 = corners[idx[0]]
    p_x = corners[idx[1]]              # +X: raw image+x direction, unchanged
    p_y_raw = corners[idx[SQUARES_X - 1]]  # raw board Y+ (down-page)
    p_y = p0 - (p_y_raw - p0)             # corrected +Y: mirror to up-page

    o = tuple(np.round(p0).astype(int))
    ax = tuple(np.round(p0 + 1.15 * (p_x - p0)).astype(int))
    ay = tuple(np.round(p0 + 1.15 * (p_y - p0)).astype(int))
    cv2.circle(rgb, o, 10, (0, 0, 255), -1)
    cv2.arrowedLine(rgb, o, ax, (255, 0, 0), 6, tipLength=0.15)
    cv2.arrowedLine(rgb, o, ay, (0, 160, 0), 6, tipLength=0.15)
    cv2.putText(rgb, 'ORIGIN', (o[0] + 12, o[1] + 28),
               cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(rgb, '+X', (ax[0] + 8, ax[1]),
               cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(rgb, '+Y', (ay[0], ay[1] - 12),
               cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 160, 0), 2, cv2.LINE_AA)
    return cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)


def render_png(board, out_path):
    m_per_in = 0.0254
    margin_px = round(_MARGIN_SQUARES * SQUARE_LENGTH_M / m_per_in * DPI)
    width_px = round(SQUARES_X * SQUARE_LENGTH_M / m_per_in * DPI) + 2 * margin_px
    height_px = round(SQUARES_Y * SQUARE_LENGTH_M / m_per_in * DPI) + 2 * margin_px
    img = board.generateImage((width_px, height_px), marginSize=margin_px, borderBits=1)
    annotated = _annotate_axes(board, img)
    Image.fromarray(annotated).save(out_path, dpi=(DPI, DPI))
    return width_px, height_px


def render_pdf(png_path, out_path):
    # total printed size = pattern + the annotation margin baked into render_png's
    # PNG -- MUST match that image's physical extent exactly, or the PDF stretches
    # it and the 28mm squares no longer print at 28mm.
    page_w, page_h = A4
    board_w_mm = (SQUARES_X + 2 * _MARGIN_SQUARES) * SQUARE_LENGTH_M * 1000
    board_h_mm = (SQUARES_Y + 2 * _MARGIN_SQUARES) * SQUARE_LENGTH_M * 1000
    x0 = (page_w - board_w_mm * mm) / 2
    y0 = page_h - 30 * mm - board_h_mm * mm  # top margin 30mm for the label

    c = canvas.Canvas(out_path, pagesize=A4)
    c.drawImage(png_path, x0, y0, width=board_w_mm * mm, height=board_h_mm * mm)
    c.setFont('Helvetica', 10)
    label_y = page_h - 15 * mm
    c.drawString(15 * mm, label_y,
                 f'ChArUco {SQUARES_X}x{SQUARES_Y}  square={SQUARE_LENGTH_M*1000:.1f}mm  '
                 f'marker={MARKER_LENGTH_M*1000:.1f}mm  dict={DICT_NAME}')
    c.drawString(15 * mm, label_y - 5 * mm,
                 'PRINT AT 100% / ACTUAL SIZE (no "fit to page"). '
                 'Verify with a ruler: one square must measure 28.0 mm.')
    c.save()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out-dir', default='/tmp')
    args = ap.parse_args()

    board = build_board()
    png_path = os.path.join(args.out_dir, 'charuco_board.png')
    pdf_path = os.path.join(args.out_dir, 'charuco_board.pdf')

    w_px, h_px = render_png(board, png_path)
    render_pdf(png_path, pdf_path)

    print(f'wrote {png_path} ({w_px}x{h_px}px @ {DPI} DPI)')
    print(f'wrote {pdf_path}  <-- print THIS at 100% / actual size')
    print(f'board: {SQUARES_X}x{SQUARES_Y} squares, square={SQUARE_LENGTH_M*1000:.1f}mm, '
          f'marker={MARKER_LENGTH_M*1000:.1f}mm, dict={DICT_NAME}')


if __name__ == '__main__':
    main()
