"""Shared ChArUco board definition for generate_charuco_board.py and
calibrate_extrinsics.py -- keep both in sync, this is the single source
of truth for the physical board that gets printed and measured against.

Sized for A4 PORTRAIT (210x297 mm), the only paper available here:
5x7 squares at 35 mm = 175x245 mm of pattern, 196x267 mm once the
ORIGIN/+X/+Y annotation margin is added -- the largest that still leaves a
printable border, with a 22 mm strip reserved at the top for the label.
Replaces the original 7x5 @ 28 mm (196x140 mm landscape), which wasted A4's
long dimension: same 24 inner corners, each square 25% larger. A 4x6 @ 41 mm
layout fits bigger squares but drops to 15 corners, which conditions the PnP
solve worse -- not worth the trade.

** Expect the plain-chessboard fallback, not ArUco marker decoding. ** At the
~2 m ceiling-to-floor distance with these cameras (fx ~646 px), a 35 mm
square projects to only ~11 px and its marker to ~8 px -- far below the ~40 px
a 5x5 marker needs to decode. Reaching that would take ~16 cm squares, which
do not fit on A4. This is fine: calibrate_extrinsics.py's
findChessboardCorners fallback only needs a resolvable corner and has
measured 0.31 px reprojection RMS on real captures. Marker decoding would
only add automatic origin-corner identification, which the fallback recovers
by testing both 180-degree labelings.
"""
import cv2.aruco as aruco

SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_M = 0.035
MARKER_LENGTH_M = 0.026
DICT_NAME = 'DICT_5X5_250'


def build_board():
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))
    return aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary)
