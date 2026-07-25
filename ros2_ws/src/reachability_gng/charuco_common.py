"""Shared ChArUco board definition for generate_charuco_board.py and
calibrate_extrinsics.py -- keep both in sync, this is the single source
of truth for the physical board that gets printed and measured against.
"""
import cv2.aruco as aruco

SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.028
MARKER_LENGTH_M = 0.021
DICT_NAME = 'DICT_5X5_250'


def build_board():
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, DICT_NAME))
    return aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary)
