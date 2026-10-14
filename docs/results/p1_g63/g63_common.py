"""G63 shared definitions: boards, arms, incumbent extrinsics, SE3 helpers.

Single source of truth for every G63 tool (detector, solver, pose generator, capture). The board geometry comes
from make_g63_boards.build_board -- the same object that rendered the printed PDF.
"""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_g63_boards import BOARDS, SQUARE_MM, build_board  # noqa: E402

# Board -> carrying arm, link it is taped to (operator 2026-10-15: "jari gripper bagian yang datar"; WHICH finger link
# is confirmed from the operator's photo before the §A lock -- the solver does not care, it estimates T_lb).
BOARD_ARM = {'A': 'arm_2', 'B': 'arm_3'}
ARM_PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
DEFAULT_BOARD_LINK = 'right_finger_prox_link'

CAMERAS = ('rgbd', 'rgbd2')
IMG_W, IMG_H = 1280, 720
# Incumbent = launch defaults composed with the G34 correction (docs/results/p1_demo_hw/harness/cam_up.sh), frame
# world -> <ns>_camera_optical, static_transform_publisher x y z roll pitch yaw (scipy 'xyz', CAM-6DOF A0.4).
INCUMBENT = {
    'rgbd': (1.7166, 0.8656, 2.0782, -2.3436, -0.1488, 2.1688),
    'rgbd2': (-0.1547, -1.0197, 2.0905, -2.2369, -0.1285, -0.9189),
}
# Nominal colour intrinsics (camera_info is authoritative on HW; these are only for planning / synthetic tests).
K_NOMINAL = {
    'rgbd': np.array([[643.544, 0, 647.053], [0, 642.759, 365.949], [0, 0, 1.0]]),
    'rgbd2': np.array([[643.2, 0, 640.0], [0, 642.4, 360.0], [0, 0, 1.0]]),
}


def T_from_xyzrpy(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = (x, y, z)
    return T


def xyzrpy_from_T(T):
    return (*T[:3, 3], *Rotation.from_matrix(T[:3, :3]).as_euler('xyz'))


def T_from_vec(v):
    """6-vector (rotvec, t) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:6]
    return T


def vec_from_T(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rot_deg(Ra, Rb):
    """Angle (deg) between two rotation matrices."""
    return float(np.degrees(Rotation.from_matrix(Ra.T @ Rb).magnitude()))


def incumbent_T(cam):
    return T_from_xyzrpy(*INCUMBENT[cam])


def board_points(name, square_m=None):
    """{corner_id: (x, y, 0) in the board frame (m)}. square_m = MEASURED square size (printer scale)."""
    pts = np.asarray(build_board(name).getChessboardCorners(), float)
    if square_m is not None:
        pts = pts * (square_m / (SQUARE_MM / 1000.0))
    return {i: p for i, p in enumerate(pts)}


def board_extent(name):
    """Full printed pattern extent (m): 4 x 5 squares."""
    b = build_board(name)
    sx, sy = b.getChessboardSize()
    return sx * b.getSquareLength(), sy * b.getSquareLength()


def mean_T(Ts):
    """Chordal mean rotation + mean translation (for initialisation only)."""
    R = Rotation.from_matrix(np.array([T[:3, :3] for T in Ts])).mean().as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.mean([T_[:3, 3] for T_ in Ts], axis=0)
    return T


__all__ = ['BOARDS', 'BOARD_ARM', 'ARM_PREFIX', 'DEFAULT_BOARD_LINK', 'CAMERAS', 'IMG_W', 'IMG_H', 'INCUMBENT',
           'K_NOMINAL', 'T_from_xyzrpy', 'xyzrpy_from_T', 'T_from_vec', 'vec_from_T', 'inv', 'rot_deg',
           'incumbent_T', 'board_points', 'board_extent', 'mean_T', 'build_board']
