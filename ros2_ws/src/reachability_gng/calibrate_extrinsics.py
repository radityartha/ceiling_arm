#!/usr/bin/env python3
"""Solve world->camera_optical extrinsics for the 2 overhead RealSense cameras
from one ChArUco snapshot each, against realsense_dual.launch.py's contract
(/<ns>/rgb, /<ns>/camera_info) -- see that launch file's tf1_*/tf2_* args and
generate_charuco_board.py for the printed board this expects.

Prerequisite: the board (charuco_common.py definition) is printed at 100%
scale, taped FLAT on a rigid surface, at a position/orientation you have
physically measured in the `world` frame -- pass that measurement in via
--board-xyz / --board-rpy. The printed sheet has "ORIGIN", "+X" and "+Y"
arrows on it (see generate_charuco_board.py) -- use THOSE, not the raw
OpenCV corner-numbering convention (its Y/Z axes run backwards from what
you'd expect and this script corrects for that internally). With that
printed labeling, board-rpy (0,0,0) means: printed +X arrow along world +X,
printed +Y arrow along world +Y, pattern face-up (local Z along world +Z).
If the board is mounted on a wall or at an angle, measure and pass the
actual roll/pitch/yaw (radians, fixed-axis xyz -- same convention
static_transform_publisher's --roll/--pitch/--yaw take), referenced to that
same printed-arrow frame.

    ros2 run reachability_gng realsense_dual.launch.py   # cameras must be up
    python3 calibrate_extrinsics.py \\
        --board-xyz 1.0 0.0 0.85 --board-rpy 0 0 0

Prints, per camera, the tf1_*/tf2_* values ready to paste into
`ros2 launch reachability_gng realsense_dual.launch.py tf1_x:=... ...`, plus
a reprojection-error sanity check and a debug image per camera in /tmp.
"""
from __future__ import annotations

import argparse

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from charuco_common import SQUARES_X, SQUARES_Y, build_board

NAMESPACES = ['rgbd', 'rgbd2']
CAPTURE_TIMEOUT_S = 15.0
REPROJ_WARN_PX = 1.0


class _Capture(Node):
    def __init__(self, namespaces):
        super().__init__('calibrate_extrinsics_capture')
        self.img = {ns: None for ns in namespaces}
        self.info = {ns: None for ns in namespaces}
        for ns in namespaces:
            self.create_subscription(
                Image, f'/{ns}/rgb',
                lambda m, ns=ns: self._on_img(ns, m), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, ns=ns: self._on_info(ns, m), 1)

    def _on_img(self, ns, msg):
        if self.img[ns] is None:
            self.img[ns] = msg

    def _on_info(self, ns, msg):
        if self.info[ns] is None:
            self.info[ns] = msg

    def ready(self):
        return all(self.img[ns] is not None and self.info[ns] is not None
                  for ns in self.img)


def _decode_rgb8(msg):
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    cols = msg.step // 3
    return a.reshape(msg.height, cols, 3)[:, :msg.width, :]


def capture_frames(namespaces, timeout_s):
    rclpy.init()
    node = _Capture(namespaces)
    end = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    try:
        while rclpy.ok() and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > end:
                missing = [ns for ns in namespaces
                          if node.img[ns] is None or node.info[ns] is None]
                raise TimeoutError(
                    f'timed out waiting for rgb+camera_info on: {missing} '
                    '-- is realsense_dual.launch.py running?')
        return {ns: (_decode_rgb8(node.img[ns]), node.info[ns]) for ns in namespaces}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _flip_to_drawing_frame(obj_pts):
    """Raw OpenCV chessboard/ChArUco object points have +Y toward the viewer
    (down the printed page) and +Z therefore INTO the page when the board is
    face-up -- backwards from the ORIGIN/+X/+Y printout convention this
    script's board-xyz/board-rpy are documented in. Flip Y,Z (proper 180 deg
    rotation about X, det=+1) into that frame."""
    obj_pts = obj_pts.copy()
    obj_pts[:, 1] *= -1
    obj_pts[:, 2] *= -1
    return obj_pts


def _solve_pnp_charuco(gray, K, dist, board):
    detector = aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if charuco_corners is None or len(charuco_corners) < 6:
        n_markers = 0 if marker_ids is None else len(marker_ids)
        n_corners = 0 if charuco_corners is None else len(charuco_corners)
        return None, (n_corners, n_markers, marker_corners, marker_ids)

    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    obj_pts = _flip_to_drawing_frame(obj_pts)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
    if not ok:
        return None, None

    def draw(rgb):
        debug = aruco.drawDetectedCornersCharuco(
            rgb.copy(), charuco_corners.reshape(-1, 1, 2), charuco_ids.reshape(-1, 1))
        cv2.drawFrameAxes(debug, K, dist, rvec, tvec, 0.05)
        return debug

    return (rvec, tvec, obj_pts, img_pts, draw), None


def _solve_pnp_plain_chessboard(gray, K, dist, board, board_xyz, board_rpy, camera_hint_xyz=None):
    """Fallback for when the board is too small in-frame for ArUco marker
    bit-pattern decoding (needs only a resolvable corner, not a decoded
    marker -- works at far lower px/square than ChArUco). A PLAIN
    checkerboard can't self-identify which physical corner is the ORIGIN, so
    try both possible 180 deg-rotated corner labelings.

    The 2 candidates are related by a 180 deg rotation about the BOARD's own
    normal. When the board is flat/face-up (the common case here) that
    normal is vertical, so the rotation does NOT change camera height --
    "pick the higher camera" is degenerate and effectively arbitrary in that
    orientation (verified: candidate heights matched to <1mm on real
    captures). It only discriminates when the board itself is tilted enough
    that "above vs below the board" is a meaningful distinction. Prefer a
    caller-supplied rough expected camera position (camera_hint_xyz) instead
    -- it only needs to be in the right quadrant/side, not precise, since it
    is merely breaking a symmetry, not replacing the PnP solve. Falls back to
    the old height heuristic when no hint is given."""
    pattern_size = (SQUARES_X - 1, SQUARES_Y - 1)
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    img_pts = corners.reshape(-1, 2)

    obj_raw = _flip_to_drawing_frame(board.getChessboardCorners())
    candidates = []
    for obj_try in (obj_raw, obj_raw[::-1]):  # the 2nd is the 180 deg relabeling
        ok, rvec, tvec = cv2.solvePnP(obj_try, img_pts, K, dist)
        if not ok:
            continue
        R_cb, _ = cv2.Rodrigues(rvec)
        t_wc, _ = world_from_camera(R_cb, tvec.flatten(), board_xyz, board_rpy)
        candidates.append((t_wc, rvec, tvec, obj_try))
    if not candidates:
        return None
    if camera_hint_xyz is not None:
        hint = np.array(camera_hint_xyz, dtype=np.float64)
        _, rvec, tvec, obj_pts = min(candidates, key=lambda c: np.linalg.norm(c[0] - hint))
    else:
        # degenerate fallback, kept for boards mounted at an angle
        _, rvec, tvec, obj_pts = max(candidates, key=lambda c: c[0][2])

    def draw(rgb):
        debug = cv2.drawChessboardCorners(rgb.copy(), pattern_size, corners, found)
        cv2.drawFrameAxes(debug, K, dist, rvec, tvec, 0.05)
        return debug

    return rvec, tvec, obj_pts, img_pts, draw


def solve_camera_pose(rgb, cam_info, board, ns, board_xyz, board_rpy, camera_hint_xyz=None):
    """Returns (R_cam_board, t_cam_board, reproj_rms_px, debug_image_path) or
    raises ValueError. Tries ChArUco (self-identifying corners) first, falls
    back to a plain chessboard-corner fit if the board is too small/far for
    ArUco's marker bit-patterns to resolve."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(cam_info.d, dtype=np.float64)

    result, fail_info = _solve_pnp_charuco(gray, K, dist, board)
    method = 'charuco'
    if result is None:
        fallback = _solve_pnp_plain_chessboard(gray, K, dist, board, board_xyz, board_rpy, camera_hint_xyz)
        if fallback is not None:
            result, method = fallback, 'plain-chessboard'

    if result is None:
        raw_path = f'/tmp/calib_debug_{ns}_raw.png'
        annotated = rgb.copy()
        if fail_info is not None:
            n_corners, n_markers, marker_corners, marker_ids = fail_info
            if marker_ids is not None and n_markers > 0:
                aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
        cv2.imwrite(raw_path, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        raise ValueError(
            f'[{ns}] neither ChArUco nor plain-chessboard corners were '
            f'detected -- check the board is fully visible, in focus, and '
            f'well lit. Raw capture saved: {raw_path}')

    rvec, tvec, obj_pts, img_pts, draw = result
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    rms_px = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts.reshape(-1, 2)) ** 2, axis=1))))

    out_path = f'/tmp/calib_debug_{ns}.png'
    cv2.imwrite(out_path, cv2.cvtColor(draw(rgb), cv2.COLOR_RGB2BGR))
    print(f'[{ns}] solved via {method}')

    R_cb, _ = cv2.Rodrigues(rvec)
    return R_cb, tvec.flatten(), rms_px, out_path


def world_from_camera(R_cb, t_cb, board_xyz, board_rpy):
    """Compose T_world_cam = T_world_board * inv(T_cam_board)."""
    R_wb = Rotation.from_euler('xyz', board_rpy).as_matrix()
    t_wb = np.array(board_xyz, dtype=np.float64)

    R_wc = R_wb @ R_cb.T
    t_wc = t_wb - R_wc @ t_cb
    roll, pitch, yaw = Rotation.from_matrix(R_wc).as_euler('xyz')
    return t_wc, (roll, pitch, yaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--board-xyz', nargs=3, type=float, required=True,
                    metavar=('X', 'Y', 'Z'),
                    help='measured world-frame position of the board origin corner (m)')
    ap.add_argument('--board-rpy', nargs=3, type=float, default=(0.0, 0.0, 0.0),
                    metavar=('ROLL', 'PITCH', 'YAW'),
                    help='measured world-frame orientation of the board (rad, '
                         'fixed-axis xyz); default 0 0 0 = flat, face-up')
    ap.add_argument('--namespaces', nargs='+', default=NAMESPACES)
    ap.add_argument('--camera-hint-xyz', nargs='+', type=float, default=None,
                    metavar='X1 Y1 Z1 X2 Y2 Z2 ...',
                    help='rough expected world-frame camera position, 3 floats per '
                         'namespace (same order as --namespaces). Only used to break '
                         'the plain-chessboard 180-degree corner-labeling ambiguity '
                         '(pick the candidate closest to this hint) -- does not need '
                         'to be precise, just in the right quadrant/side. Omit to fall '
                         'back to the (degenerate, for flat boards) height heuristic.')
    ap.add_argument('--timeout', type=float, default=CAPTURE_TIMEOUT_S)
    args = ap.parse_args()

    camera_hints = {}
    if args.camera_hint_xyz is not None:
        if len(args.camera_hint_xyz) != 3 * len(args.namespaces):
            ap.error(f'--camera-hint-xyz needs 3 floats per namespace '
                     f'({len(args.namespaces)} namespaces -> '
                     f'{3 * len(args.namespaces)} floats), got {len(args.camera_hint_xyz)}')
        for i, ns in enumerate(args.namespaces):
            camera_hints[ns] = tuple(args.camera_hint_xyz[3 * i:3 * i + 3])

    board = build_board()
    frames = capture_frames(args.namespaces, args.timeout)

    print(f'\nboard world pose: xyz={tuple(args.board_xyz)} rpy={tuple(args.board_rpy)}')
    results = {}
    for i, ns in enumerate(args.namespaces, start=1):
        rgb, cam_info = frames[ns]
        try:
            R_cb, t_cb, rms_px, debug_path = solve_camera_pose(
                rgb, cam_info, board, ns, args.board_xyz, args.board_rpy,
                camera_hints.get(ns))
        except ValueError as e:
            print(f'\n{e}')
            continue
        t_wc, (roll, pitch, yaw) = world_from_camera(
            R_cb, t_cb, args.board_xyz, args.board_rpy)
        results[ns] = (t_wc, roll, pitch, yaw)

        flag = '  <-- HIGH, recheck board pose / focus / corner count' if rms_px > REPROJ_WARN_PX else ''
        print(f'\n[{ns}] reprojection RMS = {rms_px:.3f} px{flag}')
        print(f'[{ns}] debug image: {debug_path}')
        print(f'[{ns}] world->{ns}_camera_optical:')
        print(f'  x={t_wc[0]:.4f} y={t_wc[1]:.4f} z={t_wc[2]:.4f} '
              f'roll={roll:.4f} pitch={pitch:.4f} yaw={yaw:.4f}')
        print(f'  tf{i}_x:={t_wc[0]:.4f} tf{i}_y:={t_wc[1]:.4f} tf{i}_z:={t_wc[2]:.4f} '
              f'tf{i}_roll:={roll:.4f} tf{i}_pitch:={pitch:.4f} tf{i}_yaw:={yaw:.4f}')

    if len(results) == len(args.namespaces):
        args_str = ' '.join(
            f'tf{i}_x:={t[0]:.4f} tf{i}_y:={t[1]:.4f} tf{i}_z:={t[2]:.4f} '
            f'tf{i}_roll:={r:.4f} tf{i}_pitch:={p:.4f} tf{i}_yaw:={y:.4f}'
            for i, (ns, (t, r, p, y)) in enumerate(results.items(), start=1))
        print('\nready-to-paste launch command:')
        print(f'  ros2 launch reachability_gng realsense_dual.launch.py {args_str}')
    else:
        print('\nnot all cameras solved -- fix the ones above and rerun before launching.')


if __name__ == '__main__':
    main()
