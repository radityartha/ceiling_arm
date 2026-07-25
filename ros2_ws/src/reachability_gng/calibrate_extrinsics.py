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

from charuco_common import build_board

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


def solve_camera_pose(rgb, cam_info, board, ns):
    """Returns (R_cam_board, t_cam_board, reproj_rms_px) or raises ValueError."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
    dist = np.array(cam_info.d, dtype=np.float64)

    detector = aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if charuco_corners is None or len(charuco_corners) < 6:
        n = 0 if charuco_corners is None else len(charuco_corners)
        raise ValueError(
            f'[{ns}] only {n} ChArUco corners detected (need >=6) -- check '
            'the board is fully visible, in focus, and well lit')

    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    # OpenCV's raw ChArUco object frame has +Y running toward the viewer
    # (down the printed page) and +Z therefore pointing INTO the page when
    # the board is face-up -- both backwards from the "+X/+Y arrows on the
    # printout, Z up when face-up" convention this script's board-xyz/
    # board-rpy are documented in. Flip Y and Z (a proper 180 deg rotation
    # about X, det=+1) so solvePnP's R_cb is expressed in THAT frame instead.
    obj_pts = obj_pts.copy()
    obj_pts[:, 1] *= -1
    obj_pts[:, 2] *= -1
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
    if not ok:
        raise ValueError(f'[{ns}] solvePnP failed')

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    rms_px = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts.reshape(-1, 2)) ** 2, axis=1))))

    debug = cv2.aruco.drawDetectedCornersCharuco(rgb.copy(), charuco_corners, charuco_ids)
    cv2.drawFrameAxes(debug, K, dist, rvec, tvec, 0.05)
    out_path = f'/tmp/calib_debug_{ns}.png'
    cv2.imwrite(out_path, cv2.cvtColor(debug, cv2.COLOR_RGB2BGR))

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
    ap.add_argument('--timeout', type=float, default=CAPTURE_TIMEOUT_S)
    args = ap.parse_args()

    board = build_board()
    frames = capture_frames(args.namespaces, args.timeout)

    print(f'\nboard world pose: xyz={tuple(args.board_xyz)} rpy={tuple(args.board_rpy)}')
    results = {}
    for i, ns in enumerate(args.namespaces, start=1):
        rgb, cam_info = frames[ns]
        try:
            R_cb, t_cb, rms_px, debug_path = solve_camera_pose(rgb, cam_info, board, ns)
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
