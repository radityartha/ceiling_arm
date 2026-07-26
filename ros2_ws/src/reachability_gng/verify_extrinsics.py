#!/usr/bin/env python3
"""Cross-camera extrinsic sanity check: find the SAME physical point (the
ChArUco/chessboard ORIGIN corner) in both cameras' live views, deproject it
to `world` INDEPENDENTLY using each camera's own depth + its calibrated
world->camera_optical TF (the one realsense_dual.launch.py is currently
publishing), and compare.

This does NOT assume any board position -- unlike calibrate_extrinsics.py's
own reprojection RMS (which only checks self-consistency against the board
pose YOU typed in), this checks whether the two INDEPENDENTLY calibrated
cameras agree on where a real point actually is. Small disagreement (a few
cm) = extrinsics consistent with each other. Large disagreement = at least
one camera's tf1_*/tf2_* is off.

    ros2 launch reachability_gng realsense_dual.launch.py   # must be running
    python3 verify_extrinsics.py
"""
from __future__ import annotations

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from charuco_common import SQUARES_X, SQUARES_Y, build_board
from calibrate_extrinsics import _flip_to_drawing_frame  # noqa: reuse

NAMESPACES = ['rgbd', 'rgbd2']
CAPTURE_TIMEOUT_S = 15.0


class _Capture(Node):
    def __init__(self, namespaces):
        super().__init__('verify_extrinsics_capture')
        self.rgb = {ns: None for ns in namespaces}
        self.depth = {ns: None for ns in namespaces}
        self.info = {ns: None for ns in namespaces}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for ns in namespaces:
            self.create_subscription(
                Image, f'/{ns}/rgb',
                lambda m, ns=ns: self._set('rgb', ns, m), qos_profile_sensor_data)
            self.create_subscription(
                Image, f'/{ns}/depth',
                lambda m, ns=ns: self._set('depth', ns, m), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, ns=ns: self._set('info', ns, m), 1)

    def _set(self, kind, ns, msg):
        getattr(self, kind)[ns] = msg

    def ready(self):
        return all(self.rgb[ns] is not None and self.depth[ns] is not None
                  and self.info[ns] is not None for ns in self.rgb)


def _decode_rgb8(msg):
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    cols = msg.step // 3
    return a.reshape(msg.height, cols, 3)[:, :msg.width, :]


def _decode_depth32f(msg):
    a = np.frombuffer(bytes(msg.data), dtype=np.float32)
    cols = msg.step // 4
    return a.reshape(msg.height, cols)[:, :msg.width]


def find_board_centroid_px(rgb, board):
    """Returns (u,v) pixel of the detected corners' centroid, via ChArUco if
    it locks, else the plain-chessboard fallback. Orientation/identity
    doesn't matter here -- we just need ONE consistent physical
    point per camera, not a full pose).

    Uses the CENTROID of all detected corners, not a specific named corner:
    the plain-chessboard fallback can't self-identify which corner is which
    (same 180 deg ambiguity calibrate_extrinsics.py's PnP solve has to
    disambiguate), so picking e.g. "the first found corner" would silently
    compare two DIFFERENT physical corners between cameras -- a centroid is
    invariant to that relabeling/ordering, sidestepping the problem
    entirely, at the cost of being a slightly fuzzier reference point."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    detector = aruco.CharucoDetector(board)
    cc, ci, _mc, _mi = detector.detectBoard(gray)
    if cc is not None and len(cc) >= 4:
        return cc.reshape(-1, 2).mean(axis=0)

    pattern_size = (SQUARES_X - 1, SQUARES_Y - 1)
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    return corners.reshape(-1, 2).mean(axis=0)


def deproject_to_world(u, v, depth_img, K, tf_buffer, world_frame, cam_frame, stamp):
    z = float(depth_img[int(round(v)), int(round(u))])
    if not np.isfinite(z) or z <= 0.0:
        return None, z
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x_cam = (u - cx) * z / fx
    y_cam = (v - cy) * z / fy
    p_cam = np.array([x_cam, y_cam, z])

    tf = tf_buffer.lookup_transform(world_frame, cam_frame, rclpy.time.Time())
    t = tf.transform.translation
    q = tf.transform.rotation
    # quat -> rotation matrix
    x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
        [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
        [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.array([t.x, t.y, t.z])
    p_world = R @ p_cam + T
    return p_world, z


def main():
    board = build_board()
    rclpy.init()
    node = _Capture(NAMESPACES)
    end = node.get_clock().now().nanoseconds + int(CAPTURE_TIMEOUT_S * 1e9)
    try:
        while rclpy.ok() and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > end:
                missing = [ns for ns in NAMESPACES
                          if node.rgb[ns] is None or node.depth[ns] is None
                          or node.info[ns] is None]
                print(f'timed out waiting for data on: {missing}')
                return
        # give tf2 buffer a moment to fill (static TF is latched, but the
        # listener needs at least one spin after subscribing to catch it)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        world_pts = {}
        for ns in NAMESPACES:
            rgb = _decode_rgb8(node.rgb[ns])
            depth = _decode_depth32f(node.depth[ns])
            K = np.array(node.info[ns].k, dtype=np.float64).reshape(3, 3)
            px = find_board_centroid_px(rgb, board)
            if px is None:
                print(f'[{ns}] board not detected in current view -- skipping')
                continue
            u, v = px
            p_world, z = deproject_to_world(
                u, v, depth, K, node.tf_buffer, 'world', f'{ns}_camera_optical',
                node.depth[ns].header.stamp)
            if p_world is None:
                print(f'[{ns}] invalid depth ({z}) at detected corner pixel ({u:.0f},{v:.0f})')
                continue
            world_pts[ns] = p_world
            print(f'[{ns}] board centroid at pixel ({u:.0f},{v:.0f}), depth={z:.3f}m '
                  f'-> world {p_world.round(4).tolist()}')

        if len(world_pts) == 2:
            a, b = world_pts[NAMESPACES[0]], world_pts[NAMESPACES[1]]
            d = np.linalg.norm(a - b)
            verdict = 'GOOD' if d < 0.05 else 'CHECK' if d < 0.15 else 'BAD'
            print(f'\ncross-camera agreement on the same physical point: '
                  f'{d*100:.1f} cm apart -> {verdict}')
        else:
            print('\ncould not compare -- board not detected in both cameras '
                  '(make sure it is still visible/lit in both views)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
