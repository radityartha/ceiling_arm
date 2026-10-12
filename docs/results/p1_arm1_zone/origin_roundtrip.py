#!/usr/bin/env python3
"""ARM1-ZONE A0 (i): synthetic round-trip for the calibrate_extrinsics ORIGIN-dot fix.

Ground truth = the PRINTED annotation (generate_charuco_board._annotate_axes: dot at ChArUco corner id0, +X toward id1,
+Y = id(SQUARES_X-1) mirrored), NOT the code's object-point convention. The board image is placed in world with the dot at
board_xyz / yaw, warped into a pinhole camera of known pose, solved by calibrate_extrinsics (ChArUco path near, plain-chessboard
fallback far), and the recovered camera position is compared with the truth. Negative control: the pre-fix convention (flip
only) must be off by ~one square (3.5 cm) -- a test that cannot fail on the old code would prove nothing.

    source /opt/ros/humble/setup.bash && python3 origin_roundtrip.py
"""
import os
import sys
from types import SimpleNamespace

import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../ros2_ws/src/reachability_gng'))
import calibrate_extrinsics as ce  # noqa: E402
from charuco_common import SQUARE_LENGTH_M, SQUARES_X, build_board  # noqa: E402

W, H = 1280, 720
K = np.array([[646.0, 0, 640.0], [0, 646.0, 360.0], [0, 0, 1]])
INFO = SimpleNamespace(k=K.ravel().tolist(), d=[0.0] * 5)


def printed_frame(board, px_per_sq=200, margin_sq=0.29):
    """Board image + its printed ORIGIN/+X/+Y frame in pixels (same detection as _annotate_axes)."""
    m = round(margin_sq * px_per_sq)
    img = board.generateImage((5 * px_per_sq + 2 * m, 7 * px_per_sq + 2 * m), marginSize=m, borderBits=1)
    corners, ids, _, _ = aruco.CharucoDetector(board).detectBoard(img)
    idx = {int(i): j for j, i in enumerate(ids.ravel())}
    p0, px, py_raw = (corners[idx[k]].ravel() for k in (0, 1, SQUARES_X - 1))
    s = SQUARE_LENGTH_M / np.linalg.norm(px - p0)               # m per px
    xh = (px - p0) / np.linalg.norm(px - p0)
    yh = -(py_raw - p0) / np.linalg.norm(py_raw - p0)           # printed +Y = mirrored raw Y
    return img, p0, xh, yh, s


def look_at(C, target, up=(0, 0, 1.0)):
    z = target - C; z /= np.linalg.norm(z)
    x = np.cross(z, up); x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)                  # R_world_cam (columns = cam axes)


def render(img, p0, xh, yh, s, board_xyz, yaw, C, R_wc):
    R_wb = Rotation.from_euler('z', yaw).as_matrix()
    h, w = img.shape
    src = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
    dst = []
    for u in src:
        X, Y = (u - p0) @ xh * s, (u - p0) @ yh * s
        Pc = R_wc.T @ (np.asarray(board_xyz) + R_wb @ [X, Y, 0.0] - C)
        dst.append((K @ Pc)[:2] / Pc[2])
    Hm = cv2.getPerspectiveTransform(src, np.float32(dst))
    gray = cv2.warpPerspective(img, Hm, (W, H), flags=cv2.INTER_AREA, borderValue=128)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)


def exact_case(board, frame, board_xyz, yaw, C, path):
    """Noise-free: printed-frame corner positions projected through the true camera, solved by calibrate_extrinsics'
    object points (ChArUco: id order; plain: findChessboardCorners stubbed to return them as-is AND reversed, so the
    hint-based 180-deg labeling choice is exercised too). Isolates the frame convention from corner detection."""
    img, p0, xh, yh, s = frame
    corners, ids, _, _ = aruco.CharucoDetector(board).detectBoard(img)
    pix = corners.reshape(-1, 2)[np.argsort(ids.ravel())]
    XY = np.stack([(pix - p0) @ xh * s, (pix - p0) @ yh * s], 1)
    C = np.asarray(C, float); R_wc = look_at(C, np.asarray(board_xyz) + [0.05, -0.09, 0])
    R_wb = Rotation.from_euler('z', yaw).as_matrix(); rpy = (0.0, 0.0, yaw)
    uv = ((np.asarray(board_xyz) + np.c_[XY, np.zeros(len(XY))] @ R_wb.T - C) @ R_wc) @ K.T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)
    if path == 'charuco':
        obj = ce._to_origin_dot_frame(board.getChessboardCorners(), board)
        ok, rv, tv = cv2.solvePnP(obj, uv, K, None)
        sols = [(cv2.Rodrigues(rv)[0], tv.ravel())]
    else:
        sols = []
        f0, s0 = cv2.findChessboardCorners, cv2.cornerSubPix
        try:
            for pts in (uv, uv[::-1].copy()):
                cv2.findChessboardCorners = lambda *a, pts=pts, **k: (True, pts.reshape(-1, 1, 2))
                cv2.cornerSubPix = lambda g, c, *a: c
                r = ce._solve_pnp_plain_chessboard(np.zeros((H, W), np.uint8), K, None, board, board_xyz, rpy,
                                                   camera_hint_xyz=C + [0.3, -0.2, 0.1])
                sols.append((cv2.Rodrigues(r[0])[0], r[1].ravel()))
        finally:
            cv2.findChessboardCorners, cv2.cornerSubPix = f0, s0
    out = []
    for R_cb, t_cb in sols:
        t_wc, e = ce.world_from_camera(R_cb, t_cb, board_xyz, rpy)
        dR = np.degrees(Rotation.from_matrix(Rotation.from_euler('xyz', e).as_matrix() @ R_wc.T).magnitude())
        out.append((float(np.linalg.norm(t_wc - C)), float(dR), (t_wc - C).round(4)))
    return out


def rendered_case(board, frame, board_xyz, yaw, C):
    """End-to-end ChArUco path on a warped image (detection noise included)."""
    C = np.asarray(C, float); R_wc = look_at(C, np.asarray(board_xyz) + [0.05, -0.09, 0])
    rgb = render(*frame, board_xyz, yaw, C, R_wc); rpy = (0.0, 0.0, yaw)
    assert ce._solve_pnp_charuco(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), K, np.zeros(5), board)[0] is not None
    R_cb, t_cb, rms, _ = ce.solve_camera_pose(rgb, INFO, board, 'synthetic', board_xyz, rpy)
    t_wc, e = ce.world_from_camera(R_cb, t_cb, board_xyz, rpy)
    dR = np.degrees(Rotation.from_matrix(Rotation.from_euler('xyz', e).as_matrix() @ R_wc.T).magnitude())
    return float(np.linalg.norm(t_wc - C)), float(dR), (t_wc - C).round(4)


CASES = [((0.8505, 0.1804, 0.626), 0.0064, (0.30, 0.65, 1.25)),     # near
         ((1.75, 0.55, 0.77), 0.40, (2.20, 1.10, 1.45)),             # yawed board
         ((0.8505, 0.1804, 0.626), 0.0064, (1.72, 0.87, 2.08)),      # rgbd-like
         ((0.8505, 0.1804, 0.626), 0.0064, (-0.15, -1.02, 2.09))]    # rgbd2-like


def main():
    board = build_board()
    frame = printed_frame(board)
    res = {}
    for conv in ('fixed', 'pre-fix'):
        orig = ce._to_origin_dot_frame
        if conv == 'pre-fix':
            ce._to_origin_dot_frame = lambda obj, b: ce._flip_to_drawing_frame(obj)   # NEGATIVE CONTROL
        try:
            for i, c in enumerate(CASES):
                for path in ('charuco', 'plain'):
                    for k, (e, dR, dt) in enumerate(exact_case(board, frame, *c, path)):
                        res.setdefault((conv, 'exact'), []).append(e)
                        print(f'{conv:8s} exact    case {i} {path:7s} labeling {k}  |dt| {e * 1000:9.6f} mm  dR {dR:.6f} deg  dt {dt}')
                if conv == 'fixed' and i < 2:
                    e, dR, dt = rendered_case(board, frame, *c)
                    res.setdefault((conv, 'rendered'), []).append(e)
                    print(f'{conv:8s} rendered case {i} charuco             |dt| {e * 1000:9.3f} mm  dR {dR:.3f} deg  dt {dt}')
        finally:
            ce._to_origin_dot_frame = orig
    fx, pf, rd = max(res[('fixed', 'exact')]), min(res[('pre-fix', 'exact')]), max(res[('fixed', 'rendered')])
    ok = fx < 1e-5 and pf > 0.03 and rd < 0.003
    print(f'\nexact fixed max {fx * 1000:.2e} mm (need < 1e-2: float32 px + solvePnP tolerance); exact pre-fix min {pf * 1000:.1f} mm (need > 30); '
          f'rendered ChArUco max {rd * 1000:.2f} mm (need < 3) -> {"PASS" if ok else "FAIL"}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
