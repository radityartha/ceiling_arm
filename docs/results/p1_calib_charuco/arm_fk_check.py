#!/usr/bin/env python3
"""CALIB-CHARUCO A1.7 / DEMO-HW B1 method: camera depth points vs URDF arms (live TF), per camera x arm.

For each arm: axis xy = mean of forearm/lower_wrist/upper_wrist link origins (live TF = measured joints); camera points
with |xy - axis| < 0.25 m and z in [1.15, 1.45] (arm links only, above tables); median xy - axis, plus a 4 cm
link-radius correction toward the camera (surface faces the camera). Also: board centroid (known pose) vs each camera.
    python3 arm_fk_check.py --board 0.8505 0.1804 0.626 0.0064 [--out x.json]
"""
import argparse, json, math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

ARMS = {'arm_1': 't1_a1', 'arm_2': 't1_a2', 'arm_3': 't2_a1', 'arm_4': 't2_a2'}
ap = argparse.ArgumentParser(); ap.add_argument('--board', nargs=4, type=float); ap.add_argument('--out'); a = ap.parse_args()
rclpy.init(); n = Node('arm_fk_check'); tf = Buffer(); TransformListener(tf, n); got = {}
for ns in ('rgbd', 'rgbd2'):
    n.create_subscription(Image, f'/{ns}/depth', lambda m, ns=ns: got.__setitem__(ns + 'd', np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()), 1)
    n.create_subscription(CameraInfo, f'/{ns}/camera_info', lambda m, ns=ns: got.__setitem__(ns + 'K', np.array(m.k).reshape(3, 3)), 1)
t = time.time()
while time.time() - t < 6: rclpy.spin_once(n, timeout_sec=0.1)
def q2R(x, y, z, w): return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def look(t_, s):
    tr = tf.lookup_transform(t_, s, Time()).transform
    return q2R(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w), np.array([tr.translation.x, tr.translation.y, tr.translation.z])
axis = {}
for arm, p in ARMS.items():
    axis[arm] = np.mean([look('world', f'{p}_{l}')[1][:2] for l in ('forearm_link', 'lower_wrist_link', 'upper_wrist_link')], 0)
rep = {'axis': {k: v.round(4).tolist() for k, v in axis.items()}}
for ns in ('rgbd', 'rgbd2'):
    R, T = look('world', f'{ns}_camera_optical'); d = got[ns + 'd']; K = got[ns + 'K']
    v, u = np.nonzero(np.isfinite(d) & (d > 0.2) & (d < 6)); z = d[v, u]
    P = np.stack([(u-K[0, 2])*z/K[0, 0], (v-K[1, 2])*z/K[1, 1], z], 1) @ R.T + T
    for arm, (x0, y0) in axis.items():
        s = (np.hypot(P[:, 0]-x0, P[:, 1]-y0) < 0.25) & (P[:, 2] > 1.15) & (P[:, 2] < 1.45)
        if s.sum() < 100: continue
        md = np.median(P[s, :2], 0); vd = md - T[:2]; vd /= np.linalg.norm(vd); dd = md - [x0, y0] + 0.04 * vd
        rng = float(np.linalg.norm(np.r_[md, 1.3] - T))
        rep[f'{ns}:{arm}'] = dict(n=int(s.sum()), range_m=round(rng, 2), d_xy=dd.round(3).tolist(), d_abs=round(float(np.hypot(*dd)), 3))
        print(f'{ns:6s} {arm} range {rng:.2f} m  n {s.sum():6d}  dxy {dd.round(3)}  |d| {np.hypot(*dd):.3f}')
    if a.board:
        bx, by, bz, yaw = a.board; c, sn = math.cos(yaw), math.sin(yaw)
        ctr = np.array([bx + c*0.0875 - sn*0.1225, by + sn*0.0875 + c*0.1225, bz])
        s = (np.abs(P[:, 0]-ctr[0]) < 0.07) & (np.abs(P[:, 1]-ctr[1]) < 0.10) & (np.abs(P[:, 2]-bz) < 0.08)
        if s.sum() > 50:
            m = np.median(P[s], 0); rep[f'{ns}:board'] = dict(n=int(s.sum()), d=(m-ctr).round(3).tolist())
            print(f'{ns:6s} board center: median {m.round(3)} - known {ctr.round(3)} = {(m-ctr).round(3)} (n {s.sum()})')
        else:
            print(f'{ns:6s} board center: only {s.sum()} pts within window')
if a.out: json.dump(rep, open(a.out, 'w'), indent=1)
