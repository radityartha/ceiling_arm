#!/usr/bin/env python3
"""CAM-6DOF step 1: raw data for the per-camera 6-DOF fit (cam6dof_fit.py). NO motion.

Per camera: per-pixel median of N aligned depth frames (/<ns>/depth, metres), K (/<ns>/camera_info), live TF
world -> <ns>_camera_optical (the extrinsic the frames were taken with). Per arm: URDF hull surface samples (~5 mm, same
sampling as arm_fk_check --arm-filter) + outward triangle normals, posed by live TF (measured joints) + the arm axis xy. Finger links (mimic joints,
no TF) are skipped and printed.
    python3 cam6dof_capture.py out.npz [--frames 10] [--ns rgbd rgbd2]
"""
import argparse, os, sys, time
import numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

ARMS = {'arm_1': 't1_a1', 'arm_2': 't1_a2', 'arm_3': 't2_a1', 'arm_4': 't2_a2'}
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--frames', type=int, default=10)
ap.add_argument('--ns', nargs='+', default=['rgbd', 'rgbd2']); a = ap.parse_args()
rclpy.init(); n = rclpy.create_node('cam6dof_capture'); tf = Buffer(); TransformListener(tf, n)
D = {ns: [] for ns in a.ns}; K = {}
for ns in a.ns:
    n.create_subscription(Image, f'/{ns}/depth', lambda m, ns=ns: D[ns].append(
        np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()), 2)
    n.create_subscription(CameraInfo, f'/{ns}/camera_info', lambda m, ns=ns: K.__setitem__(ns, np.array(m.k).reshape(3, 3)), 1)
t0 = time.time()
while time.time() - t0 < 30 and not all(len(D[ns]) >= a.frames and ns in K for ns in a.ns):
    rclpy.spin_once(n, timeout_sec=0.05)


def q2R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def look(s):
    tr = tf.lookup_transform('world', s, Time()).transform
    return (q2R(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w),
            np.array([tr.translation.x, tr.translation.y, tr.translation.z]),
            np.array([tr.translation.x, tr.translation.y, tr.translation.z, tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]))


out = {'t': time.time(), 'ns': np.array(a.ns)}
for ns in a.ns:
    if len(D[ns]) < a.frames or ns not in K:
        sys.exit(f'{ns}: only {len(D[ns])} depth frames / K {ns in K} in 30 s -- ABORT')
    S = np.stack(D[ns][-a.frames:]); S[~np.isfinite(S) | (S <= 0)] = np.nan
    with np.errstate(all='ignore'):
        out[f'{ns}_depth'] = np.nanmedian(S, 0).astype(np.float32)
    out[f'{ns}_K'] = K[ns]; out[f'{ns}_tf'] = look(f'{ns}_camera_optical')[2]
    print(f'{ns}: {a.frames} frames {S.shape[1:]} valid {np.isfinite(out[f"{ns}_depth"]).mean():.2f}  tf {out[f"{ns}_tf"][:3].round(4)}')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../scripts'))
from env_collision import RobotGeom
rg = RobotGeom()
for arm, p in ARMS.items():
    S, Nn = [], []
    for g in rg.geom.geometryObjects:
        if not g.name.startswith(p + '_'): continue
        fr = rg.model.frames[g.parentFrame].name
        if not tf.can_transform('world', fr, Time()):
            print(f'{arm}: {g.name} SKIPPED (no TF for {fr})'); continue
        R, T, _ = look(fr); M = g.placement; c = g.geometry
        V = np.array([c.points(i) for i in range(c.num_points)]) @ M.rotation.T + M.translation
        for k in range(c.num_polygons):
            tri = c.polygons(k); A, B, C = (V[tri[j]] for j in range(3))
            m = max(2, int(max(np.linalg.norm(B - A), np.linalg.norm(C - A)) / 0.005))
            u, w = np.meshgrid(np.linspace(0, 1, m), np.linspace(0, 1, m)); u, w = u.ravel(), w.ravel(); f = u + w <= 1
            S.append(((A + np.outer(u[f], B - A) + np.outer(w[f], C - A)) @ R.T + T))
            nv = np.cross(B - A, C - A); nv /= np.linalg.norm(nv) + 1e-12
            if nv @ (A - V.mean(0)) < 0: nv = -nv          # convex hull: outward = away from its centroid
            Nn.append(np.tile(nv @ R.T, (f.sum(), 1)))
    out[f'{arm}_surf'] = np.concatenate(S).astype(np.float32); out[f'{arm}_nrm'] = np.concatenate(Nn).astype(np.float32)
    out[f'{arm}_axis'] = np.mean([look(f'{p}_{l}')[1][:2] for l in ('forearm_link', 'lower_wrist_link', 'upper_wrist_link')], 0)
    print(f'{arm}: surface {len(out[f"{arm}_surf"])} pts, axis {out[f"{arm}_axis"].round(4)}')
np.savez_compressed(a.out, **out); print('saved', a.out)
