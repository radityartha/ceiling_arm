#!/usr/bin/env python3
"""CALIB-CHARUCO A1.7 / DEMO-HW B1 method: camera depth points vs URDF arms (live TF), per camera x arm.

For each arm: axis xy = mean of forearm/lower_wrist/upper_wrist link origins (live TF = measured joints); camera points
with |xy - axis| < 0.25 m and z in [1.15, 1.45] (arm links only, above tables); median xy - axis, plus a 4 cm
link-radius correction toward the camera (surface faces the camera). Also: board centroid (known pose) vs each camera.
--arm-filter TOL (ARM1-ZONE A0): same z band, xy window 0.40 m; keep only points <= TOL of the arm's URDF hull surface
(posed by live TF) after the best translation (xy grid +-0.25 m / 1 cm, then xyz ICP) -> shift + n dropped. Separates
"arm-shaped points sit at +0.15" (arm/gantry or camera) from "non-arm points pulled the median" (curtain/window/cable).
    python3 arm_fk_check.py --board 0.8505 0.1804 0.626 0.0064 [--arm-filter 0.03] [--out x.json]
"""
import argparse, json, math, os, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

ARMS = {'arm_1': 't1_a1', 'arm_2': 't1_a2', 'arm_3': 't2_a1', 'arm_4': 't2_a2'}
ap = argparse.ArgumentParser(); ap.add_argument('--board', nargs=4, type=float); ap.add_argument('--out')
ap.add_argument('--arm-filter', type=float, metavar='TOL'); a = ap.parse_args()
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
if a.arm_filter:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../scripts'))
    from scipy.spatial import cKDTree
    from env_collision import RobotGeom
    rg = RobotGeom(); surf = {}
    for arm, p in ARMS.items():    # hull surfaces, ~5 mm samples, posed by the same live TF as `axis`
        S = []
        for g in rg.geom.geometryObjects:
            if not g.name.startswith(p + '_'): continue
            R, T = look('world', rg.model.frames[g.parentFrame].name); M = g.placement; c = g.geometry
            V = np.array([c.points(i) for i in range(c.num_points)]) @ M.rotation.T + M.translation
            for k in range(c.num_polygons):
                tri = c.polygons(k); A, B, C = (V[tri[j]] for j in range(3))
                m = max(2, int(max(np.linalg.norm(B - A), np.linalg.norm(C - A)) / 0.005))
                u, w = np.meshgrid(np.linspace(0, 1, m), np.linspace(0, 1, m)); u, w = u.ravel(), w.ravel(); f = u + w <= 1
                S.append(((A + np.outer(u[f], B - A) + np.outer(w[f], C - A)) @ R.T + T))
        surf[arm] = cKDTree(np.concatenate(S))


def arm_filter(P, tree, tol):
    """Best translation t (camera - URDF) and the inlier mask (<= tol of the hull surface after t)."""
    P = P[np.random.default_rng(0).permutation(len(P))[:4000]]
    g = np.arange(-0.25, 0.2501, 0.01); best = (-1, np.zeros(3))
    for dx in g:
        for dy in g:
            t = np.array([dx, dy, 0.0]); n = int((tree.query(P - t, distance_upper_bound=tol)[0] <= tol).sum())
            if n > best[0]: best = (n, t)
    n0 = int((tree.query(P, distance_upper_bound=tol)[0] <= tol).sum()); t = best[1]
    for _ in range(10):
        d, i = tree.query(P - t, distance_upper_bound=tol); m = d <= tol
        if m.sum() < 10: break
        t = t + ((P[m] - t) - tree.data[i[m]]).mean(0)
    d, _ = tree.query(P - t); m = d <= tol
    return P, t, m, n0, d
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
        if a.arm_filter:
            w = (np.hypot(P[:, 0]-x0, P[:, 1]-y0) < 0.40) & (P[:, 2] > 1.15) & (P[:, 2] < 1.45)
            Pw, t, m, n0, d = arm_filter(P[w], surf[arm], a.arm_filter)
            rep[f'{ns}:{arm}']['filter'] = dict(tol=a.arm_filter, n_window=int(w.sum()), n_used=len(Pw), n_kept=int(m.sum()),
                                                n_dropped=int((~m).sum()), kept_frac=round(float(m.mean()), 3), kept_at_zero=n0,
                                                t=t.round(3).tolist(), t_xy_abs=round(float(np.hypot(*t[:2])), 3),
                                                kept_med_mm=round(float(np.median(d[m])) * 1000, 1) if m.any() else None)
            print(f'{"":6s}   arm-filter {a.arm_filter:.2f}: window {w.sum()} used {len(Pw)} kept {m.sum()} dropped {(~m).sum()} '
                  f'(kept at t=0: {n0})  t {t.round(3)}  |t_xy| {np.hypot(*t[:2]):.3f}')
    if a.board:
        bx, by, bz, yaw = a.board; c, sn = math.cos(yaw), math.sin(yaw)
        ctr = np.array([bx + c*0.0525 + sn*0.0875, by + sn*0.0525 - c*0.0875, bz])  # board = printed ORIGIN dot (A0 fix): centre (0.0525, -0.0875)
        s = (np.abs(P[:, 0]-ctr[0]) < 0.07) & (np.abs(P[:, 1]-ctr[1]) < 0.10) & (np.abs(P[:, 2]-bz) < 0.08)
        if s.sum() > 50:
            m = np.median(P[s], 0); rep[f'{ns}:board'] = dict(n=int(s.sum()), d=(m-ctr).round(3).tolist())
            print(f'{ns:6s} board center: median {m.round(3)} - known {ctr.round(3)} = {(m-ctr).round(3)} (n {s.sum()})')
        else:
            print(f'{ns:6s} board center: only {s.sum()} pts within window')
if a.out: json.dump(rep, open(a.out, 'w'), indent=1)
