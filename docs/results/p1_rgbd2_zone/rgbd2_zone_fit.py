#!/usr/bin/env python3
"""RGBD2-ZONE: planar 3-DOF (x, y, yaw about world z) correction of the rgbd2 extrinsic from bottle tracks at arm_1 plumb marks.

Correspondence = bottle TRACK (whole demo chain: YOLOE -> object_localizer, old extrinsic) <-> mark xy (arm_1 TCP from measured
joints). Fit marks only; held-out marks are reported, never fitted. T_corr = [Rz(th), (tx, ty, 0)] in world; new camera pose
T_wc_new = T_corr * T_wc_old, printed as static_transform_publisher args (R = Rz(yaw) Ry(pitch) Rx(roll) = scipy 'xyz').
Valid ONLY for the table zone it was fitted in (same view direction, same height).

    python3 rgbd2_zone_fit.py data.json [--out fit.json]
data.json: {"old": [x, y, z, roll, pitch, yaw], "marks": {name: [x, y]}, "tracks": {name: [x, y, z]},
            "fit": [names], "holdout": [names]}
    python3 rgbd2_zone_fit.py --self-test
"""
import argparse, json, sys
import numpy as np
from scipy.spatial.transform import Rotation


def fit_planar(src, dst):
    """Least-squares 2D rigid (Kabsch) src -> dst: returns th, t (2,)."""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    th = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return th, cd - R @ cs


def apply(th, t, p):
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return np.asarray(p)[..., :2] @ R.T + t


def new_extrinsic(old, th, t):
    R_old = Rotation.from_euler('xyz', old[3:]).as_matrix()
    Rz = Rotation.from_euler('z', th).as_matrix()
    p = Rz @ np.asarray(old[:3]) + [t[0], t[1], 0.0]
    R_new = Rz @ R_old
    return np.r_[p, Rotation.from_matrix(R_new).as_euler('xyz')], R_new


def run(d):
    M, T = d['marks'], d['tracks']
    src = np.array([T[k][:2] for k in d['fit']]); dst = np.array([M[k] for k in d['fit']])
    th, t = fit_planar(src, dst)
    rep = dict(th_deg=float(np.degrees(th)), t=t.round(4).tolist(), rows={})
    for k in list(d['fit']) + list(d['holdout']):
        before = np.asarray(T[k][:2]) - M[k]; after = apply(th, t, T[k]) - M[k]
        rep['rows'][k] = dict(role='fit' if k in d['fit'] else 'HOLDOUT', before=before.round(4).tolist(),
                              before_abs=round(float(np.hypot(*before)), 4), after=after.round(4).tolist(),
                              after_abs=round(float(np.hypot(*after)), 4))
    new, R_new = new_extrinsic(d['old'], th, t)
    rep['new'] = new.round(4).tolist()
    rep['new_quat_xyzw'] = Rotation.from_matrix(R_new).as_quat().round(6).tolist()
    rep['args'] = ('tf2_x:={:.4f} tf2_y:={:.4f} tf2_z:={:.4f} tf2_roll:={:.4f} tf2_pitch:={:.4f} tf2_yaw:={:.4f}'.format(*new))
    return rep


def self_test():
    rng = np.random.default_rng(3); ok = True
    old = [-0.1547, -1.0197, 2.0905, -2.2369, -0.1285, -0.9189]
    # 1) euler convention round trip: new_extrinsic with zero correction reproduces old matrix
    e, R = new_extrinsic(old, 0.0, np.zeros(2))
    ok &= np.allclose(Rotation.from_euler('xyz', e[3:]).as_matrix(), Rotation.from_euler('xyz', old[3:]).as_matrix(), atol=1e-12)
    # 2) known planar error recovered from 4 marks (+1 cm noise) and held-out predicted
    th0, t0 = np.radians(2.5), np.array([-0.05, -0.20])
    marks = {f'M{i}': p for i, p in enumerate([[1.75, 0.55], [2.0, 0.55], [1.85, 0.68], [2.05, 0.68], [1.9, 0.6], [1.7, 0.75]])}
    true_err = lambda p: apply(-th0, np.zeros(2), np.asarray(p) - t0)     # tracks = inverse of the correction
    tracks = {k: list(true_err(p)) + [0.83] for k, p in marks.items()}
    for noise in (0.0, 0.01):
        tr = {k: list(np.r_[v[:2] + rng.normal(0, noise, 2) * (k not in ('M4', 'M5')), v[2]]) for k, v in tracks.items()}
        r = run(dict(old=old, marks=marks, tracks=tr, fit=['M0', 'M1', 'M2', 'M3'], holdout=['M4', 'M5']))
        ho = max(r['rows'][k]['after_abs'] for k in ('M4', 'M5'))
        print(f"noise {noise}: th {r['th_deg']:.4f} (true {np.degrees(th0):.4f}) t {r['t']} (true {t0}) holdout max {ho * 1000:.2f} mm")
        ok &= (abs(r['th_deg'] - np.degrees(th0)) < 1e-9 and np.allclose(r['t'], t0, atol=1e-4)) if noise == 0 else ho < 0.03
    # 3) moving the camera by T_corr moves every world point it sees by T_corr (consistency of the composition)
    e1, R1 = new_extrinsic(old, th0, t0)
    pc = np.array([0.3, -0.2, 2.5]); R_old = Rotation.from_euler('xyz', old[3:]).as_matrix()
    w_old = R_old @ pc + old[:3]; w_new = R1 @ pc + e1[:3]
    ok &= np.allclose(w_new[:2], apply(th0, t0, w_old), atol=1e-12) and abs(w_new[2] - w_old[2]) < 1e-12
    print('SELF-TEST', 'PASS' if ok else 'FAIL'); return ok


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('data', nargs='?'); ap.add_argument('--out'); ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test: sys.exit(0 if self_test() else 1)
    rep = run(json.load(open(a.data)))
    print(json.dumps(rep, indent=1))
    if a.out: json.dump(rep, open(a.out, 'w'), indent=1)
