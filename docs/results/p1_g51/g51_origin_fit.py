#!/usr/bin/env python3
"""G51 B2: g2 origin check, apples-to-apples per SINGLE arm.

The new capture (g1 parked at 1.50) shows rgbd only ONE g2 arm (t2_a2) -- the
G48 baseline used both (800 pts, yaw ~0), and a one-arm 4-DOF fit is
underdetermined (yaw -34 deg). So: same camera, same arm, same background
reference (cal_06), for cal_00 / cal_07 (G34, g2 = 0) and the new capture, fitted
(a) translation only (yaw fixed 0) and (b) 4-DOF like rail_calib.

    cd docs/results/p1_g34 && python3 ../p1_g51/g51_origin_fit.py --out ../p1_g51/g51_origin_fit.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g33'))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g34'))
from build_env_map import _nearest_on_arms, capture_config, icp_to_robot, select_arm_points  # noqa: E402
from env_collision import RobotGeom  # noqa: E402
from rail_calib import background_mask, load_cam  # noqa: E402

G34 = os.path.join(HERE, '..', 'p1_g34')
CAPS = [os.path.join(G34, 'cal_00.npz'), os.path.join(G34, 'cal_07.npz'),
        os.path.join(HERE, 'cal_g2_origin.npz')]
REF = os.path.join(G34, 'cal_06.npz')   # g2 at 1.45 m: scenery only
PAIRS = [('rgbd', 't2_a2'), ('rgbd', 't2_a1'), ('rgbd2', 't2_a1')]


def fit_t(rg, oMg, P, arm, iters=12):
    """Translation-only point-to-surface ICP (yaw fixed 0)."""
    aidx = [i for i, n in enumerate(rg.names) if n[:5] == arm]
    t = np.zeros(3)
    for k in range(iters):
        m, Y, d = _nearest_on_arms(rg, oMg, P + t, aidx, 0.25 if k < 3 else 0.12)
        if k == 0:
            before = float(np.median(np.abs(d[m])))
        t = t + (Y[m] - (P[m] + t)).mean(0)
    m, _, d = _nearest_on_arms(rg, oMg, P + t, aidx, 0.25)
    return t, before, float(np.median(np.abs(d[m]))), int(m.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--caps', nargs='+', help='default: cal_00, cal_07, cal_g2_origin')
    a = ap.parse_args()
    rg = RobotGeom()
    rows = []
    for f in a.caps or CAPS:
        cfg = capture_config(f, 'capture')[0]
        q = rg.q_from(cfg)
        oMg = rg.place(q)
        for ns, arm in PAIRS:
            p, fr = load_cam(f, ns)
            pr, frr = load_cam(REF, ns)
            keep = background_mask(p, pr[frr >= 2])
            p, fr = p[keep], fr[keep]
            P, per = select_arm_points(rg, q, p, fr, (arm,), 400)
            row = dict(capture=os.path.basename(f), cam=ns, arm=arm, n=per[0],
                       s2=round(cfg['t2_linear_joint'], 5), r2=round(cfg['t2_rotation_joint'], 5))
            if per[0] < 300:
                row['skip'] = '< 300 titik'
            else:
                t, b, aft, inl = fit_t(rg, oMg, P, arm)
                row.update(t3=t.round(4).tolist(), med3=[round(b, 4), round(aft, 4)], inl3=inl)
                _, _, rep = icp_to_robot(rg, [(rg.place(q), P)], (arm,), dof=4)
                row.update(t4=rep['t'], yaw4=round(rep['yaw_deg'], 2),
                           med4=round(rep['median_absd_after'], 4))
            rows.append(row)
            print(json.dumps(row), flush=True)
    json.dump(rows, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
