#!/usr/bin/env python3
"""G51 B3: model-free rail shift per g2 arm between two captures of the SAME session
(camera bias cancels): points near each arm (selected with each capture's own joints,
after cal_06 background subtraction), translation-only nearest-neighbour ICP B -> A.

    cd docs/results/p1_g34 && python3 ../p1_g51/g51_cloud_shift.py A.npz B.npz
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g33'))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g34'))
from build_env_map import capture_config, select_arm_points  # noqa: E402
from env_collision import RobotGeom  # noqa: E402
from rail_calib import background_mask, load_cam  # noqa: E402

REF = os.path.join(HERE, '..', 'p1_g34', 'cal_06.npz')


def arm_pts(rg, f, ns, arm, qf=None):
    q = rg.q_from(capture_config(qf or f, 'capture')[0])
    p, fr = load_cam(f, ns)
    pr, frr = load_cam(REF, ns)
    keep = background_mask(p, pr[frr >= 2])
    P, per = select_arm_points(rg, q, p[keep], fr[keep], (arm,), 100000)
    return P


def main():
    A, B = sys.argv[1:3]
    rg = RobotGeom()
    for ns, arm in (('rgbd', 't2_a2'), ('rgbd2', 't2_a1'), ('rgbd', 't1_a1'), ('rgbd2', 't1_a2')):
        Pa, Pb = arm_pts(rg, A, ns, arm), arm_pts(rg, B, ns, arm)
        # centroid with the SAME selection window (A's joints) for both captures
        Ca, Cb = arm_pts(rg, A, ns, arm), arm_pts(rg, B, ns, arm, qf=A)
        cshift = ((Cb.mean(0) - Ca.mean(0)) * 1000).round(1).tolist() if min(len(Ca), len(Cb)) else None
        if min(len(Pa), len(Pb)) < 100:
            print(json.dumps(dict(cam=ns, arm=arm, na=len(Pa), nb=len(Pb), skip=True)))
            continue
        tree = cKDTree(Pa)
        t = np.zeros(3)
        for k in range(30):
            d, i = tree.query(Pb + t)
            m = d < (0.05 if k < 10 else 0.02)
            t = t + (Pa[i[m]] - (Pb[m] + t)).mean(0)
        d, _ = tree.query(Pb + t)
        # shift of the arm from A to B = -t
        print(json.dumps(dict(cam=ns, arm=arm, na=len(Pa), nb=len(Pb), shift_mm=(-t * 1000).round(1).tolist(), centroid_shift_mm=cshift,
                              med_mm=round(float(np.median(d)) * 1000, 1))), flush=True)


if __name__ == '__main__':
    main()
