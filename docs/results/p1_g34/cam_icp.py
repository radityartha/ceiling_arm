#!/usr/bin/env python3
"""G34 A3 (B-over-A): with ONE fixed correction per camera the A<->B map ICP is the
identity by construction (same fixed cameras, same transform). The meaningful
consistency check is CROSS-CAMERA: rgbd vs rgbd2 raw 1 cm clouds of one capture,
each with its correction, static scene only (crop, z >= 0.30, robot cells far).
Trimmed point-to-point ICP rgbd2 -> rgbd on the overlap.
    python3 cam_icp.py cal_00.npz rail_joint4_diff.json"""
import json
import sys

import numpy as np
from scipy.spatial import cKDTree

cap = np.load(sys.argv[1])
T = json.load(open(sys.argv[2])) if len(sys.argv) > 2 else None
CROP = dict(x=(-1.2, 3.0), y=(-2.0, 2.0), z=(0.30, 2.00))


def cloud(ns):
    p = cap[f'{ns}_xyz'].astype(float)[cap[f'{ns}_frames'] >= 3]
    if T:
        p = p @ np.array(T[ns]['R']).T + np.array(T[ns]['t'])
    m = np.ones(len(p), bool)
    for i, ax in enumerate('xyz'):
        m &= (p[:, i] >= CROP[ax][0]) & (p[:, i] <= CROP[ax][1])
    return p[m]


A, B = cloud('rgbd2'), cloud('rgbd')
tree = cKDTree(B)
d0 = tree.query(A, distance_upper_bound=0.10)[0]
ov = np.isfinite(d0)                       # overlap: within 10 cm of the other camera
A = A[ov]
R, t = np.eye(3), np.zeros(3)
for _ in range(30):
    X = A @ R.T + t
    d, j = tree.query(X)
    m = d <= max(np.quantile(d, 0.7), 1e-9)
    P, Q = X[m], B[j[m]]
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rs = Vt.T @ D @ U.T
    R, t = Rs @ R, Rs @ t + cq - Rs @ cp
d = tree.query(A @ R.T + t)[0]
ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
c = A.mean(0)
print(f'{sys.argv[1]} {"terkoreksi" if T else "mentah"}: rgbd2->rgbd rot {ang:.3f} deg, geser di centroid '
      f'{np.linalg.norm(R @ c + t - c) * 100:.2f} cm {(R @ c + t - c).round(3)}, median jarak '
      f'{np.median(d0[ov]) * 100:.2f} -> {np.median(d) * 100:.2f} cm, overlap {ov.sum()} titik, centroid {c.round(2)}')
