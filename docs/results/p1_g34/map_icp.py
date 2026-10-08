#!/usr/bin/env python3
"""G34 A3: rigid ICP (6-DOF, point-to-point, trimmed) between two maps' voxel centres.
Same scenery, captured with the gantries elsewhere -> should coincide (< 2 cm / 0.3 deg).
    python3 map_icp.py a.npz b.npz"""
import sys

import numpy as np
from scipy.spatial import cKDTree

A = np.load(sys.argv[1])['centers'].astype(float)
B = np.load(sys.argv[2])['centers'].astype(float)
tree = cKDTree(B)
R, t = np.eye(3), np.zeros(3)
d0 = tree.query(A)[0]
for _ in range(30):
    X = A @ R.T + t
    d, j = tree.query(X)
    m = d < np.quantile(d, 0.7)
    P, Q = X[m], B[j[m]]
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    Rs = Vt.T @ D @ U.T
    R, t = Rs @ R, Rs @ t + cq - Rs @ cp
d = tree.query(A @ R.T + t)[0]
ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
# displacement of the map's own centroid (t alone is about the world origin)
c = A.mean(0)
print(f'{sys.argv[1]} -> {sys.argv[2]}: rot {ang:.3f} deg, t {t.round(4)}, '
      f'geser di centroid {np.linalg.norm(R @ c + t - c) * 100:.2f} cm, '
      f'median jarak {np.median(d0) * 100:.2f} -> {np.median(d) * 100:.2f} cm, n {len(A)}/{len(B)}')
