#!/usr/bin/env python3
"""D601 literal: pooled median floor ratio for rgbd2 pixels with predicted Z >= 2.5 m (same RANSAC as U2)."""
import numpy as np

import analyze_offline as A

rng = np.random.default_rng(0)
pool = []
for cam, name, a, K, R, t in A.load_frames('dual'):
    if cam != 'rgbd2':
        continue
    d = a['depth']
    ok = A.valid(d)
    v, u = np.nonzero(ok)
    z = d[v, u]
    p = A.rays(u, v, K) * z[:, None] @ R.T + t
    near = np.abs(p[:, 2]) < 0.20
    q = p[near]
    best = None
    for _ in range(300):
        s = q[rng.choice(len(q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((q - s[0]) @ n) < 0.02
        if best is None or inl.sum() > best.sum():
            best = inl
    ui, vi, zi = u[near][best], v[near][best], z[near][best]
    _, zp = A.ray_plane(ui, vi, K, R, t, 0.0)
    m = zp >= 2.5
    pool.append(zi[m] / zp[m])
r = np.concatenate(pool)
print(f'rgbd2 pooled Z>=2.5 m: n={len(r)} median ratio={np.median(r):.4f}')
