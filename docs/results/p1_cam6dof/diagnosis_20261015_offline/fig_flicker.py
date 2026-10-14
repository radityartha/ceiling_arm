#!/usr/bin/env python3
"""Figure: rgbd depth validity across consecutive dual-run snapshots (flicker shape)."""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import analyze_offline as A

HERE = os.path.dirname(os.path.abspath(__file__))
fr = [(n, a) for c, n, a, K, R, t in A.load_frames('dual') if c == 'rgbd'][:8]
fig, ax = plt.subplots(2, 4, figsize=(14, 4.6))
rows = {}
for x, (n, a) in zip(ax.ravel(), fr):
    ok = A.valid(a['depth'])
    raw0 = (a['rawdepth'] == 0).mean()
    rows[n] = np.round(ok.mean(1)[::72], 2).tolist()
    x.imshow(ok, cmap='gray')
    x.set_title(f'{n} valid {ok.mean():.0%} raw0 {raw0:.0%}', fontsize=8)
    x.axis('off')
fig.tight_layout()
fig.savefig(os.path.join(HERE, 'flicker_rgbd.png'), dpi=60)
for n, r in rows.items():
    print(n, 'row-valid every 72 px:', r)
