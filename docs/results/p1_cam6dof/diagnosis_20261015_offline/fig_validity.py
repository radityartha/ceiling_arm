#!/usr/bin/env python3
"""Figure: RGB + depth-validity overlay, first snapshot per camera/run (archived NPZ only)."""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import analyze_offline as A

HERE = os.path.dirname(os.path.abspath(__file__))
picks = {}
for run in A.RUNS:
    for cam, name, a, K, R, t in A.load_frames(run):
        picks.setdefault((run, cam), (name, a))
fig, ax = plt.subplots(len(picks), 2, figsize=(12, 3.4 * len(picks)))
for i, ((run, cam), (name, a)) in enumerate(picks.items()):
    ok = A.valid(a['depth'])
    ax[i, 0].imshow(a['rgb'])
    ax[i, 0].set_title(f'{run} {cam} {name} RGB')
    ax[i, 1].imshow(np.where(ok, a['depth'], np.nan), cmap='viridis', vmin=1, vmax=5)
    ax[i, 1].set_title(f'depth valid {ok.mean():.1%} (NaN/invalid = white)')
    for x in ax[i]:
        x.axis('off')
fig.tight_layout()
fig.savefig(os.path.join(HERE, 'validity.png'), dpi=70)
