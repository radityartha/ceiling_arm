#!/usr/bin/env python3
"""Figure: where can all four arms hold one beam together?

Small multiples over object height (columns) and beam yaw (rows). Each cell is
one object-centre position, shaded by how far the coordination gets:

    none  : no gantry can place its arm pair on its two handles
    one   : exactly one gantry pair can grip -- the other cannot follow
    both  : all four arms grip simultaneously  <- co-manipulation feasible

The "one" tier is the point of the figure: it is the region that looks reachable
per-gantry yet fails once both gantries must agree, which is exactly the coupling
a per-arm capability map cannot express.

    python3 scripts/plot_comanip.py --npz data/comanip/comanip_map.npz
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

# one hue, light -> dark: an ordinal "how much of the team can grip" ramp.
# Sequential, so the categorical CVD validator does not apply; the three steps
# separate by lightness, which also survives greyscale printing.
SHADES = ['#eef1f4', '#a8c8ee', '#2a78d6']
LABELS = ['no gantry pair', 'one gantry pair only', 'all four arms']
RAIL_Y = {0.36: 'gantry_1', -0.36: 'gantry_2'}
INK, MUTED = '#0b0b0b', '#52514e'


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--npz', default='data/comanip/comanip_map.npz')
    p.add_argument('--out', default='data/comanip/fig1_comanip_map')
    args = p.parse_args()

    d = np.load(args.npz)
    gx, gy, length = d['gx'], d['gy'], float(d['length'])

    # '_raw' holds the kinematics-only mask (before the arm-crossing proxy); it is
    # a companion series, not a cell, so it is excluded from the panel grid here.
    keys = [k for k in d.files
            if k.startswith('z') and not k.endswith(('_g1', '_g2', '_raw'))]
    zs = sorted({float(k.split('_')[0][1:]) for k in keys})
    yaws = sorted({float(k.split('yaw')[1]) for k in keys})

    fig, axes = plt.subplots(len(yaws), len(zs),
                             figsize=(3.5 * len(zs), 3.3 * len(yaws)),
                             squeeze=False, constrained_layout=True)
    cmap = ListedColormap(SHADES)
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)

    for r, yaw in enumerate(yaws):
        for c, z in enumerate(zs):
            ax = axes[r][c]
            key = f'z{z:.2f}_yaw{yaw:.2f}'
            both = d[key]
            # tier 1 = reachable by exactly one gantry pair, 2 = by both
            tier = (d[key + '_g1'].astype(int) + d[key + '_g2'].astype(int))
            tier = np.where(both, 2, np.minimum(tier, 1))

            ax.pcolormesh(gx, gy, tier.T, cmap=cmap, norm=norm, shading='nearest')

            for y, name in RAIL_Y.items():                    # gantry rails, 0..2 m
                ax.plot([0.0, 2.0], [y, y], color=INK, lw=1.6, alpha=0.55,
                        solid_capstyle='butt', zorder=3)
                ax.text(2.03, y, name, color=MUTED, fontsize=7,
                        va='center', ha='left', zorder=3)

            pct = 100.0 * both.mean()
            ax.set_title(f'z = {z:.1f} m,  yaw = {np.degrees(yaw):.0f}°\n'
                         f'{pct:.1f}% feasible', fontsize=10, color=INK)
            ax.set_aspect('equal')
            ax.tick_params(labelsize=8, colors=MUTED, length=2)
            for s in ax.spines.values():
                s.set_color('#d8d8d4')
            if r == len(yaws) - 1:
                ax.set_xlabel('x (m)', fontsize=9, color=MUTED)
            if c == 0:
                ax.set_ylabel('y (m)', fontsize=9, color=MUTED)

    fig.legend(handles=[Patch(facecolor=s, edgecolor='#d8d8d4', label=l)
                        for s, l in zip(SHADES, LABELS)],
               loc='lower center', ncol=3, frameon=False, fontsize=9,
               bbox_to_anchor=(0.5, -0.04), labelcolor=INK)
    # Name the actual handle layout: a 0.25 m block gripped at its top-face
    # corners and a 1.2 m beam gripped along its axis are different tasks, and a
    # figure labelled only "beam" would misreport the block run.
    handles = str(d['handles']) if 'handles' in d.files else 'line'
    what = (f'{length:.2f} m block, top-face corners' if handles == 'corners'
            else f'{length:.1f} m beam, handles along the axis')
    fig.suptitle(f'Co-manipulation capability map — {what}, 4 arms, '
                 '2 shared gantries', fontsize=12, color=INK)

    for ext in ('png', 'pdf'):
        path = f'{args.out}.{ext}'
        fig.savefig(path, dpi=300, bbox_inches='tight', facecolor='white')
        print('wrote', path)


if __name__ == '__main__':
    main()
