#!/usr/bin/env python3
"""Figure 2: energy-greedy allocation cannot complete a 4-arm co-manipulation.

Reads data/marl/summary.csv (scripts/run_comanip_marl.py) and lays out:

  * a HERO band for the headline -- how many episodes delivered the 4-arm block.
    That measure is binary and separates completely, so it is a number, not a
    two-bar chart.
  * three small multiples for the measures that have a scale: how many arms ever
    acted on the block at once, energy per DELIVERED item, and the share of
    energy burnt on lifts that failed.

Energy per DELIVERED item is the honest efficiency metric: greedy's total J is
lower only because it delivers fewer items, so total energy alone would flatter
it.

    python3 scripts/plot_coalition.py --csv data/marl/summary.csv
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Path, PathPatch

# Categorical slots 1 and 2 of the reference palette, unmodified: that pair is
# documented as passing the light-mode adjacent-pair gates (CVD dE 9.1,
# normal-vision 19.6) and neither is one of the slots flagged below 3:1 on a
# light surface. Colour follows the POLICY (the entity), never its rank.
COLOR = {'auction-coalition': '#2a78d6', 'greedy-energy': '#eb6834'}
ORDER = ['greedy-energy', 'auction-coalition']          # baseline first
SURFACE, INK, MUTED, HAIR = '#fcfcfb', '#0b0b0b', '#52514e', '#d8d8d4'


def rounded_bar(ax, x, w, h, color, r_px=4):
    """A bar with 4px-rounded data-end corners, anchored square to the baseline.

    Anchored means the baseline corners stay square: a rounded foot would lift
    the mark off its own zero line and misstate the value.

    The corner radius is converted from pixels into x- and y-data units
    SEPARATELY -- the two axes have different scales here, so a single radius
    reused for both would come out visibly lopsided (and vanish on one axis).
    """
    if h <= 0:
        return
    inv = ax.transData.inverted()
    (ax0, ay0), (ax1, ay1) = inv.transform((0.0, 0.0)), inv.transform((r_px, r_px))
    rx, ry = min(abs(ax1 - ax0), w / 2), min(abs(ay1 - ay0), abs(h) / 2)
    x0, x1 = x - w / 2, x + w / 2
    verts = [(x0, 0), (x0, h - ry), (x0, h), (x0 + rx, h),
             (x1 - rx, h), (x1, h), (x1, h - ry), (x1, 0), (x0, 0)]
    codes = [Path.MOVETO, Path.LINETO, Path.CURVE3, Path.CURVE3,
             Path.LINETO, Path.CURVE3, Path.CURVE3, Path.LINETO, Path.CLOSEPOLY]
    ax.add_patch(PathPatch(Path(verts, codes), facecolor=color, edgecolor='none',
                           zorder=3))


def panel(ax, rows, key, title, note, fmt='{:.2f}', ymax=None, ref=None,
          ref_label=''):
    """One small multiple: mean per policy, with per-seed dots when they vary."""
    vals = {p: np.array([r[key] for r in rows if r['policy'] == p], float)
            for p in ORDER}
    # Headroom must clear the per-seed cloud AND the label above it, not just the
    # mean -- sizing from the mean alone pushed labels out through the title.
    top = ymax or max(max(v.max(), v.mean()) for v in vals.values()) * 1.22
    # BOTH limits must be final before any bar is drawn: rounded_bar converts a
    # pixel radius through transData, which is wrong if a limit changes later.
    ax.set_ylim(0, top)
    ax.set_xlim(-0.55, 1.55)
    for i, p in enumerate(ORDER):
        v = vals[p]
        rounded_bar(ax, i, 0.38, v.mean(), COLOR[p])
        # Direct-label both bars: with two marks this IS the selective case, and
        # it keeps identity off colour alone.
        if v.std() > 1e-9:                       # show the spread only if real
            rng = np.random.default_rng(0)
            ax.plot(i + rng.uniform(-0.13, 0.13, len(v)), v, 'o', ms=3.2,
                    color=COLOR[p], alpha=0.5, mec=SURFACE, mew=0.6, zorder=4,
                    linestyle='none')
        # clear the per-seed cloud, not just the bar, or the label lands in it
        ax.text(i, max(v.mean(), v.max()) + top * 0.05, fmt.format(v.mean()),
                ha='center', va='bottom', fontsize=11, color=INK,
                fontweight='medium')
    if ref is not None:
        ax.axhline(ref, color=MUTED, lw=1.0, zorder=2)
        # sits just UNDER the rule and hard left, clear of the bar's own label
        ax.text(-0.52, ref - top * 0.045, ref_label, fontsize=7.5, color=MUTED,
                va='top', ha='left')
    ax.set_xticks([])                          # identity comes from the legend
    ax.set_title(title, fontsize=10, color=INK, pad=8)
    ax.set_xlabel(note, fontsize=7.8, color=MUTED, labelpad=6)
    ax.grid(axis='y', color=HAIR, lw=0.6, zorder=0)   # solid hairline, never dashed
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=8, colors=MUTED, length=2)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(HAIR)
        ax.spines[s].set_linewidth(0.8)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default='data/marl/summary.csv')
    ap.add_argument('--out', default='data/marl/fig2_coalition')
    args = ap.parse_args()

    num = ('items_done', 'n_items', 'block_done', 'energy', 'wasted', 'failed',
           'coalition_peak', 'rounds', 'j_per_item', 'arms_preferring_block')
    with open(args.csv) as f:
        rows = [{k: (float(v) if k in num else v) for k, v in r.items()}
                for r in csv.DictReader(f)]
    n_seed = len({r['seed'] for r in rows})
    by = defaultdict(list)
    for r in rows:
        r['wasted_pct'] = 100.0 * r['wasted'] / max(r['energy'], 1e-9)
        by[r['policy']].append(r)
    n_items = int(rows[0]['n_items'])

    fig = plt.figure(figsize=(11.0, 5.0))
    gs = fig.add_gridspec(2, 3, height_ratios=[0.40, 1.0], hspace=0.30,
                          wspace=0.26, left=0.065, right=0.975, top=0.755,
                          bottom=0.13)

    # ---- hero band: the binary headline is a number, not a two-bar chart -----
    hero = fig.add_subplot(gs[0, :])
    hero.axis('off')
    for i, p in enumerate(ORDER):
        d = np.array([r['block_done'] for r in by[p]])
        x = 0.005 + i * 0.5
        hero.text(x, 0.5, f'{int(d.sum())} of {len(d)}', fontsize=31,
                  color=COLOR[p], ha='left', va='center')
        # offset sized for the WIDEST number the band can show ("30 of 30"), so
        # the caption does not slide under the figure as the seed count grows
        hero.text(x + 0.225, 0.5,
                  f'episodes where\n{p} delivered\nthe 1.8 kg 4-arm block',
                  fontsize=9, color=MUTED, ha='left', va='center', linespacing=1.45)
    hero.set_xlim(0, 1)
    hero.set_ylim(0, 1)

    # ---- small multiples ----------------------------------------------------
    panel(fig.add_subplot(gs[1, 0]), rows, 'coalition_peak',
          'Arms acting on the block at once',
          'a 1.8 kg load needs 4 x 0.5 kg payload', fmt='{:.1f}', ymax=5.4,
          ref=4.0, ref_label='4 required')
    panel(fig.add_subplot(gs[1, 1]), rows, 'j_per_item',
          'Energy per delivered item',
          'total J divided by items actually delivered', fmt='{:.1f}')
    panel(fig.add_subplot(gs[1, 2]), rows, 'wasted_pct',
          'Energy spent on failed lifts',
          'travel paid for, nothing transported', fmt='{:.0f}%')

    fig.legend(handles=[Patch(facecolor=COLOR[p], edgecolor='none', label=p)
                        for p in ORDER],
               loc='lower center', ncol=2, frameon=False, fontsize=9.5,
               bbox_to_anchor=(0.5, 0.005), labelcolor=INK)

    pref = np.mean([r['arms_preferring_block'] for r in rows])
    fig.suptitle('Energy-greedy allocation cannot complete a 4-arm '
                 'co-manipulation', fontsize=13.5, color=INK, x=0.07, ha='left',
                 y=0.955)
    fig.text(0.065, 0.905,
             f'{n_seed} episodes, {n_items} items each (5 single-arm + one 1.8 kg '
             f'block). Both policies get the same action space and the same energy J '
             f'— only one may bind four arms to one item.\nOn average '
             f'{pref:.1f} of 4 arms rank a block handle as their OWN lowest-J '
             f'action, so greedy is drawn to the block and still never lifts it.',
             fontsize=8.6, color=MUTED, ha='left', va='top', linespacing=1.6)

    for ext in ('png', 'pdf'):
        path = f'{args.out}.{ext}'
        fig.savefig(path, dpi=300, facecolor=SURFACE, bbox_inches='tight')
        print('wrote', path)


if __name__ == '__main__':
    main()
