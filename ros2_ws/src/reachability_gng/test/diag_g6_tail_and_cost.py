#!/usr/bin/env python3
"""Two G6 follow-up diagnostics, both measured rather than argued.

A) WHY does MS-BL win mean-NN but lose hausdorff on D3-recapture?
   hausdorff is a MAX over per-node nearest-neighbour distances, mean_nn is a
   MEAN over the same vector. So the two can disagree only through the shape of
   that distribution. Print its percentiles for both algorithms, then look at
   where the worst node actually sits (local data density around it).

B) WHY is MS-BL slower than online GNG, when a batch pass is vectorised and
   should be cheaper? Count SAMPLE PRESENTATIONS (how many point-vs-node
   comparisons each algorithm actually performs) and divide wall-clock by it.
   That separates "cost per sample" from "how many samples are presented".

  python3 diag_g6_tail_and_cost.py --part a
  python3 diag_g6_tail_and_cost.py --part b --max-nodes 400
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from bench_topo_determinism import _nn_dist, fit


def part_a(args):
    from reachability_gng.bl_gng import BLGNG
    ca = np.asarray(np.load(args.cloud_a)['cloud'], dtype=np.float64)
    cb = np.asarray(np.load(args.cloud_b)['cloud'], dtype=np.float64)

    out = {}
    for algo in ('gng', 'bl'):
        if algo == 'bl':                       # already on disk from the capture
            Wa, Wb = BLGNG.load(args.map_a).W, BLGNG.load(args.map_b).W
        else:
            ga, _, _ = fit('gng', ca, args.max_nodes, 100, 0, 12000)
            gb, _, _ = fit('gng', cb, args.max_nodes, 100, 0, 12000)
            Wa, Wb = ga.W, gb.W
        d = np.concatenate([_nn_dist(Wa, Wb), _nn_dist(Wb, Wa)])
        pct = {f'p{p}': float(np.percentile(d, p))
               for p in (50, 90, 99, 99.9)}
        # where is the worst node, and how much DATA is near it? A node in a
        # sparse region has little evidence pinning it down.
        da = _nn_dist(Wa, Wb)
        worst = int(np.argmax(da))
        near = _nn_dist(Wa[worst][None, :], ca)[0]
        dens = int((np.linalg.norm(ca - Wa[worst], axis=1) < 0.10).sum())
        out[algo] = {
            'mean_nn_m': float(d.mean()), 'hausdorff_m': float(d.max()),
            **pct,
            'frac_over_10cm': float((d > 0.10).mean()),
            'frac_over_20cm': float((d > 0.20).mean()),
            'worst_node_xyz': Wa[worst].tolist(),
            'worst_node_dist_to_nearest_data_m': float(near),
            'data_points_within_10cm_of_worst': dens,
        }
    print(json.dumps(out, indent=2))
    return out


def part_b(args):
    """Count sample presentations, so cost/sample is separable from #samples."""
    import reachability_gng.bl_gng as blm
    ca = np.asarray(np.load(args.cloud_a)['cloud'], dtype=np.float64)
    res = {}

    # --- MS-BL: wrap learn_batch to sum len(Xb) ---------------------------
    orig = blm.BLGNG.learn_batch
    counter = {'pres': 0, 'batches': 0}

    def counting(self, Xb, grow=1):
        counter['pres'] += len(Xb)
        counter['batches'] += 1
        return orig(self, Xb, grow=grow)

    blm.BLGNG.learn_batch = counting
    try:
        t0 = time.perf_counter()
        g, _, _ = fit('bl', ca, args.max_nodes, 100, 0, 12000)
        dt = time.perf_counter() - t0
    finally:
        blm.BLGNG.learn_batch = orig
    res['bl'] = {'seconds': round(dt, 1), 'presentations': counter['pres'],
                 'batches': counter['batches'], 'n_nodes': int(len(g.W)),
                 'us_per_presentation': round(1e6 * dt / counter['pres'], 3)}

    # --- GNG: presentations = epochs * points (one winner search each) -----
    t0 = time.perf_counter()
    g2, used, ep = fit('gng', ca, args.max_nodes, 100, 0, 12000)
    dt2 = time.perf_counter() - t0
    pres2 = ep * len(used)
    res['gng'] = {'seconds': round(dt2, 1), 'presentations': int(pres2),
                  'epochs': ep, 'n_nodes': int(len(g2.W)),
                  'us_per_presentation': round(1e6 * dt2 / pres2, 3)}

    res['presentation_ratio_bl_over_gng'] = round(
        res['bl']['presentations'] / res['gng']['presentations'], 1)
    res['cost_per_presentation_ratio_bl_over_gng'] = round(
        res['bl']['us_per_presentation'] / res['gng']['us_per_presentation'], 3)
    res['wallclock_ratio_bl_over_gng'] = round(dt / dt2, 1)
    print(json.dumps(res, indent=2))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--part', choices=['a', 'b'], required=True)
    ap.add_argument('--cloud-a', default='/tmp/topo_cloud_a.npz')
    ap.add_argument('--cloud-b', default='/tmp/topo_cloud_b.npz')
    ap.add_argument('--map-a', default='/tmp/topo_static_a.npz')
    ap.add_argument('--map-b', default='/tmp/topo_static_b.npz')
    ap.add_argument('--max-nodes', type=int, default=1800)
    args = ap.parse_args()
    (part_a if args.part == 'a' else part_b)(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
