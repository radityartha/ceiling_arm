#!/usr/bin/env python3
"""What does MS-BL's 1-node-per-batch growth cadence actually cost?

MS-BL is ~40x slower than online GNG at production settings (p1_g5 §B7b). That
is NOT a property of batch learning -- a batch pass is vectorised and is what
made env_gng cheaper online. It is a property of the GROWTH CADENCE: the source
(Meso-HSR/GNG.h, "add neuron per every it iteration") adds ONE node per batch,
so reaching max_nodes costs max_nodes passes over the data, while online GNG
inserts every lam samples WITHIN a pass.

`learn_batch(grow=k)` / `fit(grow=k)` adds k nodes per batch, so growth needs
max_nodes/k batches. This measures what that buys and what it costs, on the REAL
cloud, at production settings. Answers p1_next_steps Jalur B-3, which explicitly
forbids writing "can be sped up" before the numbers exist.

Reported without a threshold: this is a cost curve, not a gate. The shipping
path stays grow=1 unless a later session decides otherwise on these numbers.

  python3 bench_bl_grow.py --cloud /tmp/topo_cloud_a.npz --grows 4 8 16
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from bench_topo_determinism import node_set_equal, quality


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cloud', default='/tmp/topo_cloud_a.npz')
    ap.add_argument('--baseline-map', default='/tmp/topo_static_a.npz',
                    help='the grow=1 map already produced by the capture')
    ap.add_argument('--grows', type=int, nargs='+', default=[4, 8, 16])
    ap.add_argument('--max-nodes', type=int, default=1800)
    ap.add_argument('--json', default='/tmp/g6_grow.jsonl')
    args = ap.parse_args()

    from reachability_gng.bl_gng import BLGNG, BLGNGParams

    pool = np.asarray(np.load(args.cloud)['cloud'], dtype=np.float64)
    rows = []

    g1 = BLGNG.load(args.baseline_map)      # grow=1, already paid for
    rows.append({'grow': 1, 'seconds': None, 'reused': args.baseline_map,
                 **quality(g1, pool)})

    for k in args.grows:
        t0 = time.perf_counter()
        g = BLGNG(dim=3, task_dim=3,
                  params=BLGNGParams(max_nodes=args.max_nodes)).fit(pool, grow=k)
        dt = round(time.perf_counter() - t0, 1)
        # order-invariance must SURVIVE the knob, or the knob is not usable:
        # the whole point of MS-BL here is D2.
        gp = BLGNG(dim=3, task_dim=3,
                   params=BLGNGParams(max_nodes=args.max_nodes)).fit(
                       pool[np.random.default_rng(7).permutation(len(pool))],
                       grow=k)
        ok, delta = node_set_equal(g.W, gp.W)
        rows.append({'grow': k, 'seconds': dt, 'D2_bit_identical': bool(ok),
                     'D2_max_abs_delta_m': delta, **quality(g, pool)})
        print(json.dumps(rows[-1]), flush=True)

    out = {'cloud': args.cloud, 'max_nodes': args.max_nodes, 'rows': rows}
    print(json.dumps(out, indent=2))
    if args.json:
        with open(args.json, 'a') as f:
            f.write(json.dumps(out) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
