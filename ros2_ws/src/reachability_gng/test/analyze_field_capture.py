#!/usr/bin/env python3
"""G6 field analysis: determinism + quality of the static map on REAL clouds.

Runs the criteria locked in docs/p1_g6_map.md §A3 over the two point clouds
saved by map_topo_static (SAVE_CLOUD=...), at PRODUCTION settings. Everything
measured here is defined elsewhere and reused verbatim -- no metric is invented
in this file:

  metrics + fits   test/bench_topo_determinism.py  (map_distance, node_set_equal,
                                                    quality, n_components, fit)
  fit code path    map_topo_static.fit_static_map{,_bl}  (what the node ships)

  G1  capture validity          n points, bbox span
  G3  sensor noise              cloud_a <-> cloud_b   (measured FIRST, so that
                                                       map drift is attributable)
  G2  D2 field   *** GATE ***   cloud_a rows permuted -> bit-identical map?
  G4  D3 field   (no threshold) D3-recapture: map_a <-> map_b
                                D3-95%:       map_a <-> map(95% of cloud_a)
  G5  quality parity            QE / coverage / spacing / components on cloud_a

  python3 analyze_field_capture.py --algo bl  --json /tmp/g6_field.jsonl
  python3 analyze_field_capture.py --algo gng --json /tmp/g6_field.jsonl

One process per algo (they are independent and MS-BL is ~40x slower).
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from bench_topo_determinism import (fit, map_distance, node_set_equal, quality)

CLOUD_CHUNK = 200        # 12k x 12k pairwise -> keep the block under ~60 MB


def load_cloud(path):
    z = np.load(path)
    return np.asarray(z['cloud'], dtype=np.float64), int(z['captured'])


def bbox_span(c):
    return (c.max(axis=0) - c.min(axis=0)).tolist()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--algo', choices=['gng', 'bl'], required=True)
    ap.add_argument('--cloud-a', default='/tmp/topo_cloud_a.npz')
    ap.add_argument('--cloud-b', default='/tmp/topo_cloud_b.npz')
    # production settings -- must match map_topo_static's declared defaults
    ap.add_argument('--max-nodes', type=int, default=1800)
    ap.add_argument('--lam', type=int, default=100)
    ap.add_argument('--epochs', type=int, default=0)
    ap.add_argument('--fit-max-points', type=int, default=12000)
    ap.add_argument('--json', help='append one JSON result line to this file')
    args = ap.parse_args()

    ca, cap_a = load_cloud(args.cloud_a)
    cb, cap_b = load_cloud(args.cloud_b)
    res = {'algo': args.algo, 'max_nodes': args.max_nodes, 'lam': args.lam,
           'cloud_a': args.cloud_a, 'cloud_b': args.cloud_b}

    # --- G1 capture validity -------------------------------------------------
    res['G1'] = {
        'n_a': int(len(ca)), 'n_b': int(len(cb)),
        'captured_a': cap_a, 'captured_b': cap_b,
        'bbox_a_m': bbox_span(ca), 'bbox_b_m': bbox_span(cb),
        'n_ratio_b_over_a': round(len(cb) / len(ca), 4),
    }
    ok_a = len(ca) >= 3000 and bbox_span(ca)[0] >= 1.0 and bbox_span(ca)[1] >= 1.0
    ok_b = len(cb) >= 3000 and bbox_span(cb)[0] >= 1.0 and bbox_span(cb)[1] >= 1.0
    res['G1']['valid_a'], res['G1']['valid_b'] = bool(ok_a), bool(ok_b)
    if not (ok_a and ok_b):
        res['G1']['ABORT'] = 'capture invalid per docs/p1_g6_map.md §A3/G1'
        print(json.dumps(res, indent=2))
        return 1

    # --- G3 sensor noise, BEFORE any map comparison ---------------------------
    t0 = time.perf_counter()
    res['G3_sensor_cloud_ab'] = map_distance(ca, cb, chunk=CLOUD_CHUNK)
    res['G3_sensor_seconds'] = round(time.perf_counter() - t0, 1)

    def run(pool):
        t = time.perf_counter()
        g, used, ep = fit(args.algo, pool, args.max_nodes, args.lam,
                          args.epochs, args.fit_max_points)
        return g, used, ep, round(time.perf_counter() - t, 1)

    # --- G2 D2 field: the gate ------------------------------------------------
    g_a, used_a, ep, t_a = run(ca)
    res['epochs'] = ep
    res['fit_seconds'] = t_a
    perm = np.random.default_rng(7).permutation(len(ca))
    g_p, _, _, t_p = run(ca[perm])
    ok2, delta2 = node_set_equal(g_a.W, g_p.W)
    res['G2_D2'] = {'bit_identical': bool(ok2), 'max_abs_delta_m': delta2,
                    'drift': map_distance(g_a.W, g_p.W), 'fit_seconds': t_p}

    # --- G4 D3 field ----------------------------------------------------------
    g_b, _, _, t_b = run(cb)
    keep = np.random.default_rng(7).choice(len(ca), int(0.95 * len(ca)),
                                           replace=False)
    g_s, _, _, t_s = run(ca[keep])
    res['G4_D3_recapture'] = dict(map_distance(g_a.W, g_b.W), fit_seconds=t_b)
    res['G4_D3_95pct'] = dict(map_distance(g_a.W, g_s.W), fit_seconds=t_s)
    # attribution ratio (§A3/G3): map drift relative to the sensor's own drift
    res['G4_ratio_map_over_sensor_mean_nn'] = round(
        res['G4_D3_recapture']['mean_nn_m']
        / res['G3_sensor_cloud_ab']['mean_nn_m'], 3)

    # --- G5 quality parity on the same real cloud -----------------------------
    res['G5_quality_on_cloud_a'] = quality(g_a, used_a)

    print(json.dumps(res, indent=2))
    if args.json:
        with open(args.json, 'a') as f:
            f.write(json.dumps(res) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
