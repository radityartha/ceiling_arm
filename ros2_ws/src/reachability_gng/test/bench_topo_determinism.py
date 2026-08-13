#!/usr/bin/env python3
"""Determinism + map-quality benchmark for the static topological map.

Measures the criteria locked in docs/p1_g5_msbl_gcs.md §A1 BEFORE and AFTER the
MS-BL-GNG port, over the same pools, with the same code path the node ships
(map_topo_static.fit_static_map).

  C1/D1  same pool, same row order, fit twice     -> bit-identical?
  C1/D2  same pool, rows PERMUTED, fit twice      -> bit-identical? (the decisive one)
  C1/D3  pool resampled to 95%, fit twice         -> hausdorff / mean-NN drift
  C2     QE / coverage / spacing / components     -> quality parity

No hardware, no cameras, no ROS graph. The "real" pool is a PROXY: points
scattered around the nodes of a static map captured from a real scene on
2026-08-02 (/tmp/topo_static.npz). It is a distribution proxy, NOT a recapture.

  python3 bench_topo_determinism.py --algo gng    --scene proxy --max-nodes 400
  python3 bench_topo_determinism.py --algo bl     --scene proxy --max-nodes 400
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np


# ---------------------------------------------------------------- test pools
def pool_proxy(path='/tmp/topo_static.npz', per_node=8, jitter=0.02, seed=1):
    """Scatter points around a real-scene static map's nodes (distribution proxy)."""
    W = np.load(path)['W']
    rng = np.random.default_rng(seed)
    pts = np.repeat(W, per_node, axis=0)
    return pts + rng.normal(0.0, jitter, pts.shape)


def pool_s1(n=6000, seed=1):
    """S1: two Gaussian blobs 1.0 m apart -> must stay 2 connected components."""
    rng = np.random.default_rng(seed)
    a = rng.normal([0, 0, 0], 0.12, (n // 2, 3))
    b = rng.normal([1.0, 0, 0], 0.12, (n - n // 2, 3))
    return np.vstack([a, b])


def pool_s2(n=6000, seed=1):
    """S2: 1x1 m plane at z=0 -> 2-manifold, QE falls monotonically with nodes."""
    rng = np.random.default_rng(seed)
    xy = rng.random((n, 2))
    return np.column_stack([xy, np.zeros(n)])


def pool_s3(n=6000, r=0.5, seed=1):
    """S3: sphere shell r=0.5 -> every node radius must be ~r."""
    rng = np.random.default_rng(seed)
    v = rng.normal(0, 1, (n, 3))
    return r * v / np.linalg.norm(v, axis=1, keepdims=True)


def pool_s4(n=6000, seed=1):
    """S4: uniform noise in a cube -> structureless; must not collapse."""
    return np.random.default_rng(seed).random((n, 3))


POOLS = {'proxy': pool_proxy, 's1': pool_s1, 's2': pool_s2,
         's3': pool_s3, 's4': pool_s4}


# ------------------------------------------------------------------- metrics
def _nn_dist(a, b, chunk=2000):
    """min distance from every row of a to any row of b (chunked, no SciPy)."""
    out = np.empty(len(a))
    for i in range(0, len(a), chunk):
        d = a[i:i + chunk, None, :] - b[None, :, :]
        out[i:i + chunk] = np.sqrt(np.einsum('ijk,ijk->ij', d, d)).min(axis=1)
    return out


def node_set_equal(Wa, Wb):
    """Bit-identical as a SET: same count and identical after canonical sort."""
    if Wa.shape != Wb.shape:
        return False, float('inf')
    sa = Wa[np.lexsort(Wa.T[::-1])]
    sb = Wb[np.lexsort(Wb.T[::-1])]
    delta = float(np.max(np.abs(sa - sb))) if len(sa) else 0.0
    return delta == 0.0, delta


def map_distance(Wa, Wb):
    da, db = _nn_dist(Wa, Wb), _nn_dist(Wb, Wa)
    return {'hausdorff_m': float(max(da.max(), db.max())),
            'mean_nn_m': float((da.mean() + db.mean()) / 2)}


def n_components(n_nodes, edges):
    parent = list(range(n_nodes))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in edges:
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[ra] = rb
    return len({find(i) for i in range(n_nodes)})


def quality(g, pool):
    W = np.asarray(g.W)
    d = _nn_dist(pool, W)
    # node -> nearest OTHER node
    dd = W[:, None, :] - W[None, :, :]
    m = np.sqrt(np.einsum('ijk,ijk->ij', dd, dd))
    np.fill_diagonal(m, np.inf)
    spacing = m.min(axis=1)
    edges = [tuple(e) for e in g._edges]
    return {'QE_mean_m': float(d.mean()), 'QE_median_m': float(np.median(d)),
            'cov_2cm': float((d <= 0.02).mean()), 'cov_5cm': float((d <= 0.05).mean()),
            'n_nodes': int(len(W)), 'spacing_median_m': float(np.median(spacing)),
            'n_edges': int(len(edges)),
            'n_comp': n_components(len(W), edges)}


# ---------------------------------------------------------------------- fits
def fit(algo, pool, max_nodes, lam, epochs, fit_max_points):
    if algo == 'gng':
        from reachability_gng.map_topo_static import fit_static_map
        g, used, ep = fit_static_map(pool, max_nodes, lam, epochs, fit_max_points)
    elif algo == 'bl':
        from reachability_gng.map_topo_static import fit_static_map_bl
        g, used, ep = fit_static_map_bl(pool, max_nodes, lam, epochs,
                                        fit_max_points)
    else:
        raise SystemExit(f'unknown algo {algo}')
    return g, used, ep


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--algo', choices=['gng', 'bl'], required=True)
    ap.add_argument('--scene', choices=list(POOLS), default='proxy')
    ap.add_argument('--max-nodes', type=int, default=400)
    ap.add_argument('--lam', type=int, default=100)
    ap.add_argument('--epochs', type=int, default=0)
    ap.add_argument('--fit-max-points', type=int, default=6000)
    ap.add_argument('--json', help='append one JSON result line to this file')
    args = ap.parse_args()

    pool = POOLS[args.scene]()
    rng = np.random.default_rng(7)
    res = {'algo': args.algo, 'scene': args.scene, 'max_nodes': args.max_nodes,
           'lam': args.lam, 'pool_points': int(len(pool))}

    def run(p):
        t0 = time.perf_counter()
        g, used, ep = fit(args.algo, p, args.max_nodes, args.lam, args.epochs,
                          args.fit_max_points)
        return g, used, ep, time.perf_counter() - t0

    # --- D1: identical pool, identical order
    g1, used1, ep, t1 = run(pool)
    g2, _, _, _ = run(pool)
    ok, delta = node_set_equal(g1.W, g2.W)
    res['epochs'] = ep
    res['fit_seconds'] = round(t1, 1)
    res['D1_bit_identical'] = bool(ok)
    res['D1_max_abs_delta_m'] = delta
    res.update(quality(g1, used1))

    # --- D2: same points, PERMUTED rows (the decisive criterion)
    perm = rng.permutation(len(pool))
    g3, used3, _, _ = run(pool[perm])
    ok2, delta2 = node_set_equal(g1.W, g3.W)
    res['D2_bit_identical'] = bool(ok2)
    res['D2_max_abs_delta_m'] = delta2
    res['D2_drift'] = map_distance(g1.W, g3.W)

    # --- D3: 95% resample
    keep = rng.choice(len(pool), int(0.95 * len(pool)), replace=False)
    g4, _, _, _ = run(pool[keep])
    res['D3_drift'] = map_distance(g1.W, g4.W)

    print(json.dumps(res, indent=2))
    if args.json:
        with open(args.json, 'a') as f:
            f.write(json.dumps(res) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
