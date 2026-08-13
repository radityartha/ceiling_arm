#!/usr/bin/env python3
"""Validate MS-BL-GNG against synthetic structure whose answer is known first.

Same discipline as test/validate_reach_dwell_monitor.py: the instrument is
checked against ground truth it cannot influence, BEFORE it is trusted on real
data. The four scenes and what each must show are fixed in
docs/p1_g5_msbl_gcs.md §A2, written before this ran.

  S1  two blobs 1.0 m apart   -> exactly 2 connected components
  S2  1x1 m plane             -> QE falls monotonically as nodes are added
  S3  sphere shell r=0.5      -> every node sits on the shell
  S4  uniform cube noise      -> no collapse; nodes spread over the cube

Runs the SAME checks against plain GNG so any difference is visible rather than
asserted. Exit code 0 = every check passed.

  python3 test/validate_bl_gng.py
"""
from __future__ import annotations

import sys

import numpy as np

_HERE = __import__('pathlib').Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))
# the test dir itself, NOT as a `test.` package: that name is taken by the
# stdlib and shadowing it breaks the import
sys.path.insert(0, str(_HERE.parent))

from bench_topo_determinism import (n_components, pool_s1,      # noqa: E402
                                    pool_s2, pool_s3, pool_s4)
from reachability_gng.bl_gng import BLGNG, BLGNGParams          # noqa: E402
from reachability_gng.gng import GNG, GNGParams                 # noqa: E402

FAILS = []


def check(name, ok, detail):
    mark = 'PASS' if ok else 'FAIL'
    print(f'  [{mark}] {name}: {detail}')
    if not ok:
        FAILS.append(name)


def build(algo, X, max_nodes):
    if algo == 'bl':
        g = BLGNG(dim=3, task_dim=3, params=BLGNGParams(max_nodes=max_nodes))
        return g.fit(X)
    g = GNG(dim=3, task_dim=3, params=GNGParams(max_nodes=max_nodes, lam=100))
    return g.fit(X, epochs=max(3, -(-int(1.5 * max_nodes * 100) // len(X))))


def s1_components(algo):
    """Two blobs 1.0 m apart with 0.12 m spread: the gap is ~8 sigma, so any
    edge bridging it is a false connection, not a resolution limit."""
    g = build(algo, pool_s1(), 200)
    comp = n_components(len(g.W), [tuple(e) for e in g._edges])
    # a node stranded in the empty gap is the failure this scene is for
    gap = int(((g.W[:, 0] > 0.3) & (g.W[:, 0] < 0.7)).sum())
    check(f'{algo} S1 two components', comp == 2, f'{comp} components')
    check(f'{algo} S1 no nodes in the gap', gap == 0, f'{gap} nodes in 0.3<x<0.7')


def s2_monotone_qe(algo):
    """More nodes must tile the plane more finely -- if QE stops falling, the
    net has stopped using the nodes it is given."""
    X = pool_s2()
    qes = []
    for n in (50, 100, 200, 400):
        g = build(algo, X, n)
        d = np.linalg.norm(X[:, None, :] - g.W[None, :, :], axis=2).min(axis=1)
        qes.append(float(d.mean()))
    mono = all(b < a for a, b in zip(qes, qes[1:]))
    check(f'{algo} S2 QE monotone in node count', mono,
          ' -> '.join(f'{q:.4f}' for q in qes))


def s3_on_shell(algo, r=0.5):
    """Nodes must lie ON the shell. A net that averages across the sphere pulls
    nodes toward the centre, which shows up as radius, not as QE."""
    g = build(algo, pool_s3(r=r), 300)
    rad = np.linalg.norm(g.W, axis=1)
    check(f'{algo} S3 nodes on the shell',
          abs(rad.mean() - r) < 0.02 and rad.std() < 0.02,
          f'radius {rad.mean():.4f} +- {rad.std():.4f} (target {r})')


def s4_no_collapse(algo):
    """Structureless input: the net must still spread, not clump."""
    X = pool_s4()
    g = build(algo, X, 300)
    spread = g.W.max(0) - g.W.min(0)
    d = np.linalg.norm(X[:, None, :] - g.W[None, :, :], axis=2).min(axis=1)
    check(f'{algo} S4 no collapse', bool((spread > 0.8).all()),
          f'node bbox {np.round(spread, 3)}, QE {d.mean():.4f}')


def main():
    for algo in ('gng', 'bl'):
        print(f'\n=== {algo} ===')
        s1_components(algo)
        s2_monotone_qe(algo)
        s3_on_shell(algo)
        s4_no_collapse(algo)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s): {", ".join(FAILS)}')
        return 1
    print('all ground-truth checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
