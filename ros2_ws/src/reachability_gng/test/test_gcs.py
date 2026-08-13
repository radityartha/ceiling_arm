"""Tests for the GCS action map (no ROS required: `pytest` from the pkg root).

The centrepiece is test_simplex_invariant_*: the same run, once with the repair
disabled and once enabled. The disabled run MUST record violations -- if it ever
stops doing so the test has stopped testing anything and should be deleted, not
relaxed. That pair is the evidence that the fix is real rather than claimed
(docs/p1_g5_msbl_gcs.md §A1/C4).
"""

import numpy as np

from reachability_gng.gcs import GCS, GCSParams


def _toy_dataset(n=4000, seed=0):
    """Same shape as test_gng's: task = (x, y) on the unit square, q = x + y."""
    rng = np.random.default_rng(seed)
    xy = rng.random((n, 2))
    q = (xy[:, 0] + xy[:, 1])[:, None]
    return np.hstack([xy, q])


def _run_collecting_violations(simplex_repair, n_adds=60, seed=1):
    """Fit while checking the invariant after every single add."""
    X = _toy_dataset()
    # add_every is pushed out of reach so step() NEVER adds on its own: every
    # add below is explicit, so its (g, k, h) is in hand and the violation count
    # is exact rather than a lower bound. Learning runs between adds so the
    # graph under test is a realistic one, not a bare simplex.
    g = GCS(dim=3, task_dim=2,
            params=GCSParams(max_nodes=n_adds + 3, add_every=10 ** 9, seed=seed,
                             simplex_repair=simplex_repair))
    g.init_simplex(X)
    rng = np.random.default_rng(seed)
    violations, adds = 0, 0
    for _ in range(n_adds):
        for i in rng.choice(len(X), size=50, replace=False):
            g.step(X[i])
        out = g.add_node()
        if out is None:
            continue
        adds += 1
        violations += len(g.simplex_violations(*out))
    return g, adds, violations


def test_simplex_invariant_violated_without_repair():
    """The sensei's implementation as-is: g does NOT inherit common neighbours.

    This is Meso-HSR/GNG.h:634-640 left commented out. If this assertion starts
    failing, the repair has leaked into the unrepaired path.
    """
    _, adds, violations = _run_collecting_violations(simplex_repair=False)
    assert adds > 0
    assert violations > 0, 'unrepaired GCS should violate the simplex invariant'


def test_simplex_invariant_holds_with_repair():
    """With the block restored, every add leaves the invariant intact."""
    _, adds, violations = _run_collecting_violations(simplex_repair=True)
    assert adds > 0
    assert violations == 0


def test_repair_only_adds_edges():
    """The repair must not delete or move anything -- it only closes simplices."""
    off, _, _ = _run_collecting_violations(simplex_repair=False, seed=4)
    on, _, _ = _run_collecting_violations(simplex_repair=True, seed=4)
    assert len(on._edges) > len(off._edges)


def test_grows_and_covers():
    X = _toy_dataset()
    g = GCS(dim=3, task_dim=2, params=GCSParams(max_nodes=200, seed=1))
    g.fit(X, epochs=2)
    assert 10 < len(g.W) <= 200
    assert len(g._edges) > 0


def test_seed_recovers_function():
    X = _toy_dataset()
    g = GCS(dim=3, task_dim=2, params=GCSParams(max_nodes=300, seed=2))
    g.fit(X, epochs=3)
    q = g.seed_q(np.array([0.25, 0.75]))
    assert abs(q[0] - 1.0) < 0.2


def test_save_load_roundtrip(tmp_path):
    X = _toy_dataset()
    g = GCS(dim=3, task_dim=2, params=GCSParams(max_nodes=100, seed=3))
    g.fit(X, epochs=1)
    p = str(tmp_path / 'gcs.npz')
    g.save(p)
    h = GCS.load(p)
    assert np.allclose(g.W, h.W)
    assert g._edges == h._edges
    assert h.task_dim == g.task_dim


def test_api_compatible_with_gng_consumers():
    """The ten action-map consumers only need this surface -- keep it working."""
    X = _toy_dataset()
    g = GCS(dim=3, task_dim=2, params=GCSParams(max_nodes=120, seed=5))
    g.fit(X, epochs=1)
    assert g.query(np.array([0.5, 0.5]), k=3).shape == (3,)
    pool = g.query_radius(np.array([0.5, 0.5]), 0.2, max_k=5)
    assert 1 <= len(pool) <= 5
    # query_radius always returns at least the nearest node, even out of range
    assert len(g.query_radius(np.array([99.0, 99.0]), 0.01)) == 1
    assert g.W.shape[1] == 3 and g.task_dim == 2
