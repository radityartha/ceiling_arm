"""Smoke tests for the GNG core (no ROS required: `pytest` from the pkg root)."""

import numpy as np

from reachability_gng.gng import GNG, GNGParams


def _toy_dataset(n=4000, seed=0):
    """2D task on a plane + a 'joint' channel that is a function of the task.

    task = (x, y) on [0,1]^2 ; q = (x + y,)  -> nearest-node q should recover
    the value at the queried location.
    """
    rng = np.random.default_rng(seed)
    xy = rng.random((n, 2))
    q = (xy[:, 0] + xy[:, 1])[:, None]
    return np.hstack([xy, q])


def test_grows_and_covers():
    X = _toy_dataset()
    g = GNG(dim=3, task_dim=2, params=GNGParams(max_nodes=200, lam=100, seed=1))
    g.fit(X, epochs=2)
    assert 10 < len(g.W) <= 200
    assert len(g._edges) > 0


def test_seed_recovers_function():
    X = _toy_dataset()
    g = GNG(dim=3, task_dim=2, params=GNGParams(max_nodes=300, lam=100, seed=2))
    g.fit(X, epochs=3)
    # at (0.25, 0.75) the true q = 1.0; nearest-node seed should be close
    q = g.seed_q(np.array([0.25, 0.75]))
    assert abs(q[0] - 1.0) < 0.2


def test_save_load_roundtrip(tmp_path):
    X = _toy_dataset()
    g = GNG(dim=3, task_dim=2, params=GNGParams(max_nodes=100, seed=3))
    g.fit(X, epochs=1)
    p = tmp_path / 'm.npz'
    g.save(str(p))
    h = GNG.load(str(p))
    assert h.W.shape == g.W.shape
    assert h.task_dim == g.task_dim
    np.testing.assert_allclose(h.W, g.W)
