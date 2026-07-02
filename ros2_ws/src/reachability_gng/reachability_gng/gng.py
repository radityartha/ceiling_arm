"""Growing Neural Gas (Fritzke, 1995) with a weighted distance metric.

The key adaptation for this project (borrowed from the sensei's GCS action-map):
each sample/node vector is the concatenation ``[task | q]`` where

    task = end-effector pose features (e.g. xyz + orientation), dim = task_dim
    q    = the 8-DOF joint configuration that produced it,       dim = q_dim

Best-matching-unit (BMU) search uses ONLY the task sub-vector (via
``dist_weights`` zeroing the q dims), so nodes organise to tile the reachable
workspace. Adaptation, however, moves the FULL vector toward the sample, so each
node's q part converges to a representative configuration for its workspace cell.
Querying the map for a target pose then returns that q as a MoveIt IK seed.

This module has no ROS dependencies and is unit-testable on its own.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class GNGParams:
    max_nodes: int = 2000      # upper bound on graph size
    lam: int = 200             # insert a node every `lam` samples
    eps_b: float = 0.05        # BMU adaptation rate
    eps_n: float = 0.006       # neighbour adaptation rate
    age_max: int = 100         # prune edges older than this
    alpha: float = 0.5         # error decay of parents on insertion
    beta: float = 0.0005       # global error decay per step
    seed: int = 0


@dataclass
class GNG:
    """Growing Neural Gas over vectors x = [task | q].

    Parameters
    ----------
    dim : total vector dimension (task_dim + q_dim)
    task_dim : number of leading dims used for the BMU distance metric
    params : GNGParams
    """

    dim: int
    task_dim: int
    params: GNGParams = field(default_factory=GNGParams)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.params.seed)
        # node reference vectors, growable; start empty (seeded in fit/init)
        self.W = np.empty((0, self.dim), dtype=np.float64)
        self.error = np.empty((0,), dtype=np.float64)
        # pinned nodes are frozen boundary seeds: never moved by adaptation and
        # never removed, so the node hull stays anchored on the true reachable
        # surface instead of shrinking to Voronoi centroids (see seed_boundary).
        self.pinned = np.empty((0,), dtype=bool)
        # edge ages keyed by frozenset({i, j}); _adj is an incremental
        # neighbour index (node -> set of neighbours) kept in sync with _edges
        # so the hot path is O(degree), not O(num_edges).
        self._edges = {}
        self._adj = {}
        self._step = 0
        # distance weight mask: 1 over task dims, 0 over q dims
        self._w = np.zeros(self.dim)
        self._w[: self.task_dim] = 1.0

    # ---- internal helpers ---------------------------------------------------
    def _dist2(self, x):
        """Weighted squared distance from x to every node (task dims only)."""
        d = (self.W - x) * self._w
        return np.einsum('ij,ij->i', d, d)

    def _neighbours(self, i):
        return list(self._adj.get(i, ()))

    def _add_node(self, vec, pinned=False):
        self.W = np.vstack([self.W, vec[None, :]])
        self.error = np.append(self.error, 0.0)
        self.pinned = np.append(self.pinned, bool(pinned))
        idx = len(self.W) - 1
        self._adj[idx] = set()
        return idx

    def _set_edge(self, i, j, age=0):
        self._edges[frozenset((i, j))] = age
        self._adj[i].add(j)
        self._adj[j].add(i)

    def _del_edge(self, i, j):
        self._edges.pop(frozenset((i, j)), None)
        self._adj[i].discard(j)
        self._adj[j].discard(i)

    def _remove_node(self, idx):
        keep = [k for k in range(len(self.W)) if k != idx]
        remap = {old: new for new, old in enumerate(keep)}
        self.W = self.W[keep]
        self.error = self.error[keep]
        self.pinned = self.pinned[keep]
        new_edges = {}
        for e, age in self._edges.items():
            a, b = tuple(e)
            if a == idx or b == idx:
                continue
            new_edges[frozenset((remap[a], remap[b]))] = age
        self._edges = new_edges
        new_adj = {remap[k]: set() for k in keep}
        for e in self._edges:
            a, b = tuple(e)
            new_adj[a].add(b)
            new_adj[b].add(a)
        self._adj = new_adj

    # ---- public API ---------------------------------------------------------
    def init_two(self, X):
        """Seed the graph with two random samples (call before stepping)."""
        idx = self._rng.choice(len(X), size=2, replace=False)
        self._add_node(X[idx[0]].copy())
        self._add_node(X[idx[1]].copy())
        self._set_edge(0, 1, 0)

    def seed_boundary(self, W_seed, edges):
        """Seed the graph with fixed boundary nodes before fitting.

        `W_seed` (M, dim) are full [task | q] vectors sampled on the reachable
        workspace surface; `edges` is an iterable of (i, j) index pairs forming
        the boundary shell. All seeded nodes are pinned (frozen + never removed),
        so the interior GNG grows inside a hull anchored on the true surface.
        Call instead of init_two, before fit()."""
        for vec in np.asarray(W_seed, dtype=np.float64):
            self._add_node(vec.copy(), pinned=True)
        for i, j in edges:
            if i != j:
                self._set_edge(int(i), int(j), 0)
        return self

    def step(self, x):
        """Present a single sample and update the graph."""
        self._step += 1
        d2 = self._dist2(x)
        order = np.argsort(d2)
        s1, s2 = int(order[0]), int(order[1])

        # accumulate error of the BMU (squared task-space distance)
        self.error[s1] += d2[s1]

        # move BMU and its topological neighbours toward x (full vector).
        # Pinned (boundary) nodes are frozen: they stay on the true reachable
        # surface and their q remains the config that reached that surface point.
        if not self.pinned[s1]:
            self.W[s1] += self.params.eps_b * (x - self.W[s1])
        for n in self._neighbours(s1):
            if not self.pinned[n]:
                self.W[n] += self.params.eps_n * (x - self.W[n])

        # An edge's age only grows when one endpoint is the BMU, so an edge can
        # only become stale right after being aged here -> we age + prune just
        # the BMU's incident edges (O(degree)). Edges touching a pinned node are
        # never aged/pruned, so the boundary shell (and its anchors to the
        # interior) persists for the whole run.
        touched = set()
        for n in list(self._adj[s1]):
            if self.pinned[s1] or self.pinned[n]:
                continue
            e = frozenset((s1, n))
            self._edges[e] += 1
            if self._edges[e] > self.params.age_max:
                self._del_edge(s1, n)
                touched.add(n)
        self._set_edge(s1, s2, 0)  # refresh BMU<->2nd-BMU edge (age 0)

        # drop non-pinned nodes that just lost their last edge
        for i in sorted((n for n in touched
                         if not self._adj[n] and not self.pinned[n]),
                        reverse=True):
            if len(self.W) > 2:
                self._remove_node(i)

        # periodically insert a node between the worst pair
        if self._step % self.params.lam == 0 and len(self.W) < self.params.max_nodes:
            self._insert()

        # global error decay
        self.error *= (1.0 - self.params.beta)

    def _insert(self):
        q = int(np.argmax(self.error))
        neigh = self._neighbours(q)
        if not neigh:
            return
        f = max(neigh, key=lambda n: self.error[n])
        r = self._add_node((self.W[q] + self.W[f]) * 0.5)
        # rewire q-f through r
        self._del_edge(q, f)
        self._set_edge(q, r, 0)
        self._set_edge(f, r, 0)
        self.error[q] *= self.params.alpha
        self.error[f] *= self.params.alpha
        self.error[r] = self.error[q]

    def fit(self, X, epochs=1):
        """Train on dataset X (rows = [task | q]) for a number of epochs."""
        X = np.asarray(X, dtype=np.float64)
        if len(self.W) == 0:
            self.init_two(X)
        for _ in range(epochs):
            for i in self._rng.permutation(len(X)):
                self.step(X[i])
        return self

    def query(self, task_vec, k=1):
        """Return indices of the k nearest nodes to a task-space query."""
        x = np.zeros(self.dim)
        x[: self.task_dim] = task_vec
        d2 = self._dist2(x)
        return np.argsort(d2)[:k]

    def query_radius(self, task_vec, radius, max_k=None):
        """Indices of all nodes within `radius` (task-space), nearest-first.

        Unlike `query` (fixed k by distance), this returns the whole reachable
        POOL around a target so a caller can re-rank it by a SECONDARY cost
        (e.g. energy) instead of by task distance alone -- a slightly-farther
        node may still reach the object (via IK to the exact pose) yet cost less
        to move to. Always returns at least the single nearest node, so a caller
        never gets an empty pool. `max_k` caps the pool size."""
        x = np.zeros(self.dim)
        x[: self.task_dim] = task_vec
        d2 = self._dist2(x)
        within = np.where(d2 <= radius * radius)[0]
        if len(within) == 0:
            within = np.array([int(np.argmin(d2))])
        within = within[np.argsort(d2[within])]
        if max_k is not None:
            within = within[:max_k]
        return within

    def seed_q(self, task_vec):
        """Return the joint configuration (q part) of the nearest node."""
        i = int(self.query(task_vec, k=1)[0])
        return self.W[i, self.task_dim:].copy()

    # ---- persistence --------------------------------------------------------
    def save(self, path):
        edges = np.array([list(e) for e in self._edges], dtype=np.int64) \
            if self._edges else np.empty((0, 2), dtype=np.int64)
        ages = np.array(list(self._edges.values()), dtype=np.int64)
        np.savez(
            path, W=self.W, error=self.error, edges=edges, ages=ages,
            dim=self.dim, task_dim=self.task_dim, pinned=self.pinned,
        )

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=False)
        g = cls(dim=int(d['dim']), task_dim=int(d['task_dim']))
        g.W = d['W']
        g.error = d['error']
        g.pinned = (d['pinned'] if 'pinned' in d
                    else np.zeros(len(g.W), dtype=bool))
        g._edges = {frozenset(map(int, e)): int(a)
                    for e, a in zip(d['edges'], d['ages'])}
        g._adj = {i: set() for i in range(len(g.W))}
        for e in g._edges:
            a, b = tuple(e)
            g._adj[a].add(b)
            g._adj[b].add(a)
        return g
