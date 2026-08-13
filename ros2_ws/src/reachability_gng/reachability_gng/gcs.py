"""Growing Cell Structures (Fritzke) for the action map ``xyz -> q``.

Port of the sensei's ``GCS_init`` / ``GCS_add`` / ``GCS_learning``
(``FRD-01/nRobot.h:435,499,538,593``). NOT a contribution -- cited:
Fritzke, "Growing Cell Structures -- a self-organizing network for unsupervised
and supervised learning".

Source selection was VERIFIED, not assumed: the repo holds four copies of
``GCS_add`` (``FRD-01`` :499, ``Artha-HSR-01/ori`` :704,
``Artha-HSR-01/Artha-HSR-01`` :704, ``Meso-HSR/nRobot.h`` :675) and they are
functionally identical -- the only difference is a commented-out ``printf``. So
following the plan's ``FRD-01`` citation costs nothing.

THE SIMPLEX REPAIR
------------------
The source does NOT maintain the simplex invariant. ``GCS_add`` inserts ``g``
between ``h`` and ``k``, wires ``g-h`` and ``g-k``, deletes ``h-k`` -- and stops.
It never connects ``g`` to the neighbours ``h`` and ``k`` have in COMMON, so the
k-simplices the structure is named after decay into an ordinary graph. The block
that would do it exists but is commented out at ``Meso-HSR/GNG.h:634-640``.

Without it the net is not a GCS, and the manuscript says GCS. Restoring it is
textbook Fritzke, so it is NOT a contribution either -- but it is required for
the algorithm name in the paper to match the algorithm that runs. It is a
constructor flag (``simplex_repair``) rather than a hardcoded fix so the
unrepaired behaviour stays reachable and the invariant test can demonstrate the
difference instead of asserting it (test/test_gcs.py).

API-compatible with ``GNG``: ``load`` (classmethod), ``query(task_vec, k)``,
``query_radius(task_vec, radius, max_k)``, ``.W``, ``.task_dim``, ``seed_q``,
``save``, ``remove_nodes`` -- so the ten files that hold an action map need no
change beyond choosing which class to load.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class GCSParams:
    max_nodes: int = 2000
    rlearn1: float = 0.05      # BMU pull
    rlearn2: float = 0.006     # neighbour pull (distance-weighted)
    rgauss: float = 1.0        # width of the distance-based neighbour weight
    rdis: float = 0.995        # per-step temporal discount of the error signal
    add_every: int = 100       # GCS_learning: split every n-th sample (t%100)
    add_threshold: float = 0.0  # dataMin: only split if worst error exceeds this
    simplex_repair: bool = True  # the Meso-HSR/GNG.h:634-640 block, restored
    seed: int = 0


@dataclass
class GCS:
    """Growing Cell Structures over vectors x = [task | q].

    Same ``[task | q]`` convention as ``gng.py``: the BMU search uses only the
    leading ``task_dim`` dims, adaptation moves the full vector, so each node's
    q part converges to a representative configuration for its workspace cell
    and ``seed_q`` returns it as a MoveIt IK seed.
    """

    dim: int
    task_dim: int
    params: GCSParams = field(default_factory=GCSParams)

    def __post_init__(self):
        self._rng = np.random.default_rng(self.params.seed)
        self.W = np.empty((0, self.dim), dtype=np.float64)
        self.error = np.empty((0,), dtype=np.float64)     # source: nv[]
        self.pinned = np.empty((0,), dtype=bool)
        self._edges = {}
        self._adj = {}
        self._step = 0
        self._w = np.zeros(self.dim)
        self._w[: self.task_dim] = 1.0

    # ---- helpers ------------------------------------------------------------
    def _dist2(self, x):
        d = (self.W - x) * self._w
        return np.einsum('ij,ij->i', d, d)

    def _add_node(self, vec, pinned=False):
        self.W = np.vstack([self.W, vec[None, :]])
        self.error = np.append(self.error, 0.0)
        self.pinned = np.append(self.pinned, bool(pinned))
        idx = len(self.W) - 1
        self._adj[idx] = set()
        return idx

    def _set_edge(self, i, j):
        if i == j:
            return
        self._edges[frozenset((i, j))] = 0
        self._adj[i].add(j)
        self._adj[j].add(i)

    def _del_edge(self, i, j):
        self._edges.pop(frozenset((i, j)), None)
        self._adj[i].discard(j)
        self._adj[j].discard(i)

    def _rebuild_adj(self):
        self._adj = {i: set() for i in range(len(self.W))}
        for e in self._edges:
            a, b = tuple(e)
            self._adj[a].add(b)
            self._adj[b].add(a)

    # ---- init ---------------------------------------------------------------
    def init_simplex(self, X):
        """GCS_init: three mutually connected nodes (a 2-simplex)."""
        X = np.asarray(X, dtype=np.float64)
        idx = self._rng.choice(len(X), size=min(3, len(X)), replace=False)
        for i in idx:
            self._add_node(X[i].copy())
        for a in range(len(self.W)):
            for b in range(a + 1, len(self.W)):
                self._set_edge(a, b)
        return self

    def seed_boundary(self, W_seed, edges):
        """Pin fixed boundary-shell nodes before fitting (same contract as GNG:
        pinned nodes are frozen and never removed, so the hull stays anchored on
        the true reachable surface). Call instead of init_simplex."""
        for vec in np.asarray(W_seed, dtype=np.float64):
            self._add_node(vec.copy(), pinned=True)
        for i, j in edges:
            if i != j:
                self._set_edge(int(i), int(j))
        return self

    # ---- learning -----------------------------------------------------------
    def step(self, x):
        """GCS_learning body for one sample."""
        self._step += 1
        p = self.params
        d2 = self._dist2(x)
        # top-2 by partition, not a full argsort: this runs once per sample over
        # every node (3000 for the shipped action maps), and it is the hot loop.
        two = np.argpartition(d2, 1)[:2]
        k, h = (int(two[0]), int(two[1])) if d2[two[0]] <= d2[two[1]] \
            else (int(two[1]), int(two[0]))
        dis = np.sqrt(d2)

        self.error[k] += dis[k]
        if not self.pinned[k]:
            self.W[k] += (x - self.W[k]) * p.rlearn1
        self._set_edge(k, h)          # source: nec reset to 1 (age)

        self.error *= p.rdis          # temporal discount
        neigh = np.fromiter(self._adj.get(k, ()), dtype=np.int64)
        if len(neigh):
            free = neigh[~self.pinned[neigh]]
            if len(free):
                w = (np.exp(-dis[free] / p.rgauss) * p.rlearn2)[:, None]
                self.W[free] += (x - self.W[free]) * w
            self.error[neigh] += dis[k] * 0.2

        if self._step % p.add_every == 0 and len(self.W) < p.max_nodes:
            self.add_node()

    def add_node(self):
        """GCS_add + the restored simplex step. Returns (g, k, h) or None."""
        p = self.params
        if len(self.W) == 0:
            return None
        k = int(np.argmax(self.error))
        if self.error[k] <= p.add_threshold:
            return None
        neigh = list(self._adj.get(k, ()))
        if not neigh:
            return None
        h = max(neigh, key=lambda i: self.error[i])

        self.error[k] *= 0.5
        self.error[h] *= 0.5
        g = self._add_node((self.W[k] + self.W[h]) * 0.5)
        self.error[g] = (self.error[k] + self.error[h]) * 0.1
        self._set_edge(g, k)
        self._set_edge(g, h)
        self._del_edge(k, h)

        if p.simplex_repair:
            # Meso-HSR/GNG.h:634-640, restored: g inherits every neighbour that
            # h and k SHARE. Splitting edge h-k splits every simplex that edge
            # belonged to, and each half only closes if g joins that simplex's
            # remaining vertices. Skipping it is what degrades a cell structure
            # into a plain graph.
            for i in self._adj[h] & self._adj[k]:
                if i not in (g, h, k):
                    self._set_edge(g, i)
        return g, k, h

    def fit(self, X, epochs=1):
        X = np.asarray(X, dtype=np.float64)
        if len(self.W) == 0:
            self.init_simplex(X)
        for _ in range(epochs):
            for i in self._rng.permutation(len(X)):
                self.step(X[i])
        return self

    # ---- invariant ----------------------------------------------------------
    def simplex_violations(self, g, k, h):
        """Neighbours common to k and h that g failed to inherit (should be [])."""
        return sorted(i for i in (self._adj[h] & self._adj[k])
                      if i not in (g, h, k) and i not in self._adj[g])

    # ---- query surface (identical semantics to GNG) -------------------------
    def query(self, task_vec, k=1):
        x = np.zeros(self.dim)
        x[: self.task_dim] = task_vec
        return np.argsort(self._dist2(x))[:k]

    def query_radius(self, task_vec, radius, max_k=None):
        x = np.zeros(self.dim)
        x[: self.task_dim] = task_vec
        d2 = self._dist2(x)
        within = np.where(d2 <= radius * radius)[0]
        if len(within) == 0:
            within = np.array([int(np.argmin(d2))])
        within = within[np.argsort(d2[within])]
        return within[:max_k] if max_k is not None else within

    def seed_q(self, task_vec):
        i = int(self.query(task_vec, k=1)[0])
        return self.W[i, self.task_dim:].copy()

    def remove_nodes(self, indices):
        victims = {int(i) for i in indices if not self.pinned[int(i)]}
        if not victims or len(self.W) - len(victims) < 2:
            return
        keep = [i for i in range(len(self.W)) if i not in victims]
        remap = {old: new for new, old in enumerate(keep)}
        self.W = self.W[keep]
        self.error = self.error[keep]
        self.pinned = self.pinned[keep]
        self._edges = {frozenset((remap[a], remap[b])): age
                       for (a, b), age in ((tuple(e), v)
                                           for e, v in self._edges.items())
                       if a in remap and b in remap}
        self._rebuild_adj()

    # ---- persistence (same npz layout as GNG) -------------------------------
    def save(self, path):
        edges = np.array([list(e) for e in self._edges], dtype=np.int64) \
            if self._edges else np.empty((0, 2), dtype=np.int64)
        ages = np.array(list(self._edges.values()), dtype=np.int64)
        np.savez(path, W=self.W, error=self.error, edges=edges, ages=ages,
                 dim=self.dim, task_dim=self.task_dim, pinned=self.pinned)

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
        g._rebuild_adj()
        return g
