"""Multi-Scale Batch-Learning Growing Neural Gas (MS-BL-GNG).

Port of the sensei's ``MS_GNG_learning`` (``Meso-HSR/GNG.h:940``, with
``GNG_MS_DATA`` at :888 and ``GNG_add`` at :593). NOT a contribution -- cited:

    Ardilla, Saputra, Kubota, "Multi-Scale Batch-Learning Growing Neural Gas
    Efficiently for Dynamic Data Distributions", Int. J. Automation Technology,
    17(3), 206-216, 2023.

Why this exists next to ``gng.py`` rather than replacing it: the manuscript says
MS-BL-GNG while the code ran plain online Fritzke GNG, and the online update is
order-dependent, so the "static" map moves when the same scene is captured
again. ``gng.py`` is left untouched as the ablation baseline (and four trained
action-map models depend on it).

WHAT BATCH LEARNING BUYS
------------------------
Within one batch every node's reference vector is FROZEN: BMU search, the
neighbour term and the error term are all computed against the weights as they
stood at batch start, and only the accumulated mean is applied at the end. So
the result cannot depend on the order samples are visited -- which is exactly
the reproducibility property the static map needs, and it also lets the whole
batch be evaluated as one matrix product instead of a Python loop.

THREE DELIBERATE DEVIATIONS FROM THE SOURCE, all recorded in
docs/p1_g5_msbl_gcs.md §A4 so they are not mistaken for part of the port:

1. ``GNG_MS_DATA`` shuffles with ``rnd()``. Here the sample order is a CANONICAL
   permutation derived from the data itself (lexsort over the vectors). With an
   RNG shuffle the mini-batch partition follows row INDEX, so permuting the
   input rows changes the map; deriving it from row CONTENT makes the map
   invariant to input order. This is determinism engineering, not algorithm.
2. ``MSN[]`` (per-level stride) and ``MSNO[]`` (node-count thresholds to step up
   a level) are *declared nowhere in this repo* -- ``GNG.h`` uses them but no
   header defines them (``grep -rn maxMSN .`` is empty). Defaults here are
   chosen, not ported: strides 8->4->2->1 (coarse to fine), stepping up at
   25/50/75% of ``max_nodes``. The last level is a FULL batch.
3. Distance uses only the leading ``task_dim`` dims, and adaptation moves the
   full ``[task | q]`` vector -- this project's existing convention (see
   ``gng.py``), kept so the two are comparable.

API-compatible with ``GNG`` (same npz layout, so either class can load either
file) for the consumers that hold a map: ``.W``, ``._edges``, ``.pinned``,
``.task_dim``, ``query``, ``query_radius``, ``seed_q``, ``save``/``load``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class BLGNGParams:
    max_nodes: int = 2000
    # rlearn1/rlearn2/rgauss keep the source's names in the comments; the field
    # names match GNGParams so callers can swap params objects.
    eps_b: float = 1.0          # rlearn1: BMU pull (batch MEAN, so 1.0 is sane)
    eps_n: float = 0.05         # rlearn2: neighbour pull
    rgauss: float = 1.0         # width of the distance-based neighbour weight
    distance_based: bool = True  # source type 2 (True) vs type 3 (plain rlearn2)
    add_threshold: float = 1.0  # GNG_add: only split if worst error > this
    ms_strides: tuple = (8, 4, 2, 1)   # MSN[]: batch = every stride-th sample
    ms_fracs: tuple = (0.25, 0.5, 0.75)  # MSNO[]: level up at these * max_nodes
    settle_batches: int = 5     # extra full batches after max_nodes is reached
    chunk: int = 2048           # samples per vectorised block (bounds memory)
    seed: int = 0               # kept for API parity; unused (nothing is random)


@dataclass
class BLGNG:
    """MS-BL-GNG over vectors x = [task | q]."""

    dim: int
    task_dim: int
    params: BLGNGParams = field(default_factory=BLGNGParams)

    def __post_init__(self):
        self.W = np.empty((0, self.dim), dtype=np.float64)
        self.error = np.empty((0,), dtype=np.float64)   # source: nv[]
        self.pinned = np.empty((0,), dtype=bool)
        self._edges = {}
        self._adj = {}
        self._level = 0
        self._batch_no = 0
        self._w = np.zeros(self.dim)
        self._w[: self.task_dim] = 1.0

    # ---- helpers ------------------------------------------------------------
    def _task(self, X):
        return np.asarray(X, dtype=np.float64)[:, : self.task_dim]

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

    def _set_edge(self, i, j, age=0):
        if i == j:
            return
        self._edges[frozenset((i, j))] = age
        self._adj[i].add(j)
        self._adj[j].add(i)

    def _del_edge(self, i, j):
        self._edges.pop(frozenset((i, j)), None)
        self._adj[i].discard(j)
        self._adj[j].discard(i)

    def _adjacency(self):
        """Dense boolean neighbour matrix for the vectorised batch pass."""
        n = len(self.W)
        A = np.zeros((n, n), dtype=bool)
        for e in self._edges:
            a, b = tuple(e)
            A[a, b] = A[b, a] = True
        return A

    def _rebuild_adj(self):
        self._adj = {i: set() for i in range(len(self.W))}
        for e in self._edges:
            a, b = tuple(e)
            self._adj[a].add(b)
            self._adj[b].add(a)

    def _keep_only(self, keep):
        remap = {old: new for new, old in enumerate(keep)}
        self.W = self.W[keep]
        self.error = self.error[keep]
        self.pinned = self.pinned[keep]
        new_edges = {}
        for e, age in self._edges.items():
            a, b = tuple(e)
            if a in remap and b in remap:
                new_edges[frozenset((remap[a], remap[b]))] = age
        self._edges = new_edges
        self._rebuild_adj()

    # ---- canonical ordering (deviation 1) -----------------------------------
    @staticmethod
    def canonical_order(X):
        """Row order derived from row CONTENT, so it survives input permutation."""
        X = np.asarray(X, dtype=np.float64)
        return np.lexsort(X.T[::-1])

    # ---- public API ---------------------------------------------------------
    def init_two(self, X):
        """Seed two nodes deterministically (content-derived, not random)."""
        X = np.asarray(X, dtype=np.float64)
        order = self.canonical_order(X)
        a, b = int(order[0]), int(order[len(order) // 2])
        if np.allclose(X[a], X[b]) and len(order) > 2:
            b = int(order[-1])
        self._add_node(X[a].copy())
        self._add_node(X[b].copy())
        self._set_edge(0, 1, 0)

    def seed_boundary(self, W_seed, edges):
        """Pin fixed boundary nodes before fitting (same contract as GNG)."""
        for vec in np.asarray(W_seed, dtype=np.float64):
            self._add_node(vec.copy(), pinned=True)
        for i, j in edges:
            if i != j:
                self._set_edge(int(i), int(j), 0)
        return self

    def learn_batch(self, Xb, grow=1):
        """One MS-BL batch: accumulate against FROZEN weights, then mean-update.

        Mirrors ``MS_GNG_learning``'s body for a batch that has already been
        selected by the multi-scale schedule. ``Xb`` rows must already be in the
        order the caller wants (``fit`` passes canonical order).
        """
        Xb = np.asarray(Xb, dtype=np.float64)
        n = len(self.W)
        if n < 2 or len(Xb) == 0:
            return
        p = self.params
        A = self._adjacency()
        Wt = self.W[:, : self.task_dim]
        nes = np.zeros(n)                     # selection counts
        upd = np.zeros((n, self.dim))         # summed deltas
        nv = np.zeros(n)                      # error accumulation this batch
        necb = np.zeros((n, n), dtype=np.int64)   # connectivity rebuilt per batch

        for s in range(0, len(Xb), p.chunk):
            xb = Xb[s:s + p.chunk]
            xt = xb[:, : self.task_dim]
            # frozen-weight distances: the whole point of batch learning
            d2 = ((xt[:, None, :] - Wt[None, :, :]) ** 2).sum(-1)
            dis = np.sqrt(d2)
            if n >= 2:
                two = np.argpartition(d2, 1, axis=1)[:, :2]
                first = two[np.arange(len(xb)),
                            np.argmin(np.take_along_axis(d2, two, 1), axis=1)]
                second = two.sum(1) - first
            k, h = first, second

            # --- BMU term
            np.add.at(nes, k, 1.0)
            np.add.at(nv, k, dis[np.arange(len(xb)), k])
            np.add.at(upd, k, (xb - self.W[k]) * p.eps_b)
            necb[k, h] += 1
            necb[h, k] += 1

            # --- neighbour term, vectorised over the whole chunk.
            # coef[t, i] is nonzero only where i neighbours sample t's BMU.
            coef = A[k]                                   # (chunk, n) bool
            if p.distance_based:
                wgt = np.exp(-dis / p.rgauss) * p.eps_n
            else:
                wgt = np.full_like(dis, p.eps_n)
            coef = coef * wgt                             # (chunk, n) float
            nes += (A[k]).sum(axis=0)
            # sum_t coef[t,i] * (x_t - W[i])  ==  (coef.T @ X) - W * coef.sum(0)
            csum = coef.sum(axis=0)
            upd += coef.T @ xb - self.W * csum[:, None]
            nv += (A[k] * dis * np.exp(-dis / 1.0) * p.eps_n).sum(axis=0)

        # --- multi-scale mini-batch update: MEAN of accumulated deltas
        live = nes > 0
        moved = live & ~self.pinned
        self.W[moved] += upd[moved] / nes[moved, None]
        self.error += nv

        # connectivity is REPLACED by this batch's co-activations (this is what
        # MS-BL uses instead of edge ages); pinned-node edges are kept, matching
        # GNG.seed_boundary's contract that the shell persists for the whole run.
        kept_pinned = {e for e in self._edges
                       if any(self.pinned[i] for i in tuple(e))}
        self._edges = {e: 0 for e in kept_pinned}
        for a, b in zip(*np.nonzero(np.triu(necb, 1))):
            self._edges[frozenset((int(a), int(b)))] = 0
        self._rebuild_adj()

        # nodes nothing selected are dropped (source zeroes them and sets nv=-1)
        keep = [i for i in range(n) if live[i] or self.pinned[i]]
        if len(keep) >= 2 and len(keep) < n:
            self._keep_only(keep)

        for _ in range(grow):
            if len(self.W) < p.max_nodes:
                self._insert()

    def _insert(self):
        """GNG_add: split the worst node against its worst neighbour."""
        if len(self.W) == 0:
            return
        k = int(np.argmax(self.error))
        if self.error[k] <= self.params.add_threshold:
            return
        neigh = list(self._adj.get(k, ()))
        if not neigh:
            return
        h = max(neigh, key=lambda i: self.error[i])
        self.error[k] *= 0.5
        self.error[h] *= 0.5
        g = self._add_node((self.W[k] + self.W[h]) * 0.5)
        self.error[g] = (self.error[k] + self.error[h]) * 0.1
        self._del_edge(k, h)
        self._set_edge(g, k, 0)
        self._set_edge(g, h, 0)

    # ---- multi-scale schedule ----------------------------------------------
    def _stride(self):
        p = self.params
        fr = len(self.W) / max(p.max_nodes, 1)
        lvl = int(np.searchsorted(np.asarray(p.ms_fracs), fr, side='right'))
        return p.ms_strides[min(lvl, len(p.ms_strides) - 1)], lvl

    def fit(self, X, epochs=0):
        """Grow to ``max_nodes`` on X, then settle.

        ``epochs`` is accepted for API parity with ``GNG.fit``; for MS-BL it sets
        the number of FULL-batch settling passes after growth (<=0 keeps the
        ``settle_batches`` default). Growth itself is driven by the multi-scale
        schedule, one node per batch, exactly as the source does.
        """
        X = np.asarray(X, dtype=np.float64)
        Xc = X[self.canonical_order(X)]      # deviation 1: content-derived order
        if len(self.W) == 0:
            self.init_two(Xc)
        p = self.params
        guard = 0
        max_batches = 4 * p.max_nodes + 100
        while len(self.W) < p.max_nodes and guard < max_batches:
            stride, _ = self._stride()
            offset = self._batch_no % stride
            self.learn_batch(Xc[offset::stride], grow=1)
            self._batch_no += 1
            guard += 1
        settle = epochs if epochs > 0 else p.settle_batches
        for _ in range(settle):
            self.learn_batch(Xc, grow=0)
        return self

    def partial_fit(self, X, grow=1):
        """One batch over X, for the ONLINE map (env_gng): the live cloud IS the
        batch, so a perception tick maps 1:1 onto an MS-BL batch."""
        X = np.asarray(X, dtype=np.float64)
        if len(self.W) == 0:
            if len(X) < 2:
                return self
            self.init_two(X)
        self.learn_batch(X[self.canonical_order(X)], grow=grow)
        return self

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
        self._keep_only([i for i in range(len(self.W)) if i not in victims])

    # ---- persistence (same npz layout as GNG, so either can load either) ----
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
