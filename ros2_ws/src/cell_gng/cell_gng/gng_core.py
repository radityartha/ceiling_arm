"""
gng_core.py — Growing Neural Gas with Utility (GNG-U), a GNG-U2-style stub.

================================ STUB NOTICE ================================
No existing GNG / GNG-U2 implementation was found in this workspace when this
package was created (searched the ros2_ws, the sibling moonshot_project tree,
~/ros2_ws, and pip).  This is therefore a MINIMAL, SELF-CONTAINED placeholder:
classic Fritzke (1995) Growing Neural Gas augmented with a per-node utility
term (Fritzke's GNG-U, 1997).  It is NOT a faithful GNG-U2 implementation.

Replace this file with the lab's real GNG-U2 code when available.  The ROS
interface in gng_node.py only depends on the small public API below
(`GrowingNeuralGas.step`, `.nodes`, `.edges`, `.prune_*`), so swapping the
algorithm should not require touching the node.
============================================================================

RESEARCH NOTE (do NOT act on this in code — it is guidance for whoever tunes
this network later): do NOT auto-tune these hyperparameters against the Gazebo
Fortress LiDAR cloud.  The simulated sensor is a generic gpu_lidar RASTER
approximation of the Livox Mid360; the real Mid360 uses a non-repetitive scan
pattern with very different point density and distribution.  Tuning the GNG on
the sim cloud would overfit to an artifact of the simulator.  Tune on the real
sensor.

The algorithm is framework-independent (pure numpy) so it can be unit-tested
without ROS.
"""

from __future__ import annotations

import numpy as np


class GrowingNeuralGas:
    """Growing Neural Gas with a utility term (GNG-U).

    Public API consumed by the ROS node:
      - step(sample): feed one D-dimensional sample (np.ndarray)
      - nodes: (N, D) array of current node positions
      - edges: list of (i, j) index pairs (i < j)
      - node_count: int
    """

    def __init__(
        self,
        dim: int = 3,
        max_nodes: int = 200,
        eps_b: float = 0.05,       # winner learning rate
        eps_n: float = 0.0006,     # neighbour learning rate
        max_age: int = 50,         # edge age before removal
        lambda_insert: int = 100,  # steps between node insertions
        alpha: float = 0.5,        # error decay on insertion
        beta: float = 0.0005,      # global error decay per step
        utility_k: float = 3.0,    # GNG-U removal aggressiveness (k); <=0 disables
        rng_seed: int = 0,
    ):
        self.dim = dim
        self.max_nodes = max_nodes
        self.eps_b = eps_b
        self.eps_n = eps_n
        self.max_age = max_age
        self.lambda_insert = lambda_insert
        self.alpha = alpha
        self.beta = beta
        self.utility_k = utility_k
        self._rng = np.random.default_rng(rng_seed)

        # Node state, stored in parallel lists/arrays indexed by node id.
        self._pos: list[np.ndarray] = []     # reference vectors
        self._error: list[float] = []        # accumulated error
        self._utility: list[float] = []      # GNG-U utility
        # Edges: dict keyed by frozenset({i, j}) -> age
        self._edges: dict[frozenset, int] = {}

        self._step_count = 0
        self._init_two_nodes()

    # ---- initialisation -----------------------------------------------------
    def _init_two_nodes(self):
        for _ in range(2):
            self._add_node(self._rng.standard_normal(self.dim) * 0.01)
        self._edges[frozenset({0, 1})] = 0

    def _add_node(self, pos: np.ndarray) -> int:
        self._pos.append(np.asarray(pos, dtype=float))
        self._error.append(0.0)
        self._utility.append(0.0)
        return len(self._pos) - 1

    # ---- public accessors ---------------------------------------------------
    @property
    def node_count(self) -> int:
        return len(self._pos)

    @property
    def nodes(self) -> np.ndarray:
        if not self._pos:
            return np.empty((0, self.dim))
        return np.vstack(self._pos)

    @property
    def edges(self) -> list[tuple[int, int]]:
        out = []
        for key in self._edges:
            i, j = tuple(key)
            out.append((min(i, j), max(i, j)))
        return out

    # ---- main update --------------------------------------------------------
    def step(self, sample: np.ndarray):
        """Process one input sample through the GNG update rules."""
        x = np.asarray(sample, dtype=float)
        if x.shape[0] != self.dim:
            raise ValueError(f"sample dim {x.shape[0]} != network dim {self.dim}")

        pos = self.nodes  # (N, D)
        d2 = np.sum((pos - x) ** 2, axis=1)
        order = np.argsort(d2)
        s1, s2 = int(order[0]), int(order[1])
        dist_s1 = float(d2[s1])
        dist_s2 = float(d2[s2])

        # 1. Accumulate error and utility for the winner.
        self._error[s1] += dist_s1
        # GNG-U utility: how much worse the 2nd-best would have been.
        self._utility[s1] += max(dist_s2 - dist_s1, 0.0)

        # 2. Age all edges emanating from s1; move s1 + neighbours toward x.
        self._pos[s1] += self.eps_b * (x - self._pos[s1])
        neighbours = self._neighbours(s1)
        for n in neighbours:
            self._pos[n] += self.eps_n * (x - self._pos[n])
            self._edges[frozenset({s1, n})] += 1

        # 3. Connect (or refresh) the s1-s2 edge.
        self._edges[frozenset({s1, s2})] = 0

        # 4. Remove stale edges and any node left isolated.
        self._prune_edges()
        self._prune_isolated_nodes()

        # 5. Periodically insert a node where error is largest.
        self._step_count += 1
        if self._step_count % self.lambda_insert == 0 and self.node_count < self.max_nodes:
            self._insert_node()

        # 6. GNG-U: drop the least-useful node when utility is tiny vs max error.
        if self.utility_k > 0:
            self._prune_low_utility()

        # 7. Global error + utility decay (forgetting).
        for i in range(self.node_count):
            self._error[i] *= (1.0 - self.beta)
            self._utility[i] *= (1.0 - self.beta)

    # ---- helpers ------------------------------------------------------------
    def _neighbours(self, i: int) -> list[int]:
        out = []
        for key in self._edges:
            if i in key:
                j = next(iter(key - {i})) if len(key) == 2 else i
                if j != i:
                    out.append(j)
        return out

    def _prune_edges(self):
        stale = [k for k, age in self._edges.items() if age > self.max_age]
        for k in stale:
            del self._edges[k]

    def _prune_isolated_nodes(self):
        connected = set()
        for key in self._edges:
            connected |= set(key)
        if len(connected) == self.node_count:
            return
        keep = sorted(connected)
        # Never let the network collapse below 2 nodes.
        if len(keep) < 2:
            return
        self._reindex(keep)

    def _prune_low_utility(self):
        if self.node_count <= 2:
            return
        max_err = max(self._error) if self._error else 0.0
        min_util = min(self._utility) if self._utility else 0.0
        if min_util <= 0.0:
            return
        # GNG-U removal criterion: max_error / min_utility > k.
        if max_err / min_util > self.utility_k:
            victim = int(np.argmin(self._utility))
            keep = [i for i in range(self.node_count) if i != victim]
            self._reindex(keep)

    def _insert_node(self):
        q = int(np.argmax(self._error))           # max-error node
        nbrs = self._neighbours(q)
        if not nbrs:
            return
        f = max(nbrs, key=lambda n: self._error[n])  # max-error neighbour
        new_pos = 0.5 * (self._pos[q] + self._pos[f])
        r = self._add_node(new_pos)
        # Rewire: remove q-f, add q-r and r-f.
        self._edges.pop(frozenset({q, f}), None)
        self._edges[frozenset({q, r})] = 0
        self._edges[frozenset({r, f})] = 0
        # Redistribute error.
        self._error[q] *= self.alpha
        self._error[f] *= self.alpha
        self._error[r] = self._error[q]
        self._utility[r] = 0.5 * (self._utility[q] + self._utility[f])

    def _reindex(self, keep: list[int]):
        """Compact node arrays to `keep` and remap edge indices."""
        remap = {old: new for new, old in enumerate(keep)}
        self._pos = [self._pos[i] for i in keep]
        self._error = [self._error[i] for i in keep]
        self._utility = [self._utility[i] for i in keep]
        new_edges: dict[frozenset, int] = {}
        for key, age in self._edges.items():
            i, j = tuple(key)
            if i in remap and j in remap:
                new_edges[frozenset({remap[i], remap[j]})] = age
        self._edges = new_edges
