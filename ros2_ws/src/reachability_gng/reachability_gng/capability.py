"""Layer 2 -- the capability oracle the coordination scheduler queries.

An index over TASK SPACE where every node carries two payloads:

    node = ( xyz , mask G_a(t) over the gantry-pose grid , canonical arm q )

`mask` answers "from which gantry poses can this arm reach here?"; `canonical`
answers "with which arm posture?". Committing to one posture per (node, pose)
is what makes capability a well-defined function of the pose -- without it,
collision filtering cannot be expressed as a set operation at all (see
irm_sweep.canonical_table).

Index shape
-----------
Two variants, so the choice can be settled by measurement rather than taste:

  grid  uniform voxel grid over the bounding box. O(1) lookup, predictable
        quantisation, no training.   <-- DEFAULT, and it is not close
  topo  GNG graph trained on the observed target distribution.

MEASURED (`validate`, equal node count = equal memory): grid wins at every
budget. On a realistic surface-concentrated target distribution, 3000 cells:
IoU 91.4% (grid) vs 72.9% (topo), and topo does not improve with more nodes
(71.7 / 72.9 / 73.1 at 1k / 3k / 8k) nor with 6.5x more training (75.7%) nor
with boundary pinning (77.1% mean but p5 collapses to 0.4%).

Downstream that is not cosmetic: pair co-feasibility comes out 78.6% with the
topo index against a true 68.5%, i.e. 11.2% of pairwise decisions wrong, versus
2.65% for the grid. The error is signed -- topo is OPTIMISTIC -- so a scheduler
built on it schedules concurrency that fails at execution time.

WHY. GNG places nodes by DATA DENSITY. Mask error is governed by how fast
G_a(t) changes in space, and that gradient comes from arm kinematics, not from
where objects happen to sit -- the two are unrelated. So GNG piles nodes inside
dense blobs where the mask is nearly constant, and starves the gaps between
them where a query can land far from any node. Adaptive-density indexing is the
wrong adaptivity for this job; beating a grid would need adaptation to mask
gradient, which is not what GNG or GCS do.

This settles the INDEX only. Topology is untouched elsewhere and remains right
there: the environment map O(t) is a GNG (env_gng.py), the action map xyz->q is
sensei's GCS, and the Meso set operations run graph-to-graph on those.

Symmetry
--------
One map per GANTRY, not per arm. The two arms of a gantry are exactly a
half-turn apart (irm_sweep.base_pose), so the partner's mask is this one
rolled by half the rotation axis -- exact, verified to 2e-16. A further 2x is
available across gantries via a y-shift of the query point; not taken here
because it would force the node set to cover the shifted queries too.

Subcommands
-----------
  build     train/lay out the index, fill masks + canonical configs
  validate  topo vs grid at equal memory, against the dense sweep
  overlap   Meso GNG_HSR_Topological_Main between the two arms of a gantry:
            shared zone (mutex resource) and handover set, as a function of r
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from reachability_gng.irm_sweep import (approach_filter, canon_score,
                                        make_grid, to_base_frame)

# right-plate arm of each gantry; the left-plate partner is a half-turn roll
GANTRY_ARM = {1: 'arm1', 2: 'arm3'}
PARTNER = {1: 'arm2', 2: 'arm4'}


# --------------------------------------------------------------------------
# target distributions
# --------------------------------------------------------------------------
def sample_targets(dist, n, rng, bbox, surfaces=(1.05, 1.20, 1.35)):
    """Draw n plausible object positions.

    uniform  filled box -- the assumption a voxel grid implicitly makes
    surface  objects rest ON things: a few horizontal planes, clustered in xy.
             This is what a real cell looks like, and the case the two index
             shapes should be judged on.
    """
    (x0, x1), (y0, y1), _ = bbox
    if dist == 'uniform':
        return np.stack([rng.uniform(x0, x1, n),
                         rng.uniform(y0, y1, n),
                         rng.uniform(bbox[2][0], bbox[2][1], n)], axis=1)
    if dist != 'surface':
        raise ValueError(dist)
    # clustered blobs sitting on a handful of shelf heights
    n_blob = 12
    cx = rng.uniform(x0, x1, n_blob)
    cy = rng.uniform(y0, y1, n_blob)
    cz = rng.choice(surfaces, n_blob)
    k = rng.integers(0, n_blob, n)
    p = np.stack([cx[k] + rng.normal(0, 0.12, n),
                  cy[k] + rng.normal(0, 0.12, n),
                  cz[k] + np.abs(rng.normal(0, 0.02, n))], axis=1)
    p[:, 0] = np.clip(p[:, 0], x0, x1)
    p[:, 1] = np.clip(p[:, 1], y0, y1)
    return p


# --------------------------------------------------------------------------
# mask + canonical config for an arbitrary set of task-space points
# --------------------------------------------------------------------------
def build_payload(arm, pts, tree, score, G, tols, K):
    """(masks (n_tol, N, P) bool, canon (N, P) int32) for points `pts`.

    Masks at several tolerances because the Meso exclusion radius r is read off
    the mask at tolerance r -- "within r of the other arm's reachable set" is
    the same statement as "reachable at tolerance r".
    """
    N, P = len(pts), len(G)
    q = to_base_frame(arm, G[None, :, 0], G[None, :, 1], pts[:, None, :])
    dist, idx = tree.query(q.reshape(-1, 3), k=K, workers=-1)
    masks = np.stack([(dist[:, 0] < t).reshape(N, P) for t in tols])
    ok = dist < tols[0]
    sc = np.where(ok, score[idx], -np.inf)
    canon = idx[np.arange(len(idx)), sc.argmax(axis=1)]
    canon = np.where(ok.any(axis=1), canon, -1).reshape(N, P).astype(np.int32)
    return masks, canon


class CapabilityMap:
    """The oracle itself: index + payload, with the half-turn symmetry applied
    on read so only one arm per gantry is ever stored."""

    def __init__(self, nodes, edges, masks, canon, lin, rot, tols, gantry):
        self.nodes, self.edges = nodes, edges
        self.masks, self.canon = masks, canon
        self.lin, self.rot, self.tols = lin, rot, tols
        self.gantry = gantry
        self._tree = None

    @property
    def tree(self):
        from scipy.spatial import cKDTree
        if self._tree is None:
            self._tree = cKDTree(self.nodes)
        return self._tree

    def lookup(self, xyz):
        """Index of the node representing this position (BMU)."""
        return self.tree.query(np.atleast_2d(xyz), k=1, workers=-1)[1]

    def reach(self, arm, xyz, tol_i=0):
        """(L, Rn) bool: gantry poses from which `arm` reaches `xyz`."""
        L, Rn = len(self.lin), len(self.rot)
        m = self.masks[tol_i][self.lookup(xyz)].reshape(-1, L, Rn)
        if arm == PARTNER[self.gantry]:      # half-turn apart, exactly
            m = np.roll(m, Rn // 2, axis=2)
        elif arm != GANTRY_ARM[self.gantry]:
            raise ValueError(f'{arm} is not on gantry {self.gantry}')
        return m[0] if np.ndim(xyz) == 1 else m

    def save(self, path):
        np.savez_compressed(path, nodes=self.nodes, edges=self.edges,
                            masks=self.masks, canon=self.canon, lin=self.lin,
                            rot=self.rot, tols=self.tols, gantry=self.gantry)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d['nodes'], d['edges'], d['masks'], d['canon'], d['lin'],
                   d['rot'], d['tols'], int(d['gantry']))


# --------------------------------------------------------------------------
# index construction
# --------------------------------------------------------------------------
def build_index(kind, samples, n_cells, rng, bbox):
    """Return (nodes (N,3), edges (E,2)) for the requested index shape."""
    if kind == 'grid':
        # cell counts per axis proportional to extent, product ~= n_cells
        ext = np.array([b[1] - b[0] for b in bbox], dtype=float)
        step = (ext.prod() / n_cells) ** (1 / 3)
        dims = np.maximum(np.round(ext / step).astype(int), 1)
        axes = [np.linspace(b[0], b[1], d) for b, d in zip(bbox, dims)]
        nodes = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
        # 6-neighbour lattice edges
        gi = np.arange(len(nodes)).reshape(dims)
        e = []
        for ax in range(3):
            a = np.moveaxis(gi, ax, 0)
            e.append(np.stack([a[:-1].ravel(), a[1:].ravel()], axis=1))
        return nodes, np.concatenate(e) if e else np.empty((0, 2), int)

    if kind != 'topo':
        raise ValueError(kind)
    from reachability_gng.gng import GNG, GNGParams
    g = GNG(dim=3, task_dim=3,
            params=GNGParams(max_nodes=n_cells, lam=max(1, len(samples) // n_cells),
                             seed=int(rng.integers(1 << 30))))
    g.fit(samples, epochs=1)
    edges = (np.array([sorted(e) for e in g._edges], dtype=int)
             if g._edges else np.empty((0, 2), int))
    return g.W[:, :3].copy(), edges


def cmd_build(args):
    from scipy.spatial import cKDTree

    cl = np.load(args.cloud)
    keep = approach_filter(cl['axis'], args.approach)
    poly = cl['poly'][keep].astype(np.float64)
    score = canon_score(args.canon, cl['manip'][keep].astype(np.float64),
                        cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    tree = cKDTree(poly[:, -1])

    lin, rot = make_grid(args.lin_step, args.rot_step)
    G = np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)
    bbox = [args.tx, args.ty, args.tz]
    rng = np.random.default_rng(args.seed)
    samples = sample_targets(args.dist, args.n_samples, rng, bbox)

    t0 = time.time()
    nodes, edges = build_index(args.index, samples, args.n_cells, rng, bbox)
    arm = GANTRY_ARM[args.gantry]
    masks, canon = build_payload(arm, nodes, tree, score, G,
                                 np.asarray(args.tol), args.k_cand)
    cm = CapabilityMap(nodes, edges, masks, canon, lin, rot,
                       np.asarray(args.tol), args.gantry)
    cm.save(args.out)
    mem = (masks.nbytes + canon.nbytes) / 1e6
    print(f'{args.index} index, gantry {args.gantry} ({arm}), '
          f'{len(nodes)} nodes, {len(edges)} edges, {time.time()-t0:.1f}s')
    print(f'  reachable nodes  {(masks[0].any(1)).mean()*100:5.1f}%  '
          f'|G(t)| median {np.median(masks[0].sum(1)[masks[0].any(1)]):.0f}'
          f' of {len(G)}')
    print(f'  payload {mem:.1f} MB -> {args.out}')
    return 0


# --------------------------------------------------------------------------
# validate: does the topological index actually earn its place?
# --------------------------------------------------------------------------
def cmd_validate(args):
    """topo vs grid at EQUAL MEMORY, scored against the dense sweep.

    Query points are drawn from the same distribution the index was built on.
    For each, the index returns the mask of its representing node; ground truth
    is the mask computed directly at the query point. Reported as IoU over the
    gantry-pose set, since that set is what every downstream decision consumes.

    What this does NOT settle: it scores lookup accuracy only. The structural
    argument for `topo` -- that Meso set operations return a graph with edges
    intact, so connectivity survives them -- is not captured by IoU and has to
    be argued separately.
    """
    from scipy.spatial import cKDTree

    cl = np.load(args.cloud)
    keep = approach_filter(cl['axis'], args.approach)
    poly = cl['poly'][keep].astype(np.float64)
    score = canon_score('manip', cl['manip'][keep].astype(np.float64),
                        cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    tree = cKDTree(poly[:, -1])
    lin, rot = make_grid(args.lin_step, args.rot_step)
    G = np.stack(np.meshgrid(lin, rot, indexing='ij'), -1).reshape(-1, 2)
    bbox = [args.tx, args.ty, args.tz]
    arm = GANTRY_ARM[1]
    tols = np.array([args.tol])

    print(f'{len(G)} gantry poses, tol {args.tol} m, '
          f'{args.n_query} query points per cell budget')
    print(f'{"dist":>8} {"cells":>7} | {"grid IoU":>18} | {"topo IoU":>18} | winner')
    print('-' * 78)
    for dist in args.dist:
        rng = np.random.default_rng(args.seed)
        train = sample_targets(dist, args.n_samples, rng, bbox)
        query = sample_targets(dist, args.n_query, rng, bbox)
        truth, _ = build_payload(arm, query, tree, score, G, tols, args.k_cand)
        truth = truth[0]
        live = truth.any(axis=1)
        for n_cells in args.cells:
            row = {}
            for kind in ('grid', 'topo'):
                nodes, _ = build_index(kind, train, n_cells,
                                       np.random.default_rng(args.seed), bbox)
                m, _ = build_payload(arm, nodes, tree, score, G, tols, args.k_cand)
                got = m[0][cKDTree(nodes).query(query, k=1, workers=-1)[1]]
                inter = (got & truth).sum(1)
                union = (got | truth).sum(1)
                iou = np.where(union > 0, inter / np.maximum(union, 1), 1.0)
                row[kind] = (iou[live].mean(), np.percentile(iou[live], 5),
                             len(nodes))
            g, t = row['grid'], row['topo']
            win = 'topo' if t[0] > g[0] + 0.002 else (
                'grid' if g[0] > t[0] + 0.002 else 'tie')
            print(f'{dist:>8} {n_cells:>7} | mean {g[0]*100:5.1f}% p5 {g[1]*100:5.1f}% '
                  f'| mean {t[0]*100:5.1f}% p5 {t[1]*100:5.1f}% | {win}')
    print('\n  IoU over the gantry-pose set G(t); p5 = worst-5% query points.')
    print('  Node budgets are matched; the grid cannot hit the request exactly')
    print('  (dimension product), so it lands within a few % either way.')
    return 0


# --------------------------------------------------------------------------
# overlap: Meso GNG_HSR_Topological_Main between the two arms of a gantry
# --------------------------------------------------------------------------
def cmd_overlap(args):
    """Shared zone and handover set between the two arms of one gantry.

    Meso-HSR/GNG.h:1223 kills every node of one arm's map lying within r of the
    other arm's map, and cuts its edges. Both operands are graphs and so is the
    result -- that is the property a grid cannot reproduce.

    r -> tol collapses the zone onto the strict intersection, which is the
    HANDOVER set. Same set, two roles: mutex resource, and the only place a
    handover can physically happen.
    """
    cm = CapabilityMap.load(args.map)
    L, Rn = len(cm.lin), len(cm.rot)
    a_mask = cm.masks                                  # (n_tol, N, P)
    b_mask = np.roll(a_mask.reshape(len(cm.tols), -1, L, Rn), Rn // 2,
                     axis=3).reshape(a_mask.shape)     # partner, half-turn
    base = a_mask[0]
    print(f'gantry {cm.gantry}: {GANTRY_ARM[cm.gantry]} vs {PARTNER[cm.gantry]}, '
          f'{len(cm.nodes)} nodes, {len(cm.edges)} edges')
    print(f'{"r":>6} | {"zone nodes":>11} | {"zone (node,pose)":>17} | '
          f'{"edges cut":>10} | role')
    print('-' * 70)
    for ri, r in enumerate(cm.tols):
        shared = base & b_mask[ri]                     # a reaches, b within r
        node_hit = shared.any(axis=1)
        # Meso cuts every edge incident on a killed node
        if len(cm.edges):
            cut = node_hit[cm.edges[:, 0]] | node_hit[cm.edges[:, 1]]
            cut_frac = cut.mean()
        else:
            cut_frac = 0.0
        role = 'HANDOVER set (strict)' if ri == 0 else 'danger zone (dilated)'
        print(f'{r:6.2f} | {node_hit.mean()*100:10.1f}% | '
              f'{shared.sum()/max(base.sum(),1)*100:16.1f}% | '
              f'{cut_frac*100:9.1f}% | {role}')
    print('\n  zone nodes  = nodes both arms can work at (handover-capable at r=tol)')
    print('  edges cut   = graph connectivity lost if the zone is removed outright')
    return 0


# --------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(q):
        q.add_argument('--cloud', default='/tmp/irm_cloud.npz')
        q.add_argument('--approach', type=float, default=45.0)
        q.add_argument('--lin-step', type=float, default=0.05)
        q.add_argument('--rot-step', type=float, default=5.0)
        q.add_argument('--tx', type=float, nargs=2, default=[0.0, 2.0])
        q.add_argument('--ty', type=float, nargs=2, default=[-0.6, 0.6])
        q.add_argument('--tz', type=float, nargs=2, default=[1.00, 1.40])
        q.add_argument('--k-cand', type=int, default=64)
        q.add_argument('--seed', type=int, default=0)

    b = sub.add_parser('build')
    common(b)
    b.add_argument('--index', default='grid', choices=['grid', 'topo'])
    b.add_argument('--gantry', type=int, default=1, choices=[1, 2])
    b.add_argument('--dist', default='surface', choices=['uniform', 'surface'])
    b.add_argument('--n-samples', type=int, default=20_000)
    b.add_argument('--n-cells', type=int, default=3000)
    b.add_argument('--tol', type=float, nargs='+', default=[0.05, 0.10, 0.20])
    b.add_argument('--canon', default='manip',
                   choices=['manip', 'sigmin', 'limits', 'home', 'combo'])
    b.add_argument('--out', default='/tmp/capability_g1.npz')
    b.set_defaults(fn=cmd_build)

    v = sub.add_parser('validate')
    common(v)
    v.add_argument('--dist', nargs='+', default=['uniform', 'surface'])
    v.add_argument('--cells', type=int, nargs='+', default=[1000, 3000, 8000])
    v.add_argument('--n-samples', type=int, default=20_000)
    v.add_argument('--n-query', type=int, default=2000)
    v.add_argument('--tol', type=float, default=0.05)
    v.set_defaults(fn=cmd_validate)

    o = sub.add_parser('overlap')
    o.add_argument('--map', default='/tmp/capability_g1.npz')
    o.set_defaults(fn=cmd_overlap)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == '__main__':
    raise SystemExit(main())
