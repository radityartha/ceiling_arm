"""Train the GNG on an FK dataset and attach per-node reachability stats.

Pipeline:
    dataset.npz  --(build [task | q] matrix)-->  GNG.fit  -->  model.npz

Task space is selectable:
    --task pos   : xyz only            (task_dim = 3)
    --task pose  : xyz + quaternion    (task_dim = 7)

Orientation columns are scaled by --ori-weight so position (metres) and
quaternion units are comparable in the BMU metric.

After training, every node is annotated with:
    hits  : number of dataset samples whose BMU is this node (reachability)
    manip : mean manipulability of those samples            (capability)
    hold  : ||gravity torque|| at the node's own q          (holding energy)

`hold` needs the robot model, so it is only computed when --config (the same
YAML data_gen used) is given; otherwise it is stored as zeros. It is the static
gravitational load of holding the node's representative configuration -- the
energy-aware arm selection uses it (see gantry_reach_executor).
"""

from __future__ import annotations

import argparse

import numpy as np
import yaml

from reachability_gng.gng import GNG, GNGParams


def build_matrix(data, task, ori_weight):
    pose = data['pose']        # (N,7) xyz + quat
    q = data['q']              # (N,8)
    if task == 'pos':
        task_feat = pose[:, :3]
    elif task == 'pose':
        task_feat = pose.copy()
        task_feat[:, 3:] *= ori_weight
    else:
        raise ValueError(task)
    X = np.hstack([task_feat, q])
    return X, task_feat.shape[1]


def boundary_mask(P, k=15, tau=0.4, chunk=800):
    """Flag the outer-surface points of a point cloud, purely from geometry.

    For each point, take its `k` nearest neighbours and the unit vectors toward
    them. An INTERIOR point is surrounded, so those unit vectors roughly cancel
    (|mean| ~ 0); a SURFACE point has neighbours only on the inward side, so they
    add up (|mean| large). Points with |mean| > `tau` are the reachable-workspace
    boundary. Same one-sidedness idea the runtime `enclose` gate uses, applied to
    the raw FK samples. Chunked brute-force kNN (no scipy/voxels)."""
    P = np.asarray(P, dtype=np.float32)
    n = len(P)
    out = np.zeros(n, dtype=bool)
    k = min(k, n - 1)
    Psq = np.einsum('ij,ij->i', P, P)                         # (N,) |p|^2
    for s in range(0, n, chunk):
        blk = P[s:s + chunk]                                  # (B,3)
        # |a-b|^2 = |a|^2 + |b|^2 - 2 a.b, so the (B,N,3) diff is never formed
        d2 = Psq[None, :] + Psq[s:s + chunk, None] - 2.0 * (blk @ P.T)  # (B,N)
        nn = np.argpartition(d2, k, axis=1)[:, :k + 1]        # k nearest + self
        rows = np.arange(blk.shape[0])[:, None]
        vec = P[nn] - blk[:, None, :]                         # (B,k+1,3)
        d = np.linalg.norm(vec, axis=2)                       # (B,k+1)
        # drop the self match (distance ~0) per row, keep k neighbours
        self_col = np.argmin(d, axis=1)
        keep = d > 1e-9
        keep[rows[:, 0], self_col] = False
        units = np.where(keep[..., None], vec / np.maximum(d[..., None], 1e-9), 0.0)
        cnt = np.maximum(keep.sum(axis=1, keepdims=True), 1)
        out[s:s + blk.shape[0]] = np.linalg.norm(units.sum(1) / cnt, axis=1) > tau
    return out


def farthest_point_sample(P, m, seed=0):
    """Indices of `m` well-spread points via farthest-point sampling (pure numpy)."""
    P = np.asarray(P, dtype=np.float64)
    n = len(P)
    if m >= n:
        return np.arange(n)
    rng = np.random.default_rng(seed)
    chosen = [int(rng.integers(n))]
    d = np.linalg.norm(P - P[chosen[0]], axis=1)
    for _ in range(m - 1):
        i = int(np.argmax(d))
        chosen.append(i)
        d = np.minimum(d, np.linalg.norm(P - P[i], axis=1))
    return np.array(chosen, dtype=int)


def knn_edges(P, k=6):
    """Undirected (i, j) edges linking each point to its k nearest others."""
    P = np.asarray(P, dtype=np.float64)
    n = len(P)
    k = min(k, n - 1)
    d2 = np.einsum('ijk,ijk->ij', P[:, None, :] - P[None, :, :],
                   P[:, None, :] - P[None, :, :])
    nn = np.argpartition(d2, k, axis=1)[:, :k + 1]
    edges = set()
    for i in range(n):
        for j in nn[i]:
            if int(j) != i:
                edges.add((min(i, int(j)), max(i, int(j))))
    return list(edges)


def annotate(g: GNG, X, manip):
    """Assign each sample to its BMU and accumulate hits + mean manipulability."""
    hits = np.zeros(len(g.W))
    manip_sum = np.zeros(len(g.W))
    for x, mm in zip(X, manip):
        i = int(np.argmin(g._dist2(x)))
        hits[i] += 1
        manip_sum[i] += mm
    node_manip = np.where(hits > 0, manip_sum / np.maximum(hits, 1), 0.0)
    return hits, node_manip


def hold_cost(cfg, node_q, joint_names):
    """||generalized gravity torque|| at each node's own q (static hold energy).

    Reuses eval.build_model (Pinocchio) so the joint order matches the dataset.
    The gantry DOFs add ~0 (linear axis horizontal, rotation about vertical), so
    `hold` reflects the arm's gravitational load -- larger when the ceiling arm
    reaches far out and must fight gravity to hold the pose."""
    from reachability_gng.eval import build_model
    pin, model, data, ee_id, order, lo, hi = build_model(cfg)
    if joint_names is not None and order != list(joint_names):
        raise SystemExit(f'joint order mismatch: {order} vs {list(joint_names)}')
    hold = np.empty(len(node_q))
    for i, q in enumerate(node_q):
        tau = pin.computeGeneralizedGravity(model, data, np.asarray(q, float))
        hold[i] = float(np.linalg.norm(tau))
    return hold


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--out', default='model.npz')
    ap.add_argument('--config', help='YAML (urdf/joints) to compute per-node '
                    'gravity holding cost; omit to store hold=0')
    ap.add_argument('--task', choices=['pos', 'pose'], default='pos')
    ap.add_argument('--ori-weight', type=float, default=0.3)
    ap.add_argument('--max-nodes', type=int, default=2000)
    ap.add_argument('--lam', type=int, default=200)
    ap.add_argument('--epochs', type=int, default=2)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--boundary-nodes', type=int, default=0,
                    help='pin this many fixed boundary-shell nodes on the true '
                    'reachable surface before growing the interior (0 = off, '
                    'legacy centroid-only map)')
    ap.add_argument('--boundary-k', type=int, default=15,
                    help='kNN used for surface detection')
    ap.add_argument('--boundary-tau', type=float, default=0.4,
                    help='one-sidedness threshold (0..1); higher = fewer, more '
                    'clearly-outer points flagged as boundary')
    ap.add_argument('--boundary-edges-k', type=int, default=6,
                    help='shell connectivity: edges per boundary node')
    args = ap.parse_args()

    data = np.load(args.dataset)
    X, task_dim = build_matrix(data, args.task, args.ori_weight)

    params = GNGParams(max_nodes=args.max_nodes, lam=args.lam, seed=args.seed)
    g = GNG(dim=X.shape[1], task_dim=task_dim, params=params)
    if args.boundary_nodes > 0:
        pos = X[:, :3]                              # metric xyz for geometry
        bmask = boundary_mask(pos, k=args.boundary_k, tau=args.boundary_tau)
        bidx = np.where(bmask)[0]
        if len(bidx) == 0:
            raise SystemExit('no boundary points found; lower --boundary-tau')
        sel = bidx[farthest_point_sample(pos[bidx], args.boundary_nodes,
                                         seed=args.seed)]
        edges = knn_edges(pos[sel], k=args.boundary_edges_k)
        g.seed_boundary(X[sel], edges)
        print(f'boundary: {len(bidx)} surface pts detected, pinned '
              f'{len(sel)} shell nodes ({len(edges)} shell edges)')
    g.fit(X, epochs=args.epochs)

    hits, node_manip = annotate(g, X, data['manip'])
    g.save(args.out)
    # store node stats + joint order alongside the model
    base = args.out[:-4] if args.out.endswith('.npz') else args.out
    names = data['joint_names'] if 'joint_names' in data \
        else np.array([], dtype=str)

    hold = np.zeros(len(g.W))
    if args.config:
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        names_list = [str(n) for n in names] if len(names) else None
        hold = hold_cost(cfg, g.W[:, task_dim:], names_list)

    np.savez(base + '_stats.npz', hits=hits, manip=node_manip, hold=hold,
             joint_names=names)
    print(f'Trained GNG: {len(g.W)} nodes ({int(g.pinned.sum())} pinned '
          f'boundary), {len(g._edges)} edges, task_dim={task_dim}. '
          f'Saved {args.out} (+ _stats, '
          f'hold={"computed" if args.config else "zeros"}).')


if __name__ == '__main__':
    main()
