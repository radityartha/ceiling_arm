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
"""

from __future__ import annotations

import argparse

import numpy as np

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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--out', default='model.npz')
    ap.add_argument('--task', choices=['pos', 'pose'], default='pos')
    ap.add_argument('--ori-weight', type=float, default=0.3)
    ap.add_argument('--max-nodes', type=int, default=2000)
    ap.add_argument('--lam', type=int, default=200)
    ap.add_argument('--epochs', type=int, default=2)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    data = np.load(args.dataset)
    X, task_dim = build_matrix(data, args.task, args.ori_weight)

    params = GNGParams(max_nodes=args.max_nodes, lam=args.lam, seed=args.seed)
    g = GNG(dim=X.shape[1], task_dim=task_dim, params=params)
    g.fit(X, epochs=args.epochs)

    hits, node_manip = annotate(g, X, data['manip'])
    g.save(args.out)
    # store node stats + joint order alongside the model
    base = args.out[:-4] if args.out.endswith('.npz') else args.out
    names = data['joint_names'] if 'joint_names' in data \
        else np.array([], dtype=str)
    np.savez(base + '_stats.npz', hits=hits, manip=node_manip,
             joint_names=names)
    print(f'Trained GNG: {len(g.W)} nodes, {len(g._edges)} edges, '
          f'task_dim={task_dim}. Saved {args.out} (+ _stats).')


if __name__ == '__main__':
    main()
