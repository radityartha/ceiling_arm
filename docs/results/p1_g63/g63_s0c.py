#!/usr/bin/env python3
"""G63 S0c: live D455 cloud (capture_cloud.py) vs the frozen env map EnvChecker uses.

NEW = live 2 cm voxel (seen >= 2 frames, in the map crop, outside the robot at the
given config + pad) with NO map voxel within --tol. NEW voxels are clustered (5 cm
connectivity); each cluster is reported with its bbox and its distance to the G63
pose tool points / board centres. Board region (near arm_2/arm_3 gripper) is
reported separately (the boards are not in the URDF).

    python3 g63_s0c.py --capture hw_.../s0c_capture.npz --rail1 1.50 --rail2 0 \
        --poses poses_real_v1.json --out hw_.../s0c_report.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
from env_collision import ENV_MAP, RobotGeom, load_env_map, point_distances  # noqa: E402

CROP = dict(x=(-1.2, 3.0), y=(-2.0, 2.0), z=(0.30, 2.00))   # = build_env_map.py
RES, MIN_FRAMES, SELF_PAD = 0.02, 2, 0.20                     # = map provenance
BOARD_ARMS = ('t1_a2', 't2_a1')                               # arm_2, arm_3


def voxels(cap, ns):
    xyz, fr = cap[f'{ns}_xyz'], cap[f'{ns}_frames']
    k = np.floor(xyz / RES).astype(np.int64)
    uk, inv = np.unique(k, axis=0, return_inverse=True)
    best = np.zeros(len(uk), int)
    np.maximum.at(best, inv.ravel(), fr)
    c = (uk[best >= MIN_FRAMES] + 0.5) * RES
    m = np.ones(len(c), bool)
    for i, ax in enumerate('xyz'):
        m &= (c[:, i] >= CROP[ax][0]) & (c[:, i] <= CROP[ax][1])
    return c[m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True)
    ap.add_argument('--map', default=ENV_MAP)
    ap.add_argument('--rail1', type=float, required=True)
    ap.add_argument('--rail2', type=float, required=True)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--min-cluster', type=int, default=10)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    cap = np.load(a.capture)
    env = load_env_map(a.map)['centers']
    prov = json.loads(str(load_env_map(a.map)['provenance']))
    # REST arm config = the map's own capture config (all arms REST), rails = now.
    cfg = dict(prov.get('config') or {})
    if not cfg:
        g33 = np.load(os.path.join(HERE, '..', 'p1_g33', 'env_static_map.npz'))
        cfg = json.loads(str(g33['provenance']))['config']
    cfg['t1_linear_joint'], cfg['t2_linear_joint'] = a.rail1, a.rail2
    cfg['t1_rotation_joint'] = cfg['t2_rotation_joint'] = 0.0
    rg = RobotGeom()
    q = rg.q_from(cfg)

    P = json.load(open(a.poses))
    pts = []
    for arm in P['arms'].values():
        for p in arm['poses']:
            pts += [(p['id'], 'tool', p['tool_xyz']), (p['id'], 'board', p['board_centre'])]
    ptree = cKDTree(np.array([x[2] for x in pts]))

    tree_env = cKDTree(env)
    rep = {'capture': a.capture, 'map': os.path.relpath(a.map, HERE), 'map_voxels': len(env),
           'rails': [a.rail1, a.rail2], 'tol': a.tol, 'per_cam': {}}
    allnew = []
    for ns in ('rgbd', 'rgbd2'):
        c = voxels(cap, ns)
        d, _ = point_distances(rg, q, c, SELF_PAD)
        keep = d > SELF_PAD
        db, _ = point_distances(rg, q, c, 0.45, only=BOARD_ARMS)
        board = keep & (db <= 0.45)
        live = c[keep & ~board]
        dn, _ = tree_env.query(live)
        new = live[dn > a.tol]
        rep['per_cam'][ns] = dict(voxels=len(c), robot=int((~keep).sum()), board_region=int(board.sum()),
                                  compared=len(live), matched=int((dn <= a.tol).sum()), new=len(new))
        allnew.append(new)
    new = np.unique(np.round(np.concatenate(allnew) / RES).astype(np.int64), axis=0) * RES
    clusters, vox_all = [], []
    if len(new):
        pairs = cKDTree(new).query_pairs(0.05, output_type='ndarray')
        from scipy.sparse import coo_matrix
        g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(new),) * 2)
        n, lab = connected_components(g, directed=False)
        for k in range(n):
            v = new[lab == k]
            if len(v) < a.min_cluster:
                continue
            dd, ii = ptree.query(v)
            j = int(np.argmin(dd))
            vox_all.append(np.c_[v, np.full(len(v), len(clusters))])
            clusters.append(dict(n=len(v), lo=v.min(0).round(2).tolist(), hi=v.max(0).round(2).tolist(),
                                 nearest_pose=pts[ii[j]][0], nearest_kind=pts[ii[j]][1],
                                 nearest_m=round(float(dd[j]), 3)))
    np.save(os.path.splitext(a.out)[0] + '_clusters.npy',
            np.concatenate(vox_all) if vox_all else np.zeros((0, 4)))  # xyz + cluster idx (pre-sort)
    for i, c in enumerate(clusters):
        c['idx'] = i
    clusters.sort(key=lambda c: c['nearest_m'])
    rep['new_voxels'] = len(new)
    rep['clusters'] = clusters
    json.dump(rep, open(a.out, 'w'), indent=1)
    print(json.dumps(rep['per_cam'], indent=1))
    print(f'NEW voxels {len(new)}; cluster >= {a.min_cluster}: {len(clusters)}')
    for c in clusters[:25]:
        print(f"  #{c['idx']:2d} n={c['n']:5d} bbox {c['lo']} .. {c['hi']}  dekat {c['nearest_pose']}/{c['nearest_kind']} "
              f"{c['nearest_m']:.2f} m")


if __name__ == '__main__':
    main()
