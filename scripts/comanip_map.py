#!/usr/bin/env python3
"""Co-manipulation capability map: where can 4 arms hold ONE rigid object?

Two arms hang off the SAME gantry, so they cannot pick independent gantry
poses. That coupling is what makes the 4-arm problem hard, and it is what this
script evaluates:

    feasible(T_obj) =  [ EXISTS gantry_1 pose where arm_1 AND arm_2 reach their handles ]
                  AND  [ EXISTS gantry_2 pose where arm_3 AND arm_4 reach their handles ]

Reachability is evaluated in the GANTRY-LOCAL frame, where an arm's reachable
set is independent of where its gantry sits:

    world -> t{N}_base (0, +-0.36, 2.05) -> rail (+x) -> Rz(pi/2 + theta), z-0.04

so  p_local = Rz(-(pi/2+theta)) * (p_world - base - [rail,0,0] - [0,0,-0.04]).

Note this uses the raw FK dataset rather than the per-arm GNG model: the GNG
nodes were trained in WORLD coordinates and keep only one representative q per
node, which collapses the gantry redundancy and cannot answer an EXISTS query.

Usage (from repo root, no ROS needed):
    python3 scripts/comanip_map.py --maps data/maps --out data/comanip
"""

from __future__ import annotations

import argparse
import os

import numpy as np
from scipy.spatial import cKDTree

# arm -> (gantry index, gantry base in world)
ARM_GANTRY = {'arm1': 0, 'arm2': 0, 'arm3': 1, 'arm4': 1}
GANTRY_BASE = {0: np.array([0.0, 0.36, 2.05]), 1: np.array([0.0, -0.36, 2.05])}
GANTRY_ARMS = {0: ['arm1', 'arm2'], 1: ['arm3', 'arm4']}
Z_OFFSET = np.array([0.0, 0.0, -0.04])          # rotation joint origin


def to_local(p_world, base, rail, theta):
    """World points -> gantry-local frame for one gantry pose."""
    v = p_world - base - np.array([rail, 0.0, 0.0]) - Z_OFFSET
    ang = -(np.pi / 2.0 + theta)
    c, s = np.cos(ang), np.sin(ang)
    return np.column_stack([c * v[:, 0] - s * v[:, 1],
                            s * v[:, 0] + c * v[:, 1],
                            v[:, 2]])


def local_cloud(maps_dir, name, gantry):
    """Reachable EE positions of one arm, expressed in its gantry frame."""
    d = np.load(os.path.join(maps_dir, f'{name}_dataset.npz'))
    p, q = d['pose'][:, :3].astype(np.float64), d['q']
    base = GANTRY_BASE[gantry]
    v = p - base - Z_OFFSET
    v[:, 0] -= q[:, 0]
    ang = -(np.pi / 2.0 + q[:, 1])
    c, s = np.cos(ang), np.sin(ang)
    return np.column_stack([c * v[:, 0] - s * v[:, 1],
                            s * v[:, 0] + c * v[:, 1],
                            v[:, 2]])


def handle_world(centres, yaw, offsets):
    """(M,3) object centres + yaw -> (n_handles, M, 3) world handle positions."""
    d = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    return np.stack([centres + o * d for o in offsets])


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maps', default='data/maps')
    p.add_argument('--out', default='data/comanip')
    p.add_argument('--length', type=float, default=1.2, help='beam length (m)')
    p.add_argument('--reach-radius', type=float, default=0.05,
                   help='handle counts as reachable within this distance of an FK sample')
    p.add_argument('--grid', type=float, default=0.05, help='object centre grid step (m)')
    p.add_argument('--z', type=float, nargs='+', default=[1.2, 1.4, 1.6])
    p.add_argument('--yaw', type=float, nargs='+', default=[0.0, np.pi / 2])
    p.add_argument('--rail-step', type=float, default=0.2, help='gantry rail sweep step (m)')
    p.add_argument('--rot-step', type=float, default=0.4, help='gantry rotation sweep step (rad)')
    p.add_argument('--xlim', type=float, nargs=2, default=[-0.5, 2.5])
    p.add_argument('--ylim', type=float, nargs=2, default=[-1.2, 1.2])
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)

    trees = {}
    for a, g in ARM_GANTRY.items():
        loc = local_cloud(args.maps, a, g)
        trees[a] = cKDTree(loc)
        print(f'{a}: {len(loc)} FK samples, local radius max {np.linalg.norm(loc, axis=1).max():.3f} m')

    rails = np.arange(0.0, 2.0 + 1e-9, args.rail_step)
    rots = np.arange(-np.pi, np.pi, args.rot_step)
    print(f'gantry sweep: {len(rails)} rail x {len(rots)} rot = {len(rails) * len(rots)} poses/gantry')

    gx = np.arange(args.xlim[0], args.xlim[1] + args.grid, args.grid)
    gy = np.arange(args.ylim[0], args.ylim[1] + args.grid, args.grid)
    XX, YY = np.meshgrid(gx, gy, indexing='ij')
    # the two outermost handles are gripped by gantry_1, the two innermost by
    # gantry_2 would be arbitrary -- instead pair them by side: gantry_1 (y>0)
    # takes handles 0,1 and gantry_2 (y<0) takes handles 2,3 along the beam.
    offs = np.linspace(-args.length / 2.0, args.length / 2.0, 4)

    results = {}
    for z in args.z:
        for yaw in args.yaw:
            centres = np.column_stack([XX.ravel(), YY.ravel(), np.full(XX.size, z)])
            H = handle_world(centres, yaw, offs)             # (4, M, 3)

            # Which handle pair a gantry grips, and which of its two arms takes
            # which handle, decides feasibility outright -- so take the best over
            # all assignments instead of fixing one. (The RL policy later makes
            # this choice; here it must not be pre-empted.)
            best = {}                                        # (gantry, handle pair) -> mask
            for g in (0, 1):
                for hp in ((0, 1), (2, 3)):
                    ok_any = np.zeros(len(centres), dtype=bool)
                    for a_lo, a_hi in (GANTRY_ARMS[g], GANTRY_ARMS[g][::-1]):
                        for rail in rails:
                            for rot in rots:
                                pend = ~ok_any               # only test undecided centres
                                if not pend.any():
                                    break
                                idx = np.flatnonzero(pend)
                                l1 = to_local(H[hp[0]][idx], GANTRY_BASE[g], rail, rot)
                                d1, _ = trees[a_lo].query(l1)
                                m = d1 < args.reach_radius
                                if not m.any():
                                    continue
                                l2 = to_local(H[hp[1]][idx[m]], GANTRY_BASE[g], rail, rot)
                                d2, _ = trees[a_hi].query(l2)
                                ok_any[idx[m][d2 < args.reach_radius]] = True
                    best[(g, hp)] = ok_any

            # the two gantries must grip complementary handle pairs
            feas = ((best[(0, (0, 1))] & best[(1, (2, 3))]) |
                    (best[(0, (2, 3))] & best[(1, (0, 1))]))
            per_gantry = [best[(0, (0, 1))] | best[(0, (2, 3))],
                          best[(1, (0, 1))] | best[(1, (2, 3))]]
            key = f'z{z:.2f}_yaw{yaw:.2f}'
            results[key] = feas.reshape(XX.shape)
            results[key + '_g1'] = per_gantry[0].reshape(XX.shape)
            results[key + '_g2'] = per_gantry[1].reshape(XX.shape)
            print(f'  {key}: gantry_1 {per_gantry[0].sum():5d} | gantry_2 {per_gantry[1].sum():5d} '
                  f'| BOTH {feas.sum():5d}/{len(feas)} ({100.0 * feas.mean():.1f}%), '
                  f'area {feas.sum() * args.grid ** 2:.3f} m^2')

    np.savez(os.path.join(args.out, 'comanip_map.npz'),
             gx=gx, gy=gy, length=args.length, reach_radius=args.reach_radius, **results)
    print(f'\nsaved {os.path.join(args.out, "comanip_map.npz")}')


if __name__ == '__main__':
    main()
