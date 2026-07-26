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

Handle layouts (--handles):
    line    : 4 handles evenly spaced along the object axis (the 1.2 m beam).
    corners : 4 handles at the corners of the object's TOP face (the 25 cm
              block). A small block cannot use `line`: 4 collinear handles on a
              0.25 m edge sit 0.083 m apart, closer than one gripper's own open
              pad gap (0.085 m, isaac_sim/workcell/grasp_verify.txt), so no four
              grippers fit at all. --min-ee-dist gates exactly that.

A small object was expected to shift the binding constraint from REACH to
ARM-ARM interference: the two arms of one gantry hang from plates 0.8 m apart
yet must converge on corners only 0.25 m apart, so they can cross. Inter-EE
distance cannot see this (it is fixed by the object's own geometry), so the
check is on the arms' mount->handle LINE SEGMENTS -- ~0 when the arms cross.

MEASURED RESULT: that shift does NOT happen, at this level of fidelity. The
crossing check removes 0 cells everywhere, with the arm order searched OR fixed
(--fixed-arm-order). The reason is a symmetry, not a slack threshold: the two
mounts are mirror-symmetric in y (+-0.4), so Rz(pi) maps arm1's mount exactly
onto arm2's, i.e. rotating the gantry by 180 deg is IDENTICAL to swapping which
arm takes which handle. Every crossing assignment therefore has a non-crossing
twin at theta+pi, and the +-180 deg rotation joint means that twin is always
available. Crossing cannot bind on this workcell.

So reach stays the binding kinematic constraint. What is NOT ruled out is real
arm-BODY collision (forearm/wrist meshes, not the straight-line stand-in here);
that needs full collision checking and is outside what this map claims. Both
distance tests are declared PROXIES: see --min-ee-dist / --min-link-clearance.

Usage (from repo root, no ROS needed):
    python3 scripts/comanip_map.py --maps data/maps --out data/comanip
    python3 scripts/comanip_map.py --handles corners --length 0.25 \
        --out data/comanip_block
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
# Arm shoulder mounts in the GANTRY-LOCAL frame (= t{N}_rotation_link), from
# workcell_full.urdf: mount_plate_{right,left} at y=-+0.4, z=-0.0325, then each
# a{1,2}_base_joint drops a further -0.025. armN_a1 hangs off the RIGHT plate,
# armN_a2 off the LEFT one. Used as the fixed end of the arm-crossing proxy.
MOUNT_LOCAL = {'arm1': np.array([0.0, -0.4, -0.0575]),   # t1_a1, right plate
               'arm2': np.array([0.0, +0.4, -0.0575]),   # t1_a2, left plate
               'arm3': np.array([0.0, -0.4, -0.0575]),   # t2_a1, right plate
               'arm4': np.array([0.0, +0.4, -0.0575])}   # t2_a2, left plate
# Gen3 Lite 2F open pad gap, measured in sim (grasp_verify.txt): two grippers
# cannot occupy handles closer than this, whatever the kinematics say.
GRIPPER_WIDTH = 0.085


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


def handle_corners(centres, yaw, size):
    """4 handles on the corners of a `size` cube's TOP face, ordered around the
    face (h0 -> h1 -> h2 -> h3), so consecutive indices share an edge.

    Handles sit size/2 ABOVE the centre: `centres` stays the block's centre
    height (what the figure's z axis means) while the grasps are top-down on the
    upper face, matching grasp_orientation in the executor.
    """
    h = size / 2.0
    c, s = np.cos(yaw), np.sin(yaw)
    out = []
    for sx, sy in ((+1, +1), (-1, +1), (-1, -1), (+1, -1)):   # counter-clockwise
        dx, dy = sx * h, sy * h
        out.append(centres + np.array([c * dx - s * dy, s * dx + c * dy, h]))
    return np.stack(out)


def min_handle_gap(H):
    """Smallest distance between any two handles (checked on the first centre).

    The handle set is rigid, so this is a property of the LAYOUT, not of where
    the object sits -- which is why it is a one-off gate rather than a per-cell
    map: no arrangement of the gantries can push two handles further apart.
    """
    pts = H[:, 0, :]
    return min(float(np.linalg.norm(pts[i] - pts[j]))
               for i in range(len(pts)) for j in range(i + 1, len(pts)))


def seg_seg_dist(p1, p2, q1, q2):
    """Min distance between the segments p1-p2 and q1-q2, vectorised over (M,3).

    Clamped-parameter closest approach: solve the unconstrained line-line
    minimum, clamp to the segments, and re-solve one axis. Used as a stand-in
    for the arm bodies (see --min-link-clearance), not as exact link geometry.
    """
    d1, d2, r = p2 - p1, q2 - q1, p1 - q1
    a = (d1 * d1).sum(1)
    e = (d2 * d2).sum(1)
    b = (d1 * d2).sum(1)
    c = (d1 * r).sum(1)
    f = (d2 * r).sum(1)
    a_s = np.where(a > 1e-12, a, 1.0)
    e_s = np.where(e > 1e-12, e, 1.0)
    denom = a * e - b * b
    s = np.where(denom > 1e-12, (b * f - c * e) / np.where(denom > 1e-12, denom, 1.0), 0.0)
    s = np.clip(s, 0.0, 1.0)
    t = np.clip((b * s + f) / e_s, 0.0, 1.0)
    s = np.clip((b * t - c) / a_s, 0.0, 1.0)
    return np.linalg.norm((p1 + d1 * s[:, None]) - (q1 + d2 * t[:, None]), axis=1)


def arm_clearance(mount_a, h_a, mount_b, h_b):
    """Arm-crossing proxy for two arms on ONE gantry, all points gantry-local.

    Each arm is approximated by the straight segment shoulder-mount -> handle.
    When the two arms have to swap sides to reach their handles those segments
    intersect and this goes to ~0, which is the interference the position-only
    reach map cannot see.
    """
    m_a = np.broadcast_to(mount_a, h_a.shape)
    m_b = np.broadcast_to(mount_b, h_b.shape)
    return seg_seg_dist(m_a, h_a, m_b, h_b)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maps', default='data/maps')
    p.add_argument('--out', default='data/comanip')
    p.add_argument('--length', type=float, default=1.2,
                   help='beam length (line mode) or block edge (corners mode), m')
    p.add_argument('--handles', choices=('line', 'corners'), default='line',
                   help='4 handles along the object axis, or on its top-face corners')
    p.add_argument('--reach-radius', type=float, default=0.05,
                   help='handle counts as reachable within this distance of an FK sample')
    p.add_argument('--min-ee-dist', type=float, default=GRIPPER_WIDTH,
                   help='PROXY: layout gate -- two handles closer than this cannot '
                        'hold two grippers at once (default = measured open pad gap)')
    p.add_argument('--min-link-clearance', type=float, default=0.10,
                   help='PROXY: reject an arm pair whose mount->handle segments come '
                        'closer than this (arms cross). 0 disables the check.')
    p.add_argument('--fixed-arm-order', action='store_true',
                   help='ABLATION: do not search which arm takes which handle (test '
                        'only the first ordering). Shows what the crossing check '
                        'costs when the assignment is fixed rather than searched.')
    p.add_argument('--grid', type=float, default=0.05, help='object centre grid step (m)')
    p.add_argument('--z', type=float, nargs='+', default=[1.2, 1.4, 1.6])
    # a square block is yaw-symmetric every 90 deg, so 0/90 would be the SAME
    # map -- sample 0/45 instead (45 is the genuinely distinct corner geometry).
    p.add_argument('--yaw', type=float, nargs='+', default=None,
                   help='object yaws (rad); default 0,90deg for line, 0,45deg for corners')
    p.add_argument('--rail-step', type=float, default=0.2, help='gantry rail sweep step (m)')
    p.add_argument('--rot-step', type=float, default=0.4, help='gantry rotation sweep step (rad)')
    p.add_argument('--xlim', type=float, nargs=2, default=[-0.5, 2.5])
    p.add_argument('--ylim', type=float, nargs=2, default=[-1.2, 1.2])
    args = p.parse_args()
    if args.yaw is None:
        args.yaw = [0.0, np.pi / 4] if args.handles == 'corners' else [0.0, np.pi / 2]

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
    offs = np.linspace(-args.length / 2.0, args.length / 2.0, 4)
    # How the 4 handles split into two gantry pairs. `line`: gantry_1 takes the
    # two handles at one end, gantry_2 the other two -- the only sensible split
    # of a beam. `corners`: the two gantries take OPPOSITE EDGES of the top
    # face, and both edge partitions are searched, since which edge pair works
    # depends on the block's yaw and where it sits.
    PARTITIONS = ([((0, 1), (2, 3))] if args.handles == 'line'
                  else [((0, 1), (2, 3)), ((1, 2), (3, 0))])
    pairs = sorted({hp for part in PARTITIONS for hp in part})

    results = {}
    gate_done = False
    for z in args.z:
        for yaw in args.yaw:
            centres = np.column_stack([XX.ravel(), YY.ravel(), np.full(XX.size, z)])
            H = (handle_corners(centres, yaw, args.length) if args.handles == 'corners'
                 else handle_world(centres, yaw, offs))       # (4, M, 3)

            if not gate_done:      # layout gate: can 4 grippers coexist AT ALL?
                gate_done = True
                gap = min_handle_gap(H)
                print(f'\nlayout gate ({args.handles}, {args.length:.2f} m): min handle '
                      f'gap {gap:.3f} m vs min_ee_dist {args.min_ee_dist:.3f} m -> '
                      f'{"OK" if gap >= args.min_ee_dist else "IMPOSSIBLE"}')
                if gap < args.min_ee_dist:
                    raise SystemExit(
                        f'  two handles are only {gap:.3f} m apart: four grippers do '
                        f'not physically fit on this layout, so every cell would be '
                        f'infeasible for a reason that has nothing to do with reach. '
                        f'Use --handles corners for a small object.')
                print()

            # Which handle pair a gantry grips, and which of its two arms takes
            # which handle, decides feasibility outright -- so take the best over
            # all assignments instead of fixing one. (The RL policy later makes
            # this choice; here it must not be pre-empted.)
            # `raw` = kinematics only, `best` = kinematics AND the arms not
            # crossing, so the cost of the interference proxy is reportable.
            best, raw = {}, {}                     # (gantry, handle pair) -> mask
            for g in (0, 1):
                for hp in pairs:
                    ok_any = np.zeros(len(centres), dtype=bool)
                    ok_raw = np.zeros(len(centres), dtype=bool)
                    orders = ([GANTRY_ARMS[g]] if args.fixed_arm_order
                              else [GANTRY_ARMS[g], GANTRY_ARMS[g][::-1]])
                    for a_lo, a_hi in orders:
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
                                both = d2 < args.reach_radius
                                if not both.any():
                                    continue
                                hit = idx[m][both]
                                ok_raw[hit] = True
                                if args.min_link_clearance > 0.0:
                                    # both arms reach; do their bodies cross at
                                    # THIS gantry pose? Evaluated in the same
                                    # local frame the reach test just used.
                                    clr = arm_clearance(MOUNT_LOCAL[a_lo], l1[m][both],
                                                        MOUNT_LOCAL[a_hi], l2[both])
                                    hit = hit[clr >= args.min_link_clearance]
                                ok_any[hit] = True
                    best[(g, hp)] = ok_any
                    raw[(g, hp)] = ok_raw

            def combine(tbl):
                """Feasible = some partition assigns complementary pairs to the
                two gantries (either way round)."""
                out = np.zeros(len(centres), dtype=bool)
                for pa, pb in PARTITIONS:
                    out |= (tbl[(0, pa)] & tbl[(1, pb)]) | (tbl[(0, pb)] & tbl[(1, pa)])
                return out

            feas, feas_raw = combine(best), combine(raw)
            per_gantry = [np.logical_or.reduce([best[(g, hp)] for hp in pairs])
                          for g in (0, 1)]
            key = f'z{z:.2f}_yaw{yaw:.2f}'
            results[key] = feas.reshape(XX.shape)
            results[key + '_raw'] = feas_raw.reshape(XX.shape)
            results[key + '_g1'] = per_gantry[0].reshape(XX.shape)
            results[key + '_g2'] = per_gantry[1].reshape(XX.shape)
            lost = feas_raw.sum() - feas.sum()
            print(f'  {key}: gantry_1 {per_gantry[0].sum():5d} | gantry_2 {per_gantry[1].sum():5d} '
                  f'| BOTH {feas.sum():5d}/{len(feas)} ({100.0 * feas.mean():.1f}%), '
                  f'area {feas.sum() * args.grid ** 2:.3f} m^2'
                  + (f' | arm-crossing removed {lost} '
                     f'({100.0 * lost / max(feas_raw.sum(), 1):.1f}% of kinematic)'
                     if args.min_link_clearance > 0.0 else ''))

    np.savez(os.path.join(args.out, 'comanip_map.npz'),
             gx=gx, gy=gy, length=args.length, reach_radius=args.reach_radius,
             handles=args.handles, min_ee_dist=args.min_ee_dist,
             min_link_clearance=args.min_link_clearance, **results)
    print(f'\nsaved {os.path.join(args.out, "comanip_map.npz")}')


if __name__ == '__main__':
    main()
