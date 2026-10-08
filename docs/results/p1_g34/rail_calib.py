#!/usr/bin/env python3
"""G34 step 1: per-capture, per-camera, PER-GANTRY registration camera-world -> URDF.

G33 B9: one rigid correction per capture differs between gantry positions (A vs B).
Registering each gantry's arm pair alone, at its own rail position, shows how the
camera sees each rail: offset(t_g, yaw_g) as a function of the encoder rail s_g.

    python3 rail_calib.py --capture ../p1_g33/capture_a.npz c1.npz ... \
        --config ../p1_g32hw/g32hw_joint_states2.csv.gz capture ... --out rail_calib.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g33'))
from build_env_map import (CROP, _nearest_on_arms, capture_config, icp_to_robot,  # noqa: E402
                           register_to_robot, select_arm_points)
from env_collision import RobotGeom  # noqa: E402

GROUPS = {'g1': ('t1_a1', 't1_a2'), 'g2': ('t2_a1', 't2_a2'),
          'all': ('t1_a1', 't1_a2', 't2_a1', 't2_a2')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True, nargs='+')
    ap.add_argument('--config', required=True, nargs='+')
    ap.add_argument('--groups', nargs='+', default=['g1', 'g2', 'all'])
    ap.add_argument('--out', required=True)
    # one rigid transform per camera over ALL captures (dof 6), then the residual
    # per capture x gantry: is the offset the camera's (one fix) or the rail's?
    ap.add_argument('--joint', type=int, choices=(4, 6))
    # G34 B: static scenery under a moved gantry (work table ~0.15 m below the REST
    # grippers at x ~0.9) gets picked as "arm" points and drags the fit down.
    # Background subtraction: drop cells also seen in the capture where THIS
    # gantry is farthest away (>= 0.4 m) -- the scenery cancels, the arms stay.
    ap.add_argument('--diff', action='store_true')
    a = ap.parse_args()
    if len(a.capture) != len(a.config):
        raise SystemExit('--config harus satu per --capture -- TOLAK')
    rg = RobotGeom()
    if a.joint:
        return joint(rg, a)
    rows = json.load(open(a.out)) if os.path.exists(a.out) else []
    done = {(r['capture'], r['cam'], r['group']) for r in rows}
    cfgs = {f: capture_config(f, c)[0] for f, c in zip(a.capture, a.config)}
    for f in a.capture:
        cfg = cfgs[f]
        q = rg.q_from(cfg)
        b = os.path.basename(f)
        for ns in ('rgbd', 'rgbd2'):
            p0, fr0 = load_cam(f, ns)
            for g in a.groups:
                if (b, ns, g) in done:
                    continue
                p, fr, ref = p0, fr0, None
                if a.diff:
                    if g == 'all':
                        continue
                    lj = f't{g[1]}_linear_joint'
                    ref = max(a.capture, key=lambda o: abs(cfgs[o][lj] - cfg[lj]))
                    if abs(cfgs[ref][lj] - cfg[lj]) < 0.4:
                        raise SystemExit(f'{b} {g}: tidak ada acuan >= 0.4 m -- TOLAK')
                    pr, frr = load_cam(ref, ns)
                    keep = background_mask(p, pr[frr >= 2])
                    p, fr = p[keep], fr[keep]
                _, _, rr = register_to_robot(rg, q, p, fr, arms=GROUPS[g])
                row = dict(capture=b, cam=ns, group=g, diff_ref=ref and os.path.basename(ref),
                           s1=cfg['t1_linear_joint'], s2=cfg['t2_linear_joint'],
                           r1=cfg['t1_rotation_joint'], r2=cfg['t2_rotation_joint'], **rr)
                rows.append(row)
                print(json.dumps(row), flush=True)
                json.dump(rows, open(a.out, 'w'), indent=1)


def background_mask(p, ref, cell=0.01):
    """True for points whose 1 cm cell (+-1 cell) is NOT occupied in `ref`."""
    occ = set(map(tuple, np.floor(ref / cell).astype(np.int64)))
    k = np.floor(p / cell).astype(np.int64)
    hit = np.zeros(len(p), bool)
    for d in np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])).reshape(3, -1).T:
        hit |= np.fromiter((tuple(x) in occ for x in k + d), bool, len(k))
    return ~hit


def load_cam(f, ns):
    cap = np.load(f)
    p, fr = cap[f'{ns}_xyz'].astype(float), cap[f'{ns}_frames']
    m = np.ones(len(p), bool)
    for i, ax in enumerate('xyz'):
        m &= (p[:, i] >= CROP[ax][0]) & (p[:, i] <= CROP[ax][1])
    return p[m], fr[m]


def joint(rg, a):
    out = {}
    for ns in ('rgbd', 'rgbd2'):
        items = []
        cfgs = {f: capture_config(f, c)[0] for f, c in zip(a.capture, a.config)}
        for f in a.capture:
            cfg = cfgs[f]
            q = rg.q_from(cfg)
            p0, fr0 = load_cam(f, ns)
            for g in ('g1', 'g2'):
                p, fr = p0, fr0
                if a.diff:
                    lj = f't{g[1]}_linear_joint'
                    ref = max(a.capture, key=lambda o: abs(cfgs[o][lj] - cfg[lj]))
                    if abs(cfgs[ref][lj] - cfg[lj]) < 0.4:
                        raise SystemExit(f'{f} {g}: tidak ada acuan >= 0.4 m -- TOLAK')
                    pr, frr = load_cam(ref, ns)
                    keep = background_mask(p, pr[frr >= 2])
                    p, fr = p[keep], fr[keep]
                P, per = select_arm_points(rg, q, p, fr, GROUPS[g], 300)
                if min(per) < 300:
                    # one-arm sets leave the pair's pose underdetermined (B, rail_table)
                    P = P[:0]
                # COPY: rg.place returns pinocchio's internal oMg, overwritten by the
                # next place() -- without it every capture got the LAST pose (G34 B).
                items.append((os.path.basename(f), g, cfg,
                              [pin.SE3(M) for M in rg.place(q)], P, per))
        R, t, rep = icp_to_robot(rg, [(o, P) for *_, o, P, _ in items if len(P)],
                                 GROUPS['all'], dof=a.joint)
        print(ns, json.dumps(rep), flush=True)
        res = []
        for b, g, cfg, oMg, P, per in items:
            if len(P) < 50:
                res.append(dict(capture=b, group=g, n=len(P), per_arm=per, skip='< 50 titik'))
                continue
            aidx = [i for i, n in enumerate(rg.names) if n[:5] in GROUPS[g]]
            X = P @ R.T + t
            m, Y, d = _nearest_on_arms(rg, oMg, X, aidx, 0.12)
            r = dict(capture=b, group=g, s1=round(cfg['t1_linear_joint'], 4),
                     s2=round(cfg['t2_linear_joint'], 4), n=int(m.sum()), per_arm=per,
                     median_absd=round(float(np.median(np.abs(d[m]))), 4),
                     # mean point->surface vector: what is LEFT for this gantry here
                     resid=(Y[m] - X[m]).mean(0).round(4).tolist(),
                     centroid=X[m].mean(0).round(3).tolist())
            res.append(r)
            print(' ', json.dumps(r), flush=True)
        out[ns] = dict(fit=rep, R=R.round(6).tolist(), t=t.round(5).tolist(), per=res)
    json.dump(out, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
