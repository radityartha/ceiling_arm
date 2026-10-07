#!/usr/bin/env python3
"""G33 A2: build the frozen environment map OFFLINE from a capture_cloud.py file.

crop -> 2 cm voxels (seen in >= 2 frames) -> drop isolated (< 3 nbrs in 5 cm)
-> self-filter (52 URDF geometries at the MEASURED capture config, pad 0.05)
-> gantry corridor (structure swept over rail 0..1.6, any rotation, + pad)
-> env_static_map.npz

Also writes the (P) "filter OFF" counts and the platform-shell residual (A4).

    python3 build_env_map.py --capture capture_a.npz \
        --config ../p1_g32hw/g32hw_joint_states2.csv.gz --out env_static_map.npz
    python3 build_env_map.py --capture capture_a.npz capture_b.npz \
        --config ../p1_g32hw/g32hw_joint_states2.csv.gz capture --register ...
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
from env_collision import (CONFIG_JOINTS, ENV_MARGIN_M, RobotGeom,  # noqa: E402
                           point_distances)

CROP = dict(x=(-1.2, 3.0), y=(-2.0, 2.0), z=(0.30, 2.00))
RES = 0.02
MIN_FRAMES = 2
NB_R, NB_MIN = 0.05, 3
# gantry structure (URDF moving_table.urdf.xacro): base y +-0.36 z 2.05; plates at
# radius 0.4 (+0.055) about the platform centre; z span 1.9525 .. 2.0825.
G_Y = {'t1': 0.36, 't2': -0.36}
G_RAIL = (0.0, 1.6)
G_R = 0.455
G_PLAT = (0.175, 0.17)
G_Z = (1.9525, 2.0825)


def last_config(csv_gz):
    last = {}
    for r in csv.DictReader(gzip.open(csv_gz, 'rt')):
        last[r['joint']] = (float(r['t']), float(r['pos']))
    tmax = max(t for t, _ in last.values())
    stale = {j: tmax - t for j, (t, _) in last.items() if tmax - t > 5.0}
    if stale:
        raise ValueError(f'sendi basi > 5 s di {csv_gz}: {stale} -- TOLAK')
    return {j: p for j, (_, p) in last.items()}, tmax


def corridor_mask(c, pad):
    """True for voxels inside the gantry structure's swept volume (+pad)."""
    inz = (c[:, 2] >= G_Z[0] - pad) & (c[:, 2] <= G_Z[1] + pad)
    out = np.zeros(len(c), bool)
    for y0 in G_Y.values():
        dx = np.clip(c[:, 0], *G_RAIL) - c[:, 0]
        dy = c[:, 1] - y0
        disc = np.hypot(dx, dy) <= G_R + pad
        plat = (np.abs(dx) <= G_PLAT[0] + pad) & (np.abs(dy) <= G_PLAT[1] + pad)
        out |= inz & (disc | plat)
    return out


def _nearest_on_arms(rg, oMg, pts, arm_idx, dmax):
    """(mask, nearest, signed d) for points within dmax of an arm hull SURFACE,
    inside or outside (signed distance -> symmetric point-to-surface ICP)."""
    import coal
    probe = coal.Sphere(1e-6)
    req = coal.DistanceRequest()
    req.enable_nearest_points = True
    req.enable_signed_distance = True
    near = np.zeros_like(pts)
    best = np.full(len(pts), np.inf)
    for i in arm_idx:
        M = oMg[i]
        Tg = coal.Transform3s(M.rotation, M.translation)
        g = rg.geom.geometryObjects[i].geometry
        for k, p in enumerate(pts):
            r = coal.DistanceResult()
            d = coal.distance(probe, coal.Transform3s(np.eye(3), p), g, Tg, req, r)
            if abs(d) < abs(best[k]):
                best[k], near[k] = d, np.array(r.getNearestPoint2())
    return np.abs(best) < dmax, near, best


def register_to_robot(rg, q, pts, frames, n_per_arm=400, iters=12, seed=0):
    """4-DOF (translation + yaw about world z) correction camera-world -> URDF
    world, using the four arms standing still at the capture config as the
    target (B-over-A, docs/p1_g33_map.md B2). Returns (R, t, report)."""
    oMg = rg.place(q)
    rng = np.random.default_rng(seed)
    arms = ('t1_a1', 't1_a2', 't2_a1', 't2_a2')
    d0, _ = point_distances(rg, q, pts, 0.25, only=list(arms))
    sel_all = []
    for arm in arms:
        idx = np.nonzero((d0 <= 0.25) & (frames >= 3))[0]
        _, who = point_distances(rg, q, pts[idx], 0.25, only=[arm])
        idx = idx[who >= 0]
        if len(idx) > n_per_arm:
            idx = rng.choice(idx, n_per_arm, replace=False)
        sel_all.append(idx)
    sel = np.concatenate(sel_all)
    P = pts[sel]
    arm_idx = [i for i, n in enumerate(rg.names) if n[3] == 'a']
    R, t = np.eye(3), np.zeros(3)
    rep = {'n_pts': int(len(sel)), 'per_arm': [int(len(s)) for s in sel_all]}
    for it in range(iters):
        X = P @ R.T + t
        m, Y, d = _nearest_on_arms(rg, oMg, X, arm_idx, 0.25 if it < 3 else 0.12)
        if it == 0:
            rep['median_absd_before'] = float(np.median(np.abs(d[m])))
        A, B = X[m], Y[m]
        ca, cb = A.mean(0), B.mean(0)
        H = (A[:, :2] - ca[:2]).T @ (B[:, :2] - cb[:2])
        yaw = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        dt = cb - Rz @ ca
        R, t = Rz @ R, Rz @ t + dt
    X = P @ R.T + t
    m, _, d = _nearest_on_arms(rg, oMg, X, arm_idx, 0.25)
    rep['median_absd_after'] = float(np.median(np.abs(d[m])))
    rep['inliers_after'] = int(m.sum())
    rep['t'] = t.round(4).tolist()
    rep['yaw_deg'] = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    return R, t, rep


def ray_hits(rg, oMg, o, pts, pad):
    """True where the segment camera-origin -> point passes within `pad` of any
    robot geometry (slab-test prefilter on inflated AABBs, exact via coal)."""
    import coal
    v = pts - o
    L = np.linalg.norm(v, axis=1)
    hit = np.zeros(len(pts), bool)
    req = coal.DistanceRequest()
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / v
    for i, g in enumerate(rg.geom.geometryObjects):
        lo, hi = rg.world_aabb(i, oMg[i])
        lo, hi = lo - pad, hi + pad
        t1, t2 = (lo - o) * inv, (hi - o) * inv
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        cand = np.nonzero(~hit & (tmax >= np.maximum(tmin, 0)) & (tmin <= 1))[0]
        Tg = coal.Transform3s(oMg[i].rotation, oMg[i].translation)
        for k in cand:
            z = v[k] / L[k]
            a_ = np.cross([0, 0, 1.0], z)
            s, c = np.linalg.norm(a_), z[2]
            if s < 1e-9:
                R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
            else:
                K = np.array([[0, -a_[2], a_[1]], [a_[2], 0, -a_[0]], [-a_[1], a_[0], 0]]) / s
                R = np.eye(3) + s * K + (1 - c) * K @ K
            seg = coal.Capsule(1e-6, L[k])
            res = coal.DistanceResult()
            dk = coal.distance(seg, coal.Transform3s(R, o + v[k] / 2), g.geometry, Tg, req, res)
            if dk <= pad:
                hit[k] = True
    return hit


def capture_config(f, cfg_arg):
    """Robot config of one capture: 'capture' = the /joint_states it recorded itself
    (capture_cloud.py --joints), else a joint_states csv.gz (last row). Strict."""
    if cfg_arg == 'capture':
        cap = np.load(f)
        if 'joint_names' not in cap.files:
            raise KeyError(f'{f}: tidak merekam joint_states -- TOLAK')
        js = dict(zip([str(x) for x in cap['joint_names']], cap['joint_pos']))
        t = float(cap['t_end'])
    else:
        js, t = last_config(cfg_arg)
    miss = [j for j in CONFIG_JOINTS if j not in js]
    if miss:
        raise KeyError(f'{f}: konfigurasi tanpa {miss} -- TOLAK')
    return {j: float(js[j]) for j in CONFIG_JOINTS}, t


def capture_voxels(f, cfg, rg, a, rep):
    """One capture -> 2 cm voxels (centres, frames, seen_by) with ITS OWN robot
    config: register, crop, voxel, frames >= 2, isolation, self-filter, ray shadow."""
    q = rg.q_from(cfg)
    b = os.path.basename(f)
    cap = np.load(f)
    keys, frames, cams, origins = [], [], [], {}
    for ns in ('rgbd', 'rgbd2'):
        if f'{ns}_xyz' not in cap.files:
            raise KeyError(f'{f}: kamera {ns} tidak ada -- TOLAK')
        p, fr = cap[f'{ns}_xyz'], cap[f'{ns}_frames']
        o = cap[f'{ns}_tf'][:3]
        ci = 0 if ns == 'rgbd' else 1
        if a.register:
            m0 = np.ones(len(p), bool)
            for i, ax in enumerate('xyz'):
                m0 &= (p[:, i] >= CROP[ax][0]) & (p[:, i] <= CROP[ax][1])
            Rr, tr, rr = register_to_robot(rg, q, p[m0].astype(float), fr[m0])
            rep[f'{b}:{ns}_registration'] = rr
            print(b, ns, 'registrasi', rr, flush=True)
            if rr['median_absd_after'] > 0.03:
                raise ValueError(f'{b} {ns}: registrasi ke robot gagal ({rr}) -- TOLAK')
            p = (p.astype(float) @ Rr.T + tr).astype(np.float32)
            o = Rr @ o + tr
        origins[ci] = o
        m = np.ones(len(p), bool)
        for i, ax in enumerate('xyz'):
            m &= (p[:, i] >= CROP[ax][0]) & (p[:, i] <= CROP[ax][1])
        rep[f'{b}:{ns}_cells_1cm'] = int(len(p))
        rep[f'{b}:{ns}_cells_in_crop'] = int(m.sum())
        keys.append(np.floor(p[m] / RES).astype(np.int32))
        frames.append(fr[m])
        cams.append(np.full(m.sum(), ci, np.int8))
    K, F, C = np.concatenate(keys), np.concatenate(frames), np.concatenate(cams)
    uk, inv = np.unique(K, axis=0, return_inverse=True)
    vf = np.zeros(len(uk), np.int32)
    np.maximum.at(vf, inv, F)
    vc = np.zeros((len(uk), 2), bool)
    vc[inv, C] = True
    centers = (uk + 0.5) * RES
    rep[f'{b}:voxels_2cm'] = int(len(uk))

    keep = vf >= MIN_FRAMES
    rep[f'{b}:drop_frames_lt2'] = int((~keep).sum())
    centers, vc, vf = centers[keep], vc[keep], vf[keep]
    nn = cKDTree(centers).query_ball_point(centers, NB_R, return_length=True) - 1
    keep = nn >= NB_MIN
    rep[f'{b}:drop_isolated'] = int((~keep).sum())
    centers, vc, vf = centers[keep], vc[keep], vf[keep]

    # self-filter at THIS capture's measured config (strict). The screen sees each
    # voxel as a CUBE (coal OcTree), up to res*sqrt(3)/2 closer than its centre (B3).
    sh = max(0.15, a.self_pad + 0.10)
    d, who = point_distances(rg, q, centers, sh)
    near = d <= a.self_pad + RES * np.sqrt(3) / 2
    per = {}
    for i in np.unique(who[near]):
        n = rg.names[i]
        key = n[:5] if n[3] == 'a' else n.rsplit('_', 1)[0]
        per[key] = per.get(key, 0) + int(((who == i) & near).sum())
    rep[f'{b}:P_filter_off_removed'] = per
    rep[f'{b}:self_filter_removed'] = int(near.sum())
    struct = np.array([not rg.names[i][3] == 'a' if i >= 0 else False for i in who])
    rep[f'{b}:shell_struct_pad_to_%.2f' % sh] = int(((~near) & (d <= sh) & struct).sum())
    rep[f'{b}:shell_arm_pad_to_%.2f' % sh] = int(((~near) & (d <= sh) & ~struct & (who >= 0)).sum())
    centers, vc, vf = centers[~near], vc[~near], vf[~near]

    # ray shadow (B1): drop a camera's vote whose ray passes within ray_pad of the
    # robot; the voxel survives if the other camera saw it cleanly.
    if a.ray_pad > 0:
        shadow = np.zeros_like(vc)
        oMg = rg.place(q)
        for ci, o in sorted(origins.items()):
            idx = np.nonzero(vc[:, ci])[0]
            shadow[idx, ci] = ray_hits(rg, oMg, o, centers[idx], a.ray_pad)
        rep[f'{b}:ray_shadow_votes'] = [int(shadow[:, 0].sum()), int(shadow[:, 1].sum())]
        vc = vc & ~shadow
        gone = ~vc.any(1)
        rep[f'{b}:ray_shadow_removed'] = int(gone.sum())
        centers, vc, vf = centers[~gone], vc[~gone], vf[~gone]
    rep[f'{b}:voxels_after_filters'] = int(len(centers))
    return centers, vf, vc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True, nargs='+')
    # one per capture: a joint_states csv.gz, or 'capture' (recorded by capture_cloud --joints)
    ap.add_argument('--config', required=True, nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--pad', type=float, default=ENV_MARGIN_M)
    # B-over-A (docs/p1_g33_map.md B1/B3): arm surfaces land up to ~0.17 m off the
    # model (one camera each: registration residual + flying pixels at the arm edge)
    ap.add_argument('--self-pad', type=float, default=ENV_MARGIN_M)
    ap.add_argument('--ray-pad', type=float, default=0.0)
    # B-over-A (B2): camera-world vs URDF-world disagree by ~0.1 m; register each
    # camera's cloud to the four arms (4-DOF) before anything else.
    ap.add_argument('--register', action='store_true')
    a = ap.parse_args()
    if len(a.config) != len(a.capture):
        raise SystemExit('--config harus satu per --capture -- TOLAK')
    t0 = time.time()
    rep = {'captures': a.capture, 'configs': a.config, 'pad': a.pad, 'self_pad': a.self_pad,
           'ray_pad': a.ray_pad, 'res': RES,
           'crop': CROP, 'min_frames': MIN_FRAMES, 'nb': [NB_R, NB_MIN],
           'register': a.register}
    rg = RobotGeom()
    parts = []
    for f, c in zip(a.capture, a.config):
        cfg, tcfg = capture_config(f, c)
        rep[f'{os.path.basename(f)}:config'] = cfg
        rep[f'{os.path.basename(f)}:config_t'] = tcfg
        parts.append(capture_voxels(f, cfg, rg, a, rep))

    # union over captures: a voxel any capture kept (each already self/ray-filtered
    # with its own robot config). Occluded-in-one, seen-in-another fills in.
    C = np.concatenate([p[0] for p in parts])
    V = np.concatenate([p[2] for p in parts])
    uk, inv = np.unique(np.round(C / RES - 0.5).astype(np.int64), axis=0, return_inverse=True)
    vc = np.zeros((len(uk), 2), bool)
    np.logical_or.at(vc, inv, V)
    centers = (uk + 0.5) * RES
    rep['union_voxels'] = int(len(uk))

    cm = corridor_mask(centers, a.pad)
    rep['corridor_removed'] = int(cm.sum())
    centers, vc = centers[~cm], vc[~cm]

    rep['final_voxels'] = int(len(centers))
    rep['final_seen_by'] = {'rgbd_only': int((vc[:, 0] & ~vc[:, 1]).sum()),
                            'rgbd2_only': int((~vc[:, 0] & vc[:, 1]).sum()),
                            'both': int((vc[:, 0] & vc[:, 1]).sum())}
    rep['bbox'] = [centers.min(0).round(3).tolist(), centers.max(0).round(3).tolist()]
    rep['seconds'] = round(time.time() - t0, 1)
    np.savez_compressed(a.out, centers=centers.astype(np.float32), resolution=RES,
                        frame='world', seen_by=vc, pad=a.pad,
                        provenance=json.dumps(rep))
    with open(os.path.splitext(a.out)[0] + '_report.json', 'w') as fh:
        json.dump(rep, fh, indent=1)
    print(json.dumps({k: v for k, v in rep.items() if not k.endswith(':config')}, indent=1))


if __name__ == '__main__':
    main()
