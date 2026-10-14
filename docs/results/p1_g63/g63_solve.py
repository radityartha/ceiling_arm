#!/usr/bin/env python3
"""G63 solver (§A3): eye-to-hand extrinsics of both D455 from ChArUco corners on boards taped to arm fingers.

Unknowns: T_wc per camera (world -> camera optical, 6 DOF each) and T_lb per board (board in its finger link, 6 DOF
each, nuisance). Ground truth: T_wl per pose = FK from MEASURED joints (captured by g63_capture.py).
Observation: undistorted, normalised corner coordinates (x/z, y/z) per (pose, camera, board, corner id).
Plain least squares (scipy 'lm'), NO robust loss -- bad points must stay visible (CAM-6DOF A0.1 principle); poses with
RMS > 3x median are FLAGGED, never dropped.

Dataset JSON (g63_capture.py / synthetic):
  {"square_m": {"A": 0.05, "B": 0.05},
   "cameras": {"rgbd": {"K": 3x3, "dist": [...]}, ...},
   "poses": [{"id": "A-03", "board": "A", "role": "FIT"|"HOLDOUT"|"R0", "T_wl": 4x4,
              "obs": {"rgbd": {"ids": [...], "uv": [[u, v], ...]}, ...}}, ...]}

    python3 g63_solve.py --data ds.json --out result.json
    python3 g63_solve.py --self-test
"""
import argparse
import json
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

from g63_common import (CAMERAS, IMG_H, IMG_W, K_NOMINAL, T_from_vec, board_points, incumbent_T, inv, mean_T,
                        rot_deg, vec_from_T, xyzrpy_from_T)

# Observability flag: 1-sigma parameter uncertainty (from J^T J at the pixel noise of the fit) above these = the data
# does not pin that parameter down. Raised loudly; acceptance never proceeds on an unobservable solve.
OBS_STD_ROT_DEG = 0.10
OBS_STD_POS_M = 0.005
G6_CORNER_RMS_M = 0.002   # §A4 G6 (revised draft): lateral RMS residual of a later R0 under the first R0's T_lb


def _norm_obs(ds):
    """Undistort every observation once -> list of (pose_idx, cam, ids, xy_normalised, z_hint)."""
    out = []
    for i, p in enumerate(ds['poses']):
        for cam, o in p.get('obs', {}).items():
            if not o or not o['ids']:
                continue
            K = np.asarray(ds['cameras'][cam]['K'], float)
            dist = np.asarray(ds['cameras'][cam].get('dist', [0, 0, 0, 0, 0]), float)
            uv = np.asarray(o['uv'], float).reshape(-1, 1, 2)
            xy = cv2.undistortPoints(uv, K, dist).reshape(-1, 2)
            out.append((i, cam, np.asarray(o['ids'], int), xy, float(K[0, 0])))
    return out


class Problem:
    def __init__(self, ds, roles=('FIT',)):
        self.ds = ds
        self.cams = [c for c in CAMERAS if c in ds['cameras']]
        self.boards = sorted({p['board'] for p in ds['poses'] if p['role'] in roles})
        self.P = {b: board_points(b, ds.get('square_m', {}).get(b)) for b in self.boards}
        self.Twl = [np.asarray(p['T_wl'], float) for p in ds['poses']]
        self.obs = [o for o in _norm_obs(ds) if ds['poses'][o[0]]['role'] in roles]
        self.n_cam = len(self.cams)

    def unpack(self, x):
        Twc = {c: T_from_vec(x[6 * k:6 * k + 6]) for k, c in enumerate(self.cams)}
        Tlb = {b: T_from_vec(x[6 * (self.n_cam + k):6 * (self.n_cam + k) + 6]) for k, b in enumerate(self.boards)}
        return Twc, Tlb

    def pack(self, Twc, Tlb):
        return np.concatenate([vec_from_T(Twc[c]) for c in self.cams] + [vec_from_T(Tlb[b]) for b in self.boards])

    def residuals(self, x, obs=None):
        Twc, Tlb = self.unpack(x)
        r = []
        for i, cam, ids, xy, _f in (self.obs if obs is None else obs):
            b = self.ds['poses'][i]['board']
            Pb = np.array([self.P[b][k] for k in ids])
            Tcb = inv(Twc[cam]) @ self.Twl[i] @ Tlb[b]
            pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]
            r.append((pc[:, :2] / pc[:, 2:3] - xy).ravel())
        return np.concatenate(r) if r else np.zeros(0)


def pnp(P, ids, xy):
    """Board pose in camera from normalised points (IPPE, planar)."""
    obj = np.array([P[k] for k in ids], float)
    ok, rv, tv = cv2.solvePnP(obj, xy.reshape(-1, 1, 2), np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = tv.ravel()
    return T


def initial_guess(pb, Twc0):
    """T_wc = incumbent; T_lb = mean over images of inv(T_wl) T_wc0 T_cb (PnP)."""
    est = {b: [] for b in pb.boards}
    for i, cam, ids, xy, _f in pb.obs:
        if len(ids) < 4:
            continue
        b = pb.ds['poses'][i]['board']
        Tcb = pnp(pb.P[b], ids, xy)
        if Tcb is not None:
            est[b].append(inv(pb.Twl[i]) @ Twc0[cam] @ Tcb)
    missing = [b for b in pb.boards if not est[b]]
    if missing:
        raise SystemExit(f'INIT GAGAL: tidak ada PnP untuk papan {missing}')
    return {b: mean_T(v) for b, v in est.items()}


def solve(ds, Twc0=None, verbose=True):
    pb = Problem(ds, roles=('FIT',))
    if not pb.obs:
        raise SystemExit('tidak ada observasi FIT')
    Twc0 = Twc0 or {c: incumbent_T(c) for c in pb.cams}
    Tlb0 = initial_guess(pb, Twc0)
    x0 = pb.pack(Twc0, Tlb0)
    sol = least_squares(pb.residuals, x0, method='lm', x_scale='jac', max_nfev=20000)
    Twc, Tlb = pb.unpack(sol.x)
    # pixel-equivalent noise and parameter covariance
    n, m = sol.fun.size, sol.x.size
    sigma2 = float(sol.fun @ sol.fun) / max(n - m, 1)
    J = sol.jac
    try:
        cov = np.linalg.inv(J.T @ J) * sigma2
        std = np.sqrt(np.clip(np.diag(cov), 0, None))
        cond = float(np.linalg.cond(J))
    except np.linalg.LinAlgError:
        std, cond = np.full(m, np.inf), float('inf')
    unobs = []
    names = [f'{c}' for c in pb.cams] + [f'board_{b}' for b in pb.boards]
    for k, nm in enumerate(names[:pb.n_cam]):   # cameras only; board params are nuisance (std reported below)
        sr = np.degrees(std[6 * k:6 * k + 3]).max()
        st = std[6 * k + 3:6 * k + 6].max()
        if not (sr <= OBS_STD_ROT_DEG and st <= OBS_STD_POS_M):
            unobs.append(dict(param=nm, std_rot_deg=float(sr), std_pos_m=float(st)))
    # per-pose RMS in normalised units * depth -> metres lateral
    per_pose = pose_errors(pb, Twc, Tlb, pb.obs)
    rms = [v['rms_lat_m'] for v in per_pose]
    med = float(np.median(rms)) if rms else 0.0
    flagged = [v['pose'] for v in per_pose if v['rms_lat_m'] > 3 * med]
    res = dict(
        converged=bool(sol.success), status=int(sol.status), nfev=int(sol.nfev), cond=cond,
        sigma_norm=float(np.sqrt(sigma2)),
        Twc={c: Twc[c].tolist() for c in pb.cams},
        Twc_xyzrpy={c: [round(float(v), 5) for v in xyzrpy_from_T(Twc[c])] for c in pb.cams},
        Tlb={b: Tlb[b].tolist() for b in pb.boards},
        std={nm: dict(rot_deg=float(np.degrees(std[6 * k:6 * k + 3]).max()),
                      pos_m=float(std[6 * k + 3:6 * k + 6].max())) for k, nm in enumerate(names)},
        unobservable=unobs,
        vs_incumbent={c: dict(rot_deg=rot_deg(incumbent_T(c)[:3, :3], Twc[c][:3, :3]),
                              cam_pos_shift_m=float(np.linalg.norm(Twc[c][:3, 3] - incumbent_T(c)[:3, 3])))
                      for c in pb.cams},
        fit_pose_errors=per_pose, fit_flagged_3x_median=flagged,
    )
    if verbose and unobs:
        print(f'!!! PARAMETER TIDAK TERAMATI: {unobs}', file=sys.stderr)
    return res, Twc, Tlb


def pose_errors(pb, Twc, Tlb, obs):
    """Per (pose, camera): lateral corner error (m) = |normalised residual| * depth."""
    out = []
    for i, cam, ids, xy, _f in obs:
        b = pb.ds['poses'][i]['board']
        Pb = np.array([pb.P[b][k] for k in ids])
        Tcb = inv(Twc[cam]) @ pb.Twl[i] @ Tlb[b]
        pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]
        e = np.linalg.norm(pc[:, :2] / pc[:, 2:3] - xy, axis=1) * pc[:, 2]
        out.append(dict(pose=pb.ds['poses'][i]['id'], cam=cam, n=int(len(ids)), dist_m=float(np.median(pc[:, 2])),
                        rms_lat_m=float(np.sqrt(np.mean(e ** 2))), max_lat_m=float(e.max()), e=e.tolist()))
    return out


def evaluate_holdout(ds, Twc, Tlb):
    """G3 (lateral per camera / per zone) and G4 (two-camera triangulation vs FK) on HOLDOUT poses only."""
    pb = Problem(ds, roles=('HOLDOUT',))
    pb.boards = sorted(Tlb)
    pb.P = {b: board_points(b, ds.get('square_m', {}).get(b)) for b in pb.boards}
    per = pose_errors(pb, Twc, Tlb, pb.obs)
    g3 = {}
    for cam in pb.cams:
        e = np.concatenate([v['e'] for v in per if v['cam'] == cam]) if any(v['cam'] == cam for v in per) else []
        g3[cam] = dict(n=int(len(e)), median_m=float(np.median(e)) if len(e) else None,
                       p95_m=float(np.percentile(e, 95)) if len(e) else None)
    zone = {}
    for v in per:
        z = ds['poses'][[p['id'] for p in ds['poses']].index(v['pose'])].get('zone', '?')
        zone.setdefault((v['cam'], z), []).extend(v['e'])
    g3_zone = {f'{c}:{z}': float(np.median(e)) for (c, z), e in zone.items()}
    # G4: corners seen by both cameras in the same holdout pose -> midpoint triangulation
    by_pose = {}
    for i, cam, ids, xy, _f in pb.obs:
        by_pose.setdefault(i, {})[cam] = dict(zip(ids.tolist(), xy))
    tri = []
    for i, d in by_pose.items():
        if len(d) < 2:
            continue
        b = ds['poses'][i]['board']
        c1, c2 = sorted(d)[:2]
        for k in set(d[c1]) & set(d[c2]):
            o1, o2 = Twc[c1][:3, 3], Twc[c2][:3, 3]
            r1 = Twc[c1][:3, :3] @ np.r_[d[c1][k], 1.0]
            r2 = Twc[c2][:3, :3] @ np.r_[d[c2][k], 1.0]
            A = np.stack([r1, -r2], 1)
            s, t = np.linalg.lstsq(A, o2 - o1, rcond=None)[0]
            X = 0.5 * ((o1 + s * r1) + (o2 + t * r2))
            Xfk = (pb.Twl[i] @ Tlb[b] @ np.r_[pb.P[b][k], 1.0])[:3]
            tri.append(float(np.linalg.norm(X - Xfk)))
    g4 = dict(n=len(tri), median_m=float(np.median(tri)) if tri else None)
    # G4' (single camera): board centre from PnP with the NEW extrinsic vs FK centre (includes along-ray error)
    g4p = {}
    for i, cam, ids, xy, _f in pb.obs:
        if len(ids) < 6:
            continue
        b = ds['poses'][i]['board']
        Tcb = pnp(pb.P[b], ids, xy)
        if Tcb is None:
            continue
        c_b = np.r_[np.mean([pb.P[b][k] for k in range(12)], 0), 1.0]
        X = (Twc[cam] @ Tcb @ c_b)[:3]
        Xfk = (pb.Twl[i] @ Tlb[b] @ c_b)[:3]
        g4p.setdefault(cam, []).append(float(np.linalg.norm(X - Xfk)))
    g4p = {c: dict(n=len(v), median_m=float(np.median(v)), max_m=float(np.max(v))) for c, v in g4p.items()}
    return dict(G3=g3, G3_zone_median_m=g3_zone, G4=g4, G4p=g4p, holdout_pose_errors=per)


def drift_R0(ds, Twc):
    """G6: tape creep between R0 repeats. T_lb is fitted ONCE on the first R0 of each board (T_wc fixed); every later
    R0 is scored by its LATERAL residual (normalised residual x depth, RMS over corners, metres) under that frozen T_lb
    and its own measured FK. Works with ONE camera (a single-view 6-DOF refit is too noisy along the ray: ~7 mm at
    0.3 px, 1.5 m). Blind to a pure along-ray shift for that camera (documented limit)."""
    out, ref = [], {}
    for i, p in enumerate(ds['poses']):
        if p['role'] != 'R0':
            continue
        pb = Problem(dict(ds, poses=[dict(p, role='FIT')]), roles=('FIT',))
        if not pb.obs:
            out.append(dict(pose=p['id'], status='TIDAK TERLIHAT'))
            continue
        b = p['board']
        xc = np.concatenate([vec_from_T(Twc[c]) for c in pb.cams])
        if b not in ref:
            s0 = least_squares(lambda v: pb.residuals(np.r_[xc, v]), vec_from_T(initial_guess(pb, Twc)[b]),
                               method='lm')
            ref[b] = T_from_vec(s0.x)
        per = pose_errors(pb, Twc, {b: ref[b]}, pb.obs)
        e = np.concatenate([v['e'] for v in per])
        out.append(dict(pose=p['id'], board=b, cams=sorted({v['cam'] for v in per}),
                        lateral_rms_m=float(np.sqrt(np.mean(e ** 2))), lateral_max_m=float(e.max())))
    return out


# ------------------------------------------------------------------------------------------------ synthetic self-test
def _perturb(T, rot_deg_, pos_m, rng):
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax)
    d = rng.normal(size=3)
    d /= np.linalg.norm(d)
    Tp = T.copy()
    Tp[:3, :3] = T_from_vec(np.r_[np.radians(rot_deg_) * ax, 0, 0, 0])[:3, :3] @ T[:3, :3]
    Tp[:3, 3] = T[:3, 3] + pos_m * d
    return Tp


def _observe(truth_Twc, Twl, Tlb, P, K, px_noise, rng):
    """Project all 12 corners into each camera; keep a camera only if fully in frame (40 px) and <= 55 deg."""
    obs, vis = {}, 0
    for c in CAMERAS:
        Tcb = inv(truth_Twc[c]) @ Twl @ Tlb
        pc = np.array([P[i] for i in sorted(P)]) @ Tcb[:3, :3].T + Tcb[:3, 3]
        if (pc[:, 2] <= 0.3).any():
            continue
        m = pc.mean(0) / np.linalg.norm(pc.mean(0))
        if np.degrees(np.arccos(np.clip(np.dot(Tcb[:3, 2], m), -1, 1))) > 55:
            continue
        uv = pc[:, :2] / pc[:, 2:3] @ K[c][:2, :2].T + K[c][:2, 2]
        if (uv[:, 0] < 40).any() or (uv[:, 0] > IMG_W - 40).any() or (uv[:, 1] < 40).any() or \
                (uv[:, 1] > IMG_H - 40).any():
            continue
        obs[c] = dict(ids=sorted(P), uv=(uv + rng.normal(scale=px_noise, size=uv.shape)).tolist())
        vis += 1
    return obs, vis


def synth_dataset(rng, n_per_board=20, px_noise=0.0, fk_pos=0.0, fk_rot_deg=0.0, yaw_only=False,
                  holdout_every=4, n_r0=0, drift_after=None, r0_single=False):
    """Truth: cameras = incumbent perturbed 3 deg / 0.10 m; boards taped at an arbitrary offset on the link.
    Board poses face a chosen camera with +/-25 deg tilt about two axes (yaw_only: tilt 0 -> rotations about world z
    only = the degenerate design the negative control must flag)."""
    from scipy.spatial.transform import Rotation
    truth_Twc = {c: _perturb(incumbent_T(c), 3.0, 0.10, rng) for c in CAMERAS}
    truth_Tlb = {'A': T_from_vec(np.r_[0.3, -0.2, 1.4, 0.03, -0.10, 0.06]),
                 'B': T_from_vec(np.r_[-1.2, 0.4, 0.2, -0.02, 0.12, 0.04])}
    K = {c: K_NOMINAL[c] for c in CAMERAS}
    poses = []
    for b, (x0, x1, y0, y1) in {'A': (0.6, 1.8, -0.1, 0.7), 'B': (0.1, 1.2, -0.8, 0.1)}.items():
        P = board_points(b)
        k = 0
        tries = 0
        while k < n_per_board + n_r0 and tries < 20000:
            tries += 1
            is_r0 = k >= n_per_board
            c_face = (CAMERAS + ('both',))[k % 3] if not is_r0 else None
            pos = np.array([rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(0.85, 1.35)])
            if is_r0 and any(p['role'] == 'R0' and p['board'] == b for p in poses):
                # R0 repeat: same link pose, FRESH pixel noise; board possibly drifted (tape creep)
                p0 = next(p for p in poses if p['role'] == 'R0' and p['board'] == b)
                Tlb_now = truth_Tlb[b] if drift_after is None or k - n_per_board < drift_after \
                    else truth_Tlb[b] @ T_from_vec(np.r_[0, 0, 0, 0.003, 0, 0])
                poses.append(dict(p0, id=f'{b}-R0{k - n_per_board}',
                                  obs=_observe(truth_Twc, np.asarray(p0['T_wl']), Tlb_now, P, K, px_noise, rng)[0]))
                k += 1
                continue
            if is_r0 and r0_single:
                n = truth_Twc['rgbd'][:3, 3] - pos
            elif is_r0 or c_face == 'both':
                n = (truth_Twc['rgbd'][:3, 3] + truth_Twc['rgbd2'][:3, 3]) / 2 - pos
            else:
                n = truth_Twc[c_face][:3, 3] - pos
            n /= np.linalg.norm(n)
            z = -n                                  # board +z points away from the camera (OpenCV board frame)
            x = np.cross([0, 0, 1.0], z)
            x /= np.linalg.norm(x)
            R = np.stack([x, np.cross(z, x), z], 1)
            if yaw_only:   # fixed 35 deg tilt, rotated only about world z towards the camera
                az = np.arctan2(n[1], n[0])
                Rb = Rotation.from_euler('z', az).as_matrix() @ Rotation.from_euler('y', -np.radians(90 + 35)).as_matrix()
                Rb = Rb @ Rotation.from_euler('z', np.pi / 2).as_matrix()
            else:
                Rb = R @ Rotation.from_euler('xyz', rng.uniform(-25, 25, 3), degrees=True).as_matrix()
            Twb = np.eye(4)
            Twb[:3, :3], Twb[:3, 3] = Rb, pos
            Twl = Twb @ inv(truth_Tlb[b])
            obs, vis = _observe(truth_Twc, Twl, truth_Tlb[b], P, K, px_noise, rng)
            if (is_r0 and not r0_single and vis < 2) or vis == 0:
                continue
            Tmeas = _perturb(Twl, fk_rot_deg * abs(rng.normal()), fk_pos * abs(rng.normal()), rng) \
                if (fk_pos or fk_rot_deg) else Twl
            role = 'R0' if is_r0 else ('HOLDOUT' if (k // 3 + 1) % holdout_every == 0 else 'FIT')  # stratified per camera
            poses.append(dict(id=f'{b}-{k:02d}', board=b, role=role, zone=f'z{b}', T_wl=Tmeas.tolist(), obs=obs))
            k += 1
    ds = dict(square_m={'A': 0.05, 'B': 0.05}, poses=poses,
              cameras={c: dict(K=K[c].tolist(), dist=[0, 0, 0, 0, 0]) for c in CAMERAS})
    return ds, truth_Twc, truth_Tlb


def _err(Twc, truth):
    return {c: (rot_deg(truth[c][:3, :3], Twc[c][:3, :3]), float(np.linalg.norm(truth[c][:3, 3] - Twc[c][:3, 3])))
            for c in truth}


def self_test():
    ok = True
    rng = np.random.default_rng(63)
    print('T1a tanpa noise (batas 0.01 deg / 1 mm):')
    ds, tw, tl = synth_dataset(rng)
    r, Twc, Tlb = solve(ds, verbose=False)
    for c, (dr, dp) in _err(Twc, tw).items():
        good = dr <= 0.01 and dp <= 0.001
        ok &= good
        print(f'   {c}: rot {dr:.5f} deg, pos {dp * 1000:.3f} mm -> {"OK" if good else "GAGAL"}')
    ok &= not r['unobservable']
    print(f'   teramati semua: {not r["unobservable"]}  cond {r["cond"]:.1f}')

    print('T1b noise 0.3 px + FK 2 mm / 0.1 deg (batas 0.15 deg / 5 mm), + G3/G4 uji-tahan:')
    ds, tw, tl = synth_dataset(rng, px_noise=0.3, fk_pos=0.002, fk_rot_deg=0.1)
    r, Twc, Tlb = solve(ds, verbose=False)
    for c, (dr, dp) in _err(Twc, tw).items():
        good = dr <= 0.15 and dp <= 0.005
        ok &= good
        print(f'   {c}: rot {dr:.4f} deg, pos {dp * 1000:.2f} mm -> {"OK" if good else "GAGAL"}')
    h = evaluate_holdout(ds, Twc, Tlb)
    g3ok = all(v['median_m'] is not None for v in h['G3'].values()) and len(h['G3']) == 2
    ok &= g3ok
    g4ok = h['G4']['n'] > 0 and h['G4']['median_m'] <= 0.005
    g4pok = all(v['median_m'] <= 0.02 for v in h['G4p'].values()) and len(h['G4p']) == 2
    ok &= g4ok and g4pok
    print(f"   G4' PnP pusat vs FK: {json.dumps({c: (v['n'], round(v['median_m'] * 100, 2)) for c, v in h['G4p'].items()})}"
          f' (n, median cm) -> {"OK" if g4pok else "GAGAL"}')
    print(f'   G3 {json.dumps({c: (v["median_m"] and round(v["median_m"] * 100, 2), v["p95_m"] and round(v["p95_m"] * 100, 2)) for c, v in h["G3"].items()})}'
          f' cm (median, p95); G4 n {h["G4"]["n"]} median {h["G4"]["median_m"] and round(h["G4"]["median_m"] * 100, 2)} cm')
    n_fit = {c: sum(1 for p in ds['poses'] if p['role'] == 'FIT' and c in p['obs']) for c in CAMERAS}
    print(f'   pose FIT per kamera {n_fit}; std {json.dumps({k: (round(v["rot_deg"], 3), round(v["pos_m"] * 1000, 2)) for k, v in r["std"].items()})}')

    print('T2 kontrol negatif: rotasi papan hanya sekitar z world -> HARUS ditandai tidak teramati:')
    ds, tw, tl = synth_dataset(rng, px_noise=0.3, yaw_only=True)
    r, Twc, Tlb = solve(ds, verbose=False)
    flagged = bool(r['unobservable'])
    ok &= flagged
    print(f'   ditandai: {flagged} {[(u["param"], round(u["std_pos_m"] * 1000, 1)) for u in r["unobservable"]]}'
          f' -> {"OK" if flagged else "GAGAL (degenerasi diam)"}')

    # R0 capture = per-corner median of 30 frames -> 0.1 px (conservative vs 0.3/sqrt(30) = 0.055); 0.3 px also run.
    # False-alarm control over 3 seeds, detection of an injected 3 mm creep; both-camera AND single-camera R0.
    for single in (False, True):
        for px in (0.1, 0.3):
            for name, drift_after, want_pass in (('tanpa geser (3 seed)', None, True),
                                                 ('geser 3 mm disuntik sesudah R0 ke-2 (HARUS gagal)', 2, False)):
                worst_all, good = 0.0, True
                for seed in range(3 if drift_after is None else 1):
                    ds, tw, tl = synth_dataset(np.random.default_rng(630 + seed), px_noise=px, n_r0=4,
                                               drift_after=drift_after, r0_single=single)
                    r, Twc, Tlb = solve(ds, verbose=False)
                    dr = drift_R0(ds, Twc)
                    worst = max((d.get('lateral_rms_m', 0) for d in dr), default=0)
                    worst_all = max(worst_all, worst)
                    good &= len(dr) == 8 and (worst <= G6_CORNER_RMS_M) == want_pass and \
                        all('lateral_rms_m' in d for d in dr)
                ok &= good
                print(f'T3 R0 {"1 kamera" if single else "2 kamera"} {px} px, {name}: lateral RMS maks '
                      f'{worst_all * 1000:.2f} mm (gerbang {G6_CORNER_RMS_M * 1000:.0f}) -> {"OK" if good else "GAGAL"}')
    print('SELF-TEST', 'LULUS' if ok else 'GAGAL')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data')
    ap.add_argument('--out')
    ap.add_argument('--self-test', action='store_true')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.data or not a.out:
        ap.error('--data dan --out wajib (atau --self-test)')
    ds = json.load(open(a.data))
    res, Twc, Tlb = solve(ds)
    res['holdout'] = evaluate_holdout(ds, Twc, Tlb)
    res['R0_drift'] = drift_R0(ds, Twc)
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('converged', 'Twc_xyzrpy', 'vs_incumbent', 'unobservable')}, indent=1))
    print('G3', json.dumps(res['holdout']['G3']), 'G4', json.dumps(res['holdout']['G4']))
    return 0 if res['converged'] and not res['unobservable'] else 2


if __name__ == '__main__':
    sys.exit(main())
