#!/usr/bin/env python3
"""CAM-6DOF: per-camera 6-DOF extrinsic fit (M6 rigid; M7 = M6 + depth scale) from bottle tracks at arm-pointer plumb marks
(several surface heights) + depth points on the URDF arms (point-to-surface, near AND far arms). Offline, no ROS.

Camera model: p_world = R (s p_cam) + t, R = R_old Exp(w) (w in the camera frame), t = t_old + tau, s = 1 (M6) or free (M7).
Bottle obs: p_cam = R_old^T (track - t_old) (the track was made with the OLD extrinsic by a ONE-camera localizer); target =
mark xy + surface z + centroid offset of a cylinder (r, H) as seen from the camera (localizer = per-axis median of the
visible surface pixels -> biased toward the camera; modelled, see centroid_offset). Arm obs: depth pixels in a window
around each arm (0.40 m xy, z 1.15-1.45, chosen with the OLD extrinsic, max 4000), associated each iteration to the nearest
URDF hull surface point after that arm's best shift (grid +-0.25 / 1 cm at the start, then ICP, TOL 0.03); residual =
POINT-TO-PLANE n . (point - surface point), only where the surface is near-vertical (|n_z| <= 0.5): REST arms hang
vertically, so arms constrain world xy, not z (z comes from the bottles at 3 heights); the shift is used for association
only (sliding along a link is unobservable and must not be forced into the fit). Start = bottles-only M6 fit (association
of the far arms needs the right z), then bottles + arms. Weights LOCKED in §A: bottle sigma 0.015 m per 3-D obs; each arm counts K_ARM = 6
1-D obs at sigma 0.03 m. Plain weighted least squares (no robust loss: bad points must show, not hide).
Model selection LOCKED: leave-one-MARK-out on the FIT marks only; M7 iff LOO_M7 <= LOO_M6 - max(0.005, 0.15 LOO_M6).
Gate LOCKED: |w| <= 5 deg, |tau| <= 0.15 m, |s - 1| <= 0.06, converged.

    python3 cam6dof_fit.py cfg.json [--out fit.json]
cfg: {"ns": "rgbd2", "old": [x,y,z,roll,pitch,yaw], "bottle": {"r": .., "H": ..}, "capture": "rest.npz" | null,
      "obs": [{"name", "mark", "xy": [x,y], "surf_z", "track": [x,y,z], "role": "fit"|"holdout"}]}
    python3 cam6dof_fit.py --self-test
    python3 cam6dof_fit.py --tf-roundtrip "x y z roll pitch yaw"   (ROS domain 78, static_transform_publisher live)
"""
import argparse, json, os, subprocess, sys, time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

SIG_B, SIG_A, K_ARM, TOL = 0.015, 0.03, 6.0, 0.03
GATE_W_DEG, GATE_TAU, GATE_S = 5.0, 0.15, 0.06
ARMS = ('arm_1', 'arm_2', 'arm_3', 'arm_4')


# ---------- geometry ----------
def R_of(old): return Rotation.from_euler('xyz', old[3:]).as_matrix()


def to_world(x, R0, t0, P, model):
    s = np.exp(x[6]) if model == 'M7' else 1.0
    return (s * P) @ (R0 @ Rotation.from_rotvec(x[:3]).as_matrix()).T + t0 + x[3:6]


def wmedian(v, w):
    o = np.argsort(v); c = np.cumsum(w[o]); return v[o][np.searchsorted(c, 0.5 * c[-1])]


def centroid_offset(cam, base, r, H):
    """Per-axis median (pixel-weighted: area * cos(incidence) / range^2) of a cylinder's visible surface - base centre."""
    th = np.linspace(0, 2 * np.pi, 360, endpoint=False); h = (np.arange(40) + 0.5) / 40 * H
    T, Hh = np.meshgrid(th, h); nrm = np.stack([np.cos(T), np.sin(T), 0 * T], -1).reshape(-1, 3)
    side = base + np.c_[r * nrm[:, :2], Hh.ravel()]; a_side = np.full(len(side), r * 2 * np.pi / 360 * H / 40)
    rho = (np.arange(20) + 0.5) / 20 * r; Rr, Tt = np.meshgrid(rho, th)
    top = base + np.c_[(Rr * np.cos(Tt)).ravel(), (Rr * np.sin(Tt)).ravel(), np.full(Rr.size, H)]
    a_top = (Rr * (r / 20) * (2 * np.pi / 360)).ravel()
    P = np.r_[side, top]; N = np.r_[nrm, np.tile([0, 0, 1.0], (len(top), 1))]; A = np.r_[a_side, a_top]
    v = cam - P; d = np.linalg.norm(v, axis=1); c = (N * v).sum(1) / d; m = c > 0
    w = A[m] * c[m] / d[m] ** 2
    return np.array([wmedian(P[m, k], w) for k in range(3)]) - base


# ---------- arms ----------
def icp(W, tree, t, tol, it=10):
    for _ in range(it):
        d, i = tree.query(W - t, distance_upper_bound=tol); m = d <= tol
        if m.sum() < 10: break
        t = t + ((W[m] - t) - tree.data[i[m]]).mean(0)
    d, i = tree.query(W - t, distance_upper_bound=tol); m = d <= tol
    return t, m, i, d


def grid_shift(W, tree, tol):
    g = np.arange(-0.25, 0.2501, 0.01); best = (-1, np.zeros(3))
    for dx in g:
        for dy in g:
            t = np.array([dx, dy, 0.0]); k = int((tree.query(W - t, distance_upper_bound=tol)[0] <= tol).sum())
            if k > best[0]: best = (k, t)
    return icp(W, tree, best[1], tol)


def arm_sets(Pc, R0, t0, surf, nrm, axis, tag=''):
    """Window (OLD extrinsic) + incumbent gate. Gate: window >= 300, kept >= 30 %, kept >= 200, kept median <= 15 mm."""
    W0 = Pc @ R0.T + t0; out = {}
    for arm in ARMS:
        if arm not in surf: continue
        w = (np.hypot(W0[:, 0] - axis[arm][0], W0[:, 1] - axis[arm][1]) < 0.40) & (W0[:, 2] > 1.15) & (W0[:, 2] < 1.45)
        if w.sum() < 300: continue
        P = Pc[w][np.random.default_rng(0).permutation(int(w.sum()))[:4000]]; tree = cKDTree(surf[arm])
        t, m, i, d = grid_shift(P @ R0.T + t0, tree, TOL)
        rec = dict(n_window=int(w.sum()), n_used=len(P), kept=int(m.sum()), kept_frac=round(float(m.mean()), 3),
                   med_mm=round(float(np.median(d[m])) * 1000, 1) if m.any() else None, t_inc=t.round(4).tolist(),
                   t_inc_xy=round(float(np.hypot(*t[:2])), 4))
        rec['used'] = bool(rec['n_window'] >= 300 and rec['kept_frac'] >= 0.30 and rec['kept'] >= 200 and rec['med_mm'] <= 15)
        out[arm] = dict(P=P, tree=tree, N=nrm[arm], t=t, rec=rec)
        print(f'  {tag}{arm}: window {w.sum()} kept {m.sum()} ({m.mean():.0%}) med {rec["med_mm"]} mm  t_inc {t.round(3)}'
              f'  -> {"USED" if rec["used"] else "not used"}')
    return out


def deproject(depth, K, stride=2):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]; z = depth[v, u]; f = np.isfinite(z) & (z > 0.2) & (z < 6)
    u, v, z = u[f], v[f], z[f]
    return np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)


# ---------- fit ----------
def fit(bott, arms, R0, t0, model, iters=40):
    """bott: list of (p_cam, target_world). arms: dict from arm_sets (only used ones). Returns x (7,), info."""
    k = 7 if model == 'M7' else 6; x = np.zeros(7)
    if arms and len(bott) >= 4:
        x = fit(bott, {}, R0, t0, 'M6')[0]
    st = {}
    for a, v in arms.items():
        W = to_world(x, R0, t0, v['P'], model); st[a] = dict(t=grid_shift(W, v['tree'], TOL)[0], Wp=W)
    Pb = np.array([b[0] for b in bott]).reshape(-1, 3); Tb = np.array([b[1] for b in bott]).reshape(-1, 3)
    conv = False
    for it in range(iters):
        corr = []
        for a, v in arms.items():
            W = to_world(x, R0, t0, v['P'], model)
            t, m, i, _ = icp(W, v['tree'], st[a]['t'] + (W - st[a]['Wp']).mean(0), TOL)
            st[a].update(t=t, Wp=W)
            m &= np.abs(v['N'][np.minimum(i, len(v['N']) - 1)][:, 2]) <= 0.5
            if m.sum() >= 10: corr.append((v['P'][m], v['tree'].data[i[m]], v['N'][i[m]], np.sqrt(K_ARM / m.sum()) / SIG_A))

        def fun(xk):
            xx = np.r_[xk, np.zeros(7 - k)]
            r = [((to_world(xx, R0, t0, Pb, model) - Tb) / SIG_B).ravel()]
            r += [((to_world(xx, R0, t0, P, model) - Q) * N).sum(1) * w for P, Q, N, w in corr]
            return np.concatenate(r)
        sol = least_squares(fun, x[:k], x_scale=np.r_[[0.01] * 3, [0.01] * 3, [0.01] * (k - 6)])
        dx = np.abs(sol.x - x[:k]).max(); x[:k] = sol.x
        if it > 0 and dx < 1e-7: conv = True; break
    return x, dict(iters=it + 1, converged=conv, cost=float(sol.cost))


def describe(x, R0, t0, model):
    R = R0 @ Rotation.from_rotvec(x[:3]).as_matrix(); t = t0 + x[3:6]
    e = Rotation.from_matrix(R).as_euler('xyz')
    w_cam = np.degrees(x[:3]); w_world = R0 @ w_cam   # small-angle rotation axis*angle in world (deg)
    return dict(model=model, w_deg=round(float(np.linalg.norm(w_cam)), 4), w_cam_deg=w_cam.round(4).tolist(),
                w_world_deg=w_world.round(4).tolist(), tau=x[3:6].round(4).tolist(), tau_abs=round(float(np.linalg.norm(x[3:6])), 4),
                s=round(float(np.exp(x[6])), 5) if model == 'M7' else 1.0,
                new=np.r_[t, e].round(5).tolist(), quat_xyzw=Rotation.from_matrix(R).as_quat().round(7).tolist())


def gate(d, info):
    return bool(info['converged'] and d['w_deg'] <= GATE_W_DEG and d['tau_abs'] <= GATE_TAU and abs(d['s'] - 1) <= GATE_S)


def run(cfg, verbose=True):
    old = np.asarray(cfg['old'], float); R0, t0 = R_of(old), old[:3]; r, H = cfg['bottle']['r'], cfg['bottle']['H']
    obs = cfg['obs']
    for o in obs:
        o['pc'] = R0.T @ (np.asarray(o['track']) - t0); base = np.r_[o['xy'], o['surf_z']]
        o['off'] = centroid_offset(t0, base, r, H); o['target'] = base + o['off']
    arms = cfg.get('_arms')
    if arms is None:
        arms = {}
        if cfg.get('capture'):
            cap = np.load(cfg['capture']); ns = cfg['ns']
            tfc = cap[f'{ns}_tf']; d_tf = np.linalg.norm(tfc[:3] - t0)
            d_rot = np.degrees((Rotation.from_quat(tfc[3:]) * Rotation.from_matrix(R0).inv()).magnitude())
            print(f'capture TF vs old: {d_tf * 1000:.2f} mm {d_rot:.4f} deg' + ('  !! MISMATCH' if d_tf > 0.001 or d_rot > 0.01 else ''))
            arms = arm_sets(deproject(cap[f'{ns}_depth'], cap[f'{ns}_K']), R0, t0,
                            {a: cap[f'{a}_surf'] for a in ARMS}, {a: cap[f'{a}_nrm'] for a in ARMS},
                            {a: cap[f'{a}_axis'] for a in ARMS}, ns + ':')
    used = {a: v for a, v in arms.items() if v['rec']['used']}
    F = [o for o in obs if o['role'] == 'fit']; Ho = [o for o in obs if o['role'] == 'holdout']
    rep = dict(ns=cfg.get('ns'), bottle=cfg['bottle'], arms={a: v['rec'] for a, v in arms.items()}, models={})
    for model in ('M6', 'M7'):
        x, info = fit([(o['pc'], o['target']) for o in F], used, R0, t0, model)
        d = describe(x, R0, t0, model); d.update(info, gate=gate(d, info))
        rows = {}
        for o in obs:
            p = to_world(x, R0, t0, o['pc'][None], model)[0]
            rows[o['name']] = dict(role=o['role'], mark=o['mark'], surf_z=o['surf_z'], off=o['off'].round(4).tolist(),
                                   track_old=np.round(o['track'], 4).tolist(), pred=p.round(4).tolist(),
                                   dxy_old=round(float(np.hypot(*(np.asarray(o['track'][:2]) - o['xy']))), 4),
                                   dxy_new=round(float(np.hypot(*(p[:2] - o['xy']))), 4),
                                   dz_new=round(float(p[2] - o['target'][2]), 4), res3=round(float(np.linalg.norm(p - o['target'])), 4))
        d['rows'] = rows
        d['fit_rms'] = round(float(np.sqrt(np.mean([rows[o['name']]['res3'] ** 2 for o in F]))), 4) if F else None
        arm_after = {}
        for a, v in arms.items():
            W = to_world(x, R0, t0, v['P'], model); t, m, _, dd = grid_shift(W, v['tree'], TOL)
            arm_after[a] = dict(t=t.round(4).tolist(), t_xy=round(float(np.hypot(*t[:2])), 4), kept_frac=round(float(m.mean()), 3),
                                used=v['rec']['used'])
        d['arms_after'] = arm_after
        # LOO over FIT marks (holdout never touched)
        marks = sorted({o['mark'] for o in F}); se = []
        for mk in marks:
            tr = [o for o in F if o['mark'] != mk]; te = [o for o in F if o['mark'] == mk]
            xl, _ = fit([(o['pc'], o['target']) for o in tr], used, R0, t0, model)
            se += [float(np.sum((to_world(xl, R0, t0, o['pc'][None], model)[0] - o['target']) ** 2)) for o in te]
        d['loo_rms'] = round(float(np.sqrt(np.mean(se))), 4) if se else None
        rep['models'][model] = d
        if verbose:
            print(f"{model}: |w| {d['w_deg']:.3f} deg (cam {d['w_cam_deg']}) tau {d['tau']} s {d['s']}  fit RMS {d['fit_rms']}"
                  f"  LOO {d['loo_rms']}  gate {'OK' if d['gate'] else 'FAIL'}  iters {info['iters']} conv {info['converged']}")
            for k, rw in rows.items():
                print(f"   {k:10s} {rw['role']:7s} dxy old {rw['dxy_old']:.3f} -> new {rw['dxy_new']:.3f}  dz {rw['dz_new']:+.3f}  res3 {rw['res3']:.3f}")
            for a, aa in arm_after.items():
                print(f"   arm {a} {'(used)' if aa['used'] else '(info)'} t_inc {arms[a]['rec']['t_inc_xy']:.3f} -> t {aa['t_xy']:.3f} kept {aa['kept_frac']:.0%}")
    l6, l7 = rep['models']['M6']['loo_rms'], rep['models']['M7']['loo_rms']
    rep['select'] = 'M7' if (l6 is not None and l7 is not None and l7 <= l6 - max(0.005, 0.15 * l6)) else 'M6'
    sel = rep['models'][rep['select']]; ix = '1' if cfg.get('ns') == 'rgbd' else '2'
    rep['args_M6'] = ' '.join(f'tf{ix}_{k}:={v:.5f}' for k, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), rep['models']['M6']['new']))
    if verbose: print(f"SELECT {rep['select']} (LOO M6 {l6} M7 {l7}); deployable args (M6, static TF): {rep['args_M6']}")
    return rep


# ---------- self-test ----------
def _synth(true_w_cam_deg, true_tau, s_true, noise_b=0.005, noise_a=0.003, seed=1):
    rng = np.random.default_rng(seed)
    old = np.array([-0.1547, -1.0197, 2.0905, -2.2369, -0.1285, -0.9189]); R0, t0 = R_of(old), old[:3]
    Rt = R0 @ Rotation.from_rotvec(np.radians(true_w_cam_deg)).as_matrix(); tt = t0 + true_tau
    r, H = 0.03, 0.07
    marks = dict(M1=(1.7497, 0.5493), C5=(1.6506, 0.6503), N1=(1.9389, 0.4984), N2=(1.9388, 0.6806), C6=(1.9004, 0.6004),
                 C3=(1.8486, 0.6786))
    obs = []
    for mk, xy in marks.items():
        for hz in (0.77, 0.92, 1.12):
            c = np.r_[xy, hz] + centroid_offset(t0, np.r_[xy, hz], r, H)   # same centroid model as the fit (solver test)
            pc = Rt.T @ (c - tt) / s_true + rng.normal(0, noise_b, 3)
            obs.append(dict(name=f'{mk}@{hz}', mark=mk, xy=list(xy), surf_z=hz, track=list(R0 @ pc + t0),
                            role='holdout' if mk in ('C6', 'C3') else 'fit'))
    surf, nrms, axis, Pc = {}, {}, {}, []
    for arm, (ax, ay) in dict(arm_1=(1.8993, 0.36), arm_2=(1.0993, 0.36), arm_3=(0.40, -0.36)).items():
        th = np.linspace(0, 2 * np.pi, 120, endpoint=False); z = np.arange(1.0, 1.6, 0.005)
        T, Z = np.meshgrid(th, z); vert = np.c_[ax + 0.045 * np.cos(T).ravel(), ay + 0.045 * np.sin(T).ravel(), Z.ravel()]
        xh = np.arange(0, 0.3, 0.005); T2, X2 = np.meshgrid(th, xh)   # horizontal forearm at z 1.30 toward +x
        hor = np.c_[ax + X2.ravel(), ay + 0.04 * np.cos(T2).ravel(), 1.30 + 0.04 * np.sin(T2).ravel()]
        S = np.r_[vert, hor]; surf[arm] = S; axis[arm] = np.array([ax + 0.1, ay])
        nrm = np.r_[np.c_[np.cos(T).ravel(), np.sin(T).ravel(), 0 * T.ravel()],
                    np.c_[0 * T2.ravel(), np.cos(T2).ravel(), np.sin(T2).ravel()]]
        nrms[arm] = nrm; vis = ((tt - S) * nrm).sum(1) > 0
        Pc.append((S[vis] - tt) @ Rt / s_true + rng.normal(0, noise_a, (vis.sum(), 3)))
    arms = arm_sets(np.concatenate(Pc), R0, t0, surf, nrms, axis, 'synth:')
    return dict(ns='rgbd2', old=old.tolist(), bottle=dict(r=r, H=H), obs=obs, _arms=arms), Rt, tt


def planar_yaw_only(cfg):
    """Negative control: 3-DOF world planar (yaw + xy) on FIT bottles, as RGBD2-ZONE. Holdout judged in 3-D (acceptance has z)."""
    old = np.asarray(cfg['old']); R0, t0 = R_of(old), old[:3]
    F = [o for o in cfg['obs'] if o['role'] == 'fit']
    src = np.array([o['track'][:2] for o in F]); dst = np.array([o['target'][:2] for o in F])
    cs, cd = src.mean(0), dst.mean(0); Hm = (src - cs).T @ (dst - cd); th = np.arctan2(Hm[0, 1] - Hm[1, 0], Hm[0, 0] + Hm[1, 1])
    R2 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); t2 = cd - R2 @ cs
    ho = [(float(np.linalg.norm(R2 @ np.asarray(o['track'][:2]) + t2 - o['target'][:2])), abs(o['track'][2] - o['target'][2]))
          for o in cfg['obs'] if o['role'] == 'holdout']
    return np.degrees(th), max(h[0] for h in ho), max(h[1] for h in ho)


def _check(rep, model, Rt, tt):
    m = rep['models'][model]; Rn = Rotation.from_quat(m['quat_xyzw']).as_matrix()
    er = np.degrees(Rotation.from_matrix(Rn @ Rt.T).magnitude()); ep = np.linalg.norm(np.asarray(m['new'][:3]) - tt)
    ho = max(r['res3'] for r in m['rows'].values() if r['role'] == 'holdout')
    return m, er, ep, ho


def self_test():
    ok = True; W0, T0 = [2.5, 0, 0], np.array([0.02, -0.01, 0.01])
    print('== T1a: tilt 2.5 deg about camera x + tau (0.02, -0.01, 0.01), s = 1, NO noise -> exact')
    cfg, Rt, tt = _synth(W0, T0, 1.0, noise_b=0, noise_a=0); rep = run(cfg, verbose=False); m, er, ep, ho = _check(rep, 'M6', Rt, tt)
    a = er < 0.01 and ep < 0.001 and ho < 0.001 and rep['select'] == 'M6' and m['gate']
    print(f"   M6 rot err {er:.4f} deg pos err {ep * 1000:.2f} mm holdout max {ho * 1000:.2f} mm; select {rep['select']}  {'PASS' if a else 'FAIL'}")
    ok &= a
    print('== T1b: same + noise bottle 5 mm / arm 3 mm (tolerance = noise floor: rot 0.25 deg, pos 15 mm, holdout 12 mm)')
    cfg, Rt, tt = _synth(W0, T0, 1.0); rep = run(cfg, verbose=False); m, er, ep, ho = _check(rep, 'M6', Rt, tt)
    a = er < 0.25 and ep < 0.015 and ho < 0.012 and rep['select'] == 'M6' and m['gate']
    print(f"   M6 rot err {er:.4f} deg pos err {ep * 1000:.1f} mm holdout max {ho * 1000:.1f} mm; M7 s {rep['models']['M7']['s']};"
          f" LOO M6 {m['loo_rms']} M7 {rep['models']['M7']['loo_rms']} -> select {rep['select']}  {'PASS' if a else 'FAIL'}")
    ok &= a
    th, hxy, hz = planar_yaw_only(cfg)
    b = max(hxy, hz) > 0.03 and ho < 0.012
    print(f"== T3 negative control yaw-only on T1b data: yaw {th:.2f} deg, holdout max xy {hxy * 1000:.0f} mm z {hz * 1000:.0f} mm "
          f"(M6 3-D {ho * 1000:.0f} mm) -> yaw-only does {'NOT ' if b else ''}close the tilt  {'PASS' if b else 'FAIL'}")
    ok &= b
    print('== T2: tilt 2.5 deg + depth scale 1.04 (true p_world = R (1.04 p_meas) + t), noise as T1b')
    cfg, Rt, tt = _synth(W0, T0, 1.04); rep = run(cfg, verbose=False); m, er, ep, ho = _check(rep, 'M7', Rt, tt)
    c = abs(m['s'] - 1.04) < 0.005 and er < 0.25 and ep < 0.015 and rep['select'] == 'M7' and m['gate']
    print(f"   M7 s {m['s']} rot err {er:.4f} deg pos err {ep * 1000:.1f} mm holdout max {ho * 1000:.1f} mm; LOO M6 "
          f"{rep['models']['M6']['loo_rms']} M7 {m['loo_rms']} -> select {rep['select']}  {'PASS' if c else 'FAIL'}")
    ok &= c
    print('== T4: Euler args (5 dp) -> R round trip vs fitted R')
    e = np.asarray(m['new']); Rr = Rotation.from_euler('xyz', np.round(e[3:], 5)).as_matrix()
    d = np.degrees(Rotation.from_matrix(Rr @ Rotation.from_quat(m['quat_xyzw']).as_matrix().T).magnitude())
    print(f'   {d:.5f} deg  {"PASS" if d < 0.01 else "FAIL"}'); ok &= d < 0.01
    print('== centroid model (r 0.03, H 0.07) from rgbd2 at table M1 / block 35:',
          centroid_offset(np.array([-0.1547, -1.0197, 2.0905]), np.r_[1.75, 0.55, 0.77], 0.03, 0.07).round(4),
          centroid_offset(np.array([-0.1547, -1.0197, 2.0905]), np.r_[1.75, 0.55, 1.12], 0.03, 0.07).round(4))
    print('SELF-TEST', 'PASS' if ok else 'FAIL'); return ok


def tf_roundtrip(e):
    """Live: static_transform_publisher (binary, not the ros2 run wrapper) in ROS domain 78 -> tf2 lookup == args."""
    env = dict(os.environ, ROS_DOMAIN_ID='78')
    a = [f'{v:.5f}' for v in e]
    p = subprocess.Popen(['/opt/ros/humble/lib/tf2_ros/static_transform_publisher', '--x', a[0], '--y', a[1], '--z', a[2],
                          '--roll', a[3], '--pitch', a[4], '--yaw', a[5], '--frame-id', 'world', '--child-frame-id', 'rt_optical'],
                         env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        q = subprocess.run([sys.executable, '-c', '''
import rclpy, time
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node('rt'); b = Buffer(); TransformListener(b, n); t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
    if b.can_transform('world', 'rt_optical', Time()):
        tr = b.lookup_transform('world', 'rt_optical', Time()).transform
        print(tr.translation.x, tr.translation.y, tr.translation.z, tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w); break
'''], env=env, capture_output=True, text=True, timeout=30).stdout.split()
    finally:
        p.terminate(); p.wait(5)
    v = np.array(q, float); dt = np.linalg.norm(v[:3] - np.asarray(e[:3]))
    dr = np.degrees((Rotation.from_quat(v[3:]) * Rotation.from_euler('xyz', np.round(e[3:], 5)).inv()).magnitude())
    ok = dt <= 0.001 and dr <= 0.01
    print(f'TF live vs args: {dt * 1000:.3f} mm {dr:.5f} deg  {"PASS" if ok else "FAIL"}'); return ok


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('cfg', nargs='?'); ap.add_argument('--out')
    ap.add_argument('--self-test', action='store_true'); ap.add_argument('--tf-roundtrip'); a = ap.parse_args()
    if a.self_test: sys.exit(0 if self_test() else 1)
    if a.tf_roundtrip: sys.exit(0 if tf_roundtrip([float(v) for v in a.tf_roundtrip.split()]) else 1)
    rep = run(json.load(open(a.cfg)))
    if a.out: json.dump(rep, open(a.out, 'w'), indent=1)
