#!/usr/bin/env python3
"""G63e tugas 2: uji mandiri sintetis M3 (penanda ArUco statis + papan polos genggam) -> ambang G3/G5 SEBELUM data.

Kebenaran: kamera = incumbent diganggu (rotasi acak ROT_DEG, geser TRANS_M); prior detektor & awal solver = incumbent (seperti
HW). Penanda benar = rencana; pita = tiap sudut + galat (xy SIG_XY, z SIG_Z) -> Procrustes. Papan genggam: sisi BENAR SQ_TRUE,
solver mulai SQ_START (PDF). Citra: penanda (lembar penuh / inti untuk vertikal) + papan (tepi putih 21 mm), render
g63e_render (blur 1.3-1.6, noise 2.3, latar frame lama), N_FRAMES frame per kamera untuk penanda (median), 1 frame per pose papan
per kamera. Pose papan: pusat x 0.1-1.5, y -0.6-0.7, z 0.4-1.1, normal ke bisektor dua kamera +-15 deg, kedua kamera: semua sudut
di bingkai (tepi 40 px), kotak >= 9.5 px, jarak <= 2.2 m, sudut <= 55 deg, tidak kena masker lengan REST.
Mode: nominal | neg3cm (satu penanda terlihat diberi geser pita 3 cm horizontal: G3 HARUS menandainya).

    python3 g63e_selftest.py --plan g63e_plan.json --seeds 20 [--mode both] [--out selftest.json]
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402
import g63e_detect as D  # noqa: E402
import g63e_render as RN  # noqa: E402
import g63e_solve as S  # noqa: E402
import g63d_common as GD  # noqa: E402

ROT_DEG, TRANS_M = 1.5, 0.05
SIG_XY, SIG_Z = 0.003, 0.003
N_FRAMES = 5
N_VIEWS = 25
SQ_TRUE, SQ_START = 0.041, 0.042
X_GRID = np.array([[x, y, z] for x in np.arange(-0.5, 2.6, 0.25) for y in np.arange(-1.2, 1.3, 0.25) for z in (0, 0.77, 1.3)])


def tape_tags(plan, rng, bad=None, shift=None):
    out = {}
    for t in plan['tags']:
        P = np.array(t['corners'])
        P = P + np.c_[rng.normal(scale=SIG_XY, size=(4, 2)), rng.normal(scale=SIG_Z, size=4)]
        if t['id'] == bad:
            P = P + shift
        out[t['id']] = E.to_w(E.procrustes_pose(P)[0], E.tag_corners_t())     # sudut dunia (bentuk kotak nominal)
    return out


def board_views(rng, Tt, cp, masks, n):
    o1, o2 = Tt['rgbd'][:3, 3], Tt['rgbd2'][:3, 3]
    views = []
    for _ in range(20000):
        if len(views) >= n:
            break
        c = np.r_[rng.uniform(0.1, 1.5), rng.uniform(-0.6, 0.7), rng.uniform(0.4, 1.1)]
        nv = (o1 - c) / np.linalg.norm(o1 - c) + (o2 - c) / np.linalg.norm(o2 - c)
        nv = Rotation.from_rotvec(np.radians(15) * rng.normal(size=3) / 1.7).as_matrix() @ (nv / np.linalg.norm(nv))
        z = -nv
        x = np.cross([0, 0, 1.0], z)
        x /= np.linalg.norm(x)
        R = np.c_[x, np.cross(z, x), z] @ Rotation.from_euler('z', rng.uniform(0, 2 * np.pi)).as_matrix()
        T = E.T_of(R, c - R @ np.array([2 * SQ_TRUE, 2.5 * SQ_TRUE, 0]))
        W = GD.to_w(T, GD.corners_b(SQ_TRUE))
        ok = True
        for cam in E.CAMS:
            K = cp[cam]['K']
            uv, zz = E.project(Tt[cam], K, np.zeros(5), W)
            g = uv.reshape(4, 3, 2)
            side = min(np.linalg.norm(np.diff(g, axis=0), axis=2).min(), np.linalg.norm(np.diff(g, axis=1), axis=2).min())
            ray = (c - Tt[cam][:3, 3]) / np.linalg.norm(c - Tt[cam][:3, 3])
            ang = np.degrees(np.arccos(abs(ray @ R[:, 2])))
            if not ((zz > 0).all() and (uv[:, 0] > 40).all() and (uv[:, 0] < E.IMG_W - 40).all() and (uv[:, 1] > 40).all()
                    and (uv[:, 1] < E.IMG_H - 40).all() and side >= 9.5 and ang <= 55
                    and np.linalg.norm(c - Tt[cam][:3, 3]) <= 2.2):
                ok = False
                break
            ui = np.clip(uv.round().astype(int), 0, [E.IMG_W - 1, E.IMG_H - 1])
            if masks[cam][ui[:, 1], ui[:, 0]].any():
                ok = False
                break
        if ok:
            views.append(T)
    return views


def pt_err(T_true, T_est, K):
    uv, dep = E.project(T_true, K, np.zeros(5), X_GRID)
    m = (dep > 0.3) & (dep < 3.5) & (uv[:, 0] > 0) & (uv[:, 0] < E.IMG_W) & (uv[:, 1] > 0) & (uv[:, 1] < E.IMG_H)
    uv2, _ = E.project(T_est, K, np.zeros(5), X_GRID[m])
    pe = np.linalg.norm(uv2 - uv[m], axis=1) * dep[m] / K[0, 0] * 1000
    return round(float(np.median(pe)), 2), round(float(np.percentile(pe, 95)), 2)


def run(plan, seeds, mode, bgs, cp, masks, log):
    true = {t['id']: np.array(t['T']) for t in plan['tags']}
    vis = {c: [t['id'] for t in plan['tags'] if t['eff'][c]] for c in E.CAMS}
    core = {t['id']: t['kind'] == 'vertikal' for t in plan['tags']}
    tex = {i: RN.tag_texture(i, core=core[i]) for i in true}
    btex = RN.board_texture(SQ_TRUE)
    rows = []
    for sd in range(seeds):
        rng = np.random.default_rng(5000 + sd)
        bad, shift = None, None
        if mode == 'neg3cm':
            seen = sorted(set(vis['rgbd']) | set(vis['rgbd2']))
            bad = int(seen[rng.integers(len(seen))])
            a = rng.uniform(0, 2 * np.pi)
            shift = 0.03 * np.array([np.cos(a), np.sin(a), 0])
        tape = tape_tags(plan, rng, bad, shift)
        tape_T = {i: E.procrustes_pose(P)[0] for i, P in tape.items()}
        Tt, cams = {}, {}
        for c in E.CAMS:
            T = cp[c]['T_wc'].copy()
            ax = rng.normal(size=3)
            T[:3, :3] = Rotation.from_rotvec(np.radians(ROT_DEG) * ax / np.linalg.norm(ax)).as_matrix() @ T[:3, :3]
            d = rng.normal(size=3)
            T[:3, 3] += TRANS_M * d / np.linalg.norm(d)
            Tt[c] = T
            cams[c] = dict(K=cp[c]['K'], dist=np.zeros(5))
        tag_obs, det = {}, {}
        for c in E.CAMS:
            items = [(tex[i][0], tex[i][1], true[i]) for i in vis[c]]
            frames = [RN.render(bgs[c], items, Tt[c], cp[c]['K'], rng) for _ in range(N_FRAMES)]
            obs, notes = D.detect_tags(frames, tape_T, cp[c]['T_wc'], cp[c]['K'], np.zeros(5))
            tag_obs[c] = {i: o['uv'] for i, o in obs.items() if 'uv' in o}
            det[c] = dict(planned=len(vis[c]), got=len(tag_obs[c]), wrong=[i for i in tag_obs[c] if i not in vis[c]], notes=notes)
        Vt = board_views(rng, Tt, cp, masks, N_VIEWS)
        bv, bnotes = [], {}
        for T in Vt:
            v = {}
            for c in E.CAMS:
                img = RN.render(bgs[c], [(btex[0], btex[1], T)], Tt[c], cp[c]['K'], rng)
                uv, note = D.detect_board_free(img, SQ_TRUE)
                if uv is None:
                    bnotes[note.split(':')[0]] = bnotes.get(note.split(':')[0], 0) + 1
                    break
                tu, _ = E.project(Tt[c], cp[c]['K'], np.zeros(5), GD.to_w(T, GD.corners_b(SQ_TRUE)))
                if np.abs(uv - tu).max() > 3:
                    bnotes['LABEL SALAH'] = bnotes.get('LABEL SALAH', 0) + 1
                v[c] = uv
            if len(v) == 2:
                bv.append(v)
        S.SQ0 = SQ_START
        rep, f = S.solve(tag_obs, tape, bv, cams, {c: cp[c]['T_wc'] for c in E.CAMS}, sq_meas=SQ_TRUE)
        row = dict(seed=sd, mode=mode, bad=bad, det=det, board_views=len(bv), board_used=rep['n_board_views'], board_rejected=rep['board_rejected'], board_sampled=len(Vt), board_notes=bnotes,
                   sigma_px=rep['sigma_px'], sq_err_mm=rep.get('G5_sq_err_mm'), loto=rep['loto_mm'],
                   rel_tags_only=rep.get('info_rel_tags_only_vs_joint'))
        for c, Te in (('rgbd', f['T1']), ('rgbd2', f['T2'])):
            dT = np.linalg.inv(Tt[c]) @ Te
            row[c] = dict(rot_deg=round(float(np.degrees(Rotation.from_matrix(dT[:3, :3]).magnitude())), 3),
                          pos_mm=round(float(1000 * np.linalg.norm(dT[:3, 3])), 1), pt_mm=pt_err(Tt[c], Te, cp[c]['K']))
        rows.append(row)
        print(json.dumps(row, default=str), flush=True)
        if log:
            log.write(json.dumps(row, default=str) + '\n')
            log.flush()
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', default=os.path.join(E.HERE, 'g63e_plan.json'))
    ap.add_argument('--seeds', type=int, default=20)
    ap.add_argument('--mode', default='both', choices=('nominal', 'neg3cm', 'both'))
    ap.add_argument('--out', default=None, help='jsonl, ditulis per seed')
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    cp = GD.cam_params()
    bgs = RN.backgrounds()
    pts = GD.robot_points()
    masks = {c: GD.robot_mask(c, cp, pts, 0.05) for c in E.CAMS}
    log = open(a.out, 'a') if a.out else None
    for m in (('nominal', 'neg3cm') if a.mode == 'both' else (a.mode,)):
        run(plan, a.seeds, m, bgs, cp, masks, log)
    return 0


if __name__ == '__main__':
    sys.exit(main())
