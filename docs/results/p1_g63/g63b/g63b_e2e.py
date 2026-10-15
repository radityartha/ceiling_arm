#!/usr/bin/env python3
"""G63b uji ujung-ke-ujung LURING (tanpa ROS, tanpa HW): pose v3 -> render dua muka -> g63b_detect -> g63_solve.

Kebenaran tersembunyi: T_wc = incumbent digeser (--cam-rot-deg, --cam-pos-m, arah acak per kamera); rakitan per fase
(arm_2, arm_3) = mount nominal digeser (--mount-rot-deg, --mount-pos-m); muka B = muka A @ T_12 (kaku). FK pose
diukur dengan derau (--fk-pos-m, --fk-rot-deg). Prior detektor = incumbent + mount NOMINAL (seperti capture HW).
Render: lembar A4 polos (pola 4x5 @ 42 mm, tepi 21 / 43.5 mm), latar = frame P0 nyata kamera itu (distraktor papan
lama ada), blur 1.3 px, derau 2.3, --frames frame/pose, median per sudut (g63_detect.median_over_frames).
Solver = g63_solve (TIDAK diubah); papan per fase = A2/B2 (arm_2), A3/B3 (arm_3), geometri sudut = board_points.
R0 dirender dua kali (awal + akhir, sendi sama) -> G6' tanpa geser harus <= 2 mm.

    python3 g63b_e2e.py [--seed 1] [--frames 5] > g63b_e2e.log
Lulus: label salah 0; ekstrinsik pulih <= 0.15 deg / 5 mm (= toleransi self-test T3 G63 dgn derau); tidak ada
parameter kamera "tidak teramati"; G1 >= 15 FIT per kamera; G3 median <= 1.5 cm; G4' <= 2.0 cm; G6' <= 2 mm.
"""
import argparse
import glob
import json
import os
import zlib
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
P1 = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, P1)
import g63_detect  # noqa: E402
import g63_solve  # noqa: E402
import g63b_detect  # noqa: E402
import plain_study as PS  # noqa: E402
from g63_common import (ARM_PREFIX, K_NOMINAL, T_from_xyzrpy, board_points, incumbent_T, inv, rot_deg)  # noqa: E402

REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
FRAMES = os.path.join(P1, 'hw_20261015b', 'cap', 'frames')

# solver: any face name (A2, B2, A3, B3) -> the same 12-corner geometry (board_points is name-agnostic in geometry)
g63_solve.board_points = lambda b, sq=None: board_points('A', sq)


def perturb(T, rot_deg_, pos_m, rng):
    from scipy.spatial.transform import Rotation
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax)
    d = rng.normal(size=3)
    d /= np.linalg.norm(d)
    D = np.eye(4)
    D[:3, :3] = Rotation.from_rotvec(np.radians(rot_deg_) * ax).as_matrix()
    D[:3, 3] = pos_m * d
    return T @ D


def sheet_tex(sq):
    t = PS.plain_tex(4, 5, sq, 0.021)
    extra = int(round((0.0435 - 0.021) * PS.PPM))
    return cv2.copyMakeBorder(t, extra, extra, 0, 0, cv2.BORDER_CONSTANT, value=230)


def render(tex, sq, Tcb, K, bg, rng):
    w_m, h_m = 4 * sq + 0.042, 5 * sq + 0.087
    obj = np.array([[-0.021, -0.0435, 0], [w_m - 0.021, -0.0435, 0], [w_m - 0.021, h_m - 0.0435, 0],
                    [-0.021, h_m - 0.0435, 0]])
    rv, _ = cv2.Rodrigues(Tcb[:3, :3])
    dst, _ = cv2.projectPoints(obj, rv, Tcb[:3, 3], K, np.zeros(5))
    h, w = tex.shape
    Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst.reshape(-1, 2).astype(np.float32))
    H_, W_ = bg.shape
    mask = cv2.warpPerspective(np.full_like(tex, 255), Hm, (W_, H_), flags=cv2.INTER_AREA)
    img = np.where(mask > 127, cv2.warpPerspective(tex, Hm, (W_, H_), flags=cv2.INTER_AREA), bg).astype(np.uint8)
    img = cv2.GaussianBlur(img, (0, 0), 1.3)
    return np.clip(img + rng.normal(scale=2.3, size=img.shape), 0, 255).astype(np.uint8)


def project(Tcb, K, sq):
    P = np.array([board_points('A', sq)[i] for i in range(12)])
    rv, _ = cv2.Rodrigues(Tcb[:3, :3])
    uv, _ = cv2.projectPoints(P, rv, Tcb[:3, 3], K, np.zeros(5))
    return uv.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--poses', default=os.path.join(HERE, 'poses_g63b_v3.json'))
    ap.add_argument('--mount', default=os.path.join(HERE, 'mount_g63b.json'))
    ap.add_argument('--urdf', default=os.path.join(P1, 'live_urdf_20261011.urdf'))
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--cam-rot-deg', type=float, default=2.0)
    ap.add_argument('--cam-pos-m', type=float, default=0.08)
    ap.add_argument('--mount-rot-deg', type=float, default=3.0)
    ap.add_argument('--mount-pos-m', type=float, default=0.015)
    ap.add_argument('--fk-pos-m', type=float, default=0.002)
    ap.add_argument('--fk-rot-deg', type=float, default=0.05)
    ap.add_argument('--fk-rep-pos-m', type=float, default=0.0003)
    ap.add_argument('--dump-ds', help='tulis dataset per fase <prefix>_arm_2.json / _arm_3.json (uji g63b_solve)')
    ap.add_argument('--r0b-shift-m', type=float, default=0.0,
                    help="kontrol positif G6': rakitan digeser (m, arah acak) sebelum R0 akhir -> G6' HARUS gagal")
    ap.add_argument('--fk-rep-rot-deg', type=float, default=0.01)
    a = ap.parse_args()
    import pinocchio as pin
    rng = np.random.default_rng(a.seed)
    PD = json.load(open(a.poses))
    mount = json.load(open(a.mount))['A']
    sq = float(mount['square_m'])
    T12 = np.array(PD['g63b']['T_12'])
    Tlb_nom = T_from_xyzrpy(*mount['xyz'], *mount['rpy'])
    m = pin.buildModelFromUrdf(a.urdf)
    d = m.createData()
    cams = ('rgbd', 'rgbd2')
    Twc_true = {c: perturb(incumbent_T(c), a.cam_rot_deg, a.cam_pos_m, rng) for c in cams}
    bgs = {c: [cv2.imread(f, 0) for f in sorted(glob.glob(os.path.join(FRAMES, '*', c, '*.png')))[::10]] for c in cams}
    if not all(bgs.values()):
        sys.exit('GAGAL: frame latar P0 tidak ada')
    tex = sheet_tex(sq)
    ds = dict(square_m={}, cameras={c: dict(K=K_NOMINAL[c].tolist(), dist=[0.0] * 5) for c in cams}, poses=[])
    stats = dict(rendered=0, detected=0, wrong=0, notes={})
    truth_lb = {}
    for arm, A in PD['arms'].items():
        ph = arm[-1]
        names = {1: f'A{ph}', 2: f'B{ph}'}
        Tlb_true = perturb(Tlb_nom, a.mount_rot_deg, a.mount_pos_m, rng)
        sh = np.eye(4)
        v = rng.normal(size=3)
        sh[:3, 3] = a.r0b_shift_m * v / np.linalg.norm(v)
        shift = {ph: sh}
        truth_lb[names[1]], truth_lb[names[2]] = Tlb_true, Tlb_true @ T12
        for nm in names.values():
            ds['square_m'][nm] = sq
        pre = ARM_PREFIX[arm]
        seq = A['poses'] + [dict(A['poses'][0], id=A['poses'][0]['id'] + 'b')]     # R0 again at the end
        for p in seq:
            q = pin.neutral(m)
            base = {'t1_linear_joint': PD['rail1'], 't2_linear_joint': PD['rail2']}
            for ap_, pp in ARM_PREFIX.items():
                for k, v in enumerate(REST):
                    base[f'{pp}joint_{k + 1}'] = v
            base.update(p['goal_joints'])
            for n, v in base.items():
                q[m.joints[m.getJointId(n)].idx_q] = v
            pin.framesForwardKinematics(m, d, q)
            M = d.oMf[m.getFrameId(pre + mount['link'])]
            Twl = np.eye(4)
            Twl[:3, :3], Twl[:3, 3] = M.rotation, M.translation
            # FK error = SYSTEMATIC per joint configuration (same joints -> same error; seeded by pose id without the
            # repeat suffix) + small per-visit repeatability noise. (v1 of this test redrew 2 mm per visit -> false G6'
            # alarms up to 4.1 mm on an R0 repeat with no drift: a test-model error, not a tool error.)
            rng_cfg = np.random.default_rng([a.seed, zlib.crc32(p['id'].rstrip('b').encode())])
            Twl_meas = perturb(perturb(Twl, a.fk_rot_deg, a.fk_pos_m, rng_cfg), a.fk_rep_rot_deg, a.fk_rep_pos_m, rng)
            for face, nm in names.items():
                obs = {}
                for c, v in p['cams'].items():
                    if int(v['face']) != face:
                        continue
                    Tl = truth_lb[nm]
                    if p['id'].endswith('b') and a.r0b_shift_m > 0:
                        Tl = shift[ph] @ Tl
                    Tcb = inv(Twc_true[c]) @ Twl @ Tl
                    prior = project(inv(incumbent_T(c)) @ Twl_meas @ (Tlb_nom if face == 1 else Tlb_nom @ T12),
                                    K_NOMINAL[c], sq)
                    truth = project(Tcb, K_NOMINAL[c], sq)
                    dets = []
                    for f in range(a.frames):
                        bg = bgs[c][int(rng.integers(len(bgs[c])))]
                        img = render(tex, sq, Tcb, K_NOMINAL[c], bg, rng)
                        ids, uv, note = g63b_detect.detect_face(img, prior, sq)
                        stats['rendered'] += 1
                        stats['notes'][note.split(' ')[0]] = stats['notes'].get(note.split(' ')[0], 0) + 1
                        if ids:
                            stats['detected'] += 1
                            if np.linalg.norm(np.array(uv) - truth, axis=1).max() > 0.5 * sq * K_NOMINAL[c][0, 0] / \
                                    v['dist']:
                                stats['wrong'] += 1
                            dets.append(dict(ids=ids, uv=uv, method='polos'))
                    if dets:
                        r = g63_detect.median_over_frames(dets)
                        if r['ids']:
                            obs[c] = dict(ids=r['ids'], uv=r['uv'])
                            e = np.linalg.norm(np.array(r['uv']) - truth[r['ids']], axis=1)
                            if e.max() > 2.0 and os.environ.get('G63B_DEBUG_DIR'):
                                vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                                for k, (pt, tt) in enumerate(zip(np.array(r['uv']), truth)):
                                    cv2.circle(vis, tuple(int(x) for x in tt), 4, (0, 255, 0), 1)
                                    cv2.circle(vis, tuple(int(x) for x in pt), 2, (0, 0, 255), -1)
                                    cv2.putText(vis, str(k), tuple(int(x) + 5 for x in pt), 0, 0.4, (255, 0, 0), 1)
                                u0, v0 = truth.mean(0).astype(int)
                                crop = vis[max(0, v0 - 110):v0 + 110, max(0, u0 - 110):u0 + 110]
                                cv2.imwrite(os.path.join(os.environ['G63B_DEBUG_DIR'], f"{p['id']}_{nm}_{c}.png"),
                                            cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
                                print('DEBUG', p['id'], nm, c, 'galat per sudut', np.round(e, 2).tolist())
                            stats.setdefault('worst', []).append((float(e.max()), f"{p['id']}:{nm}:{c}", len(dets),
                                                                  float(v['dist'])))
                if obs or p['role'] == 'R0':
                    ds['poses'].append(dict(id=f"{p['id']}:{nm}", board=nm, role=p['role'], stage=p['stage'],
                                            zone=p['zone'], T_wl=Twl_meas.tolist(), obs=obs))
    print('deteksi: %d/%d frame, label SALAH %d, catatan %s' % (stats['detected'], stats['rendered'], stats['wrong'],
                                                                 stats['notes']))
    w = sorted(stats.get('worst', []), reverse=True)
    print('galat sudut median-frame vs kebenaran (px): p50 %.2f, 5 terburuk %s' % (
        float(np.median([x[0] for x in w])), [(round(x[0], 2), x[1], x[2], round(x[3], 2)) for x in w[:5]]))
    fit = {c: sum(1 for p in ds['poses'] if p['role'] == 'FIT' and c in p['obs']) for c in cams}
    print('G1 FIT per kamera:', fit)
    if a.dump_ds:
        for ph in ('2', '3'):
            sub = dict(ds, square_m={b: v for b, v in ds['square_m'].items() if b.endswith(ph)},
                       poses=[p for p in ds['poses'] if p['board'].endswith(ph)])
            json.dump(sub, open(f'{a.dump_ds}_arm_{ph}.json', 'w'))
    res, Twc, Tlb = g63_solve.solve(ds)
    print('Twc_xyzrpy', res['Twc_xyzrpy'])
    err = {c: (rot_deg(Twc_true[c][:3, :3], Twc[c][:3, :3]), float(np.linalg.norm(Twc_true[c][:3, 3] - Twc[c][:3, 3])))
           for c in cams}
    print('konvergen', res['converged'], 'tak teramati', res['unobservable'])
    for c in cams:
        print('  %s: galat vs KEBENARAN %.4f deg / %.1f mm | geser dari incumbent %.2f deg / %.1f mm' % (
            c, err[c][0], err[c][1] * 1e3, res['vs_incumbent'][c]['rot_deg'],
            res['vs_incumbent'][c]['cam_pos_shift_m'] * 1e3))
    for b in sorted(Tlb):
        e = (rot_deg(truth_lb[b][:3, :3], Tlb[b][:3, :3]), np.linalg.norm(truth_lb[b][:3, 3] - Tlb[b][:3, 3]))
        print('  papan %s: T_lb galat %.2f deg / %.1f mm' % (b, e[0], e[1] * 1e3))
    ho = g63_solve.evaluate_holdout(ds, Twc, Tlb)
    print('G3 uji-tahan:', {c: {k: (round(v * 100, 2) if isinstance(v, float) else v) for k, v in g.items()}
                            for c, g in ho['G3'].items()}, '(cm)')
    print("G4' :", {c: {k: (round(v * 100, 2) if isinstance(v, float) else v) for k, v in g.items()}
                    for c, g in ho['G4p'].items()}, '(cm)')
    print('G4 dua-kamera:', ho['G4'])
    dr = g63_solve.drift_R0(ds, Twc)
    print("G6' R0:", [(x['pose'], round(x.get('lateral_rms_m', float('nan')) * 1e3, 2)) for x in dr], '(mm)')
    ok = (stats['wrong'] == 0 and not res['unobservable'] and all(e[0] <= 0.15 and e[1] <= 0.005 for e in err.values())
          and all(v >= 15 for v in fit.values())
          and all(g['median_m'] is not None and g['median_m'] <= 0.015 for g in ho['G3'].values())
          and all(g['median_m'] <= 0.02 for g in ho['G4p'].values())
          and all(x.get('lateral_rms_m', 0) <= 0.002 for x in dr))
    print('HASIL E2E:', 'LULUS' if ok else 'GAGAL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
