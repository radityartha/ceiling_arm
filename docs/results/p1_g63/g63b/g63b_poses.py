#!/usr/bin/env python3
"""G63b pembangkit pose: g63_poses.py (G63, TIDAK diubah) + rakitan DUA muka di badan gripper + aturan j4 opsional.

Tambalan (monkeypatch modul g63_poses, hanya di proses ini):
  * visibility: muka 1 = frame papan di mount; muka 2 = T_wb @ T_12 (rot y 180 deg, asal (4*kotak, 0, card_t)) --
    papan kedua punggung-ke-punggung. Tiap kamera dinilai pada muka yang menghadapnya; cams[c]['face'] = 1|2.
    Satu pose kini BISA terlihat dua kamera (muka berbeda) -> pengikat antar-kamera (G4 lama) mungkin lagi.
  * occluded: sinar ke sudut muka yang dilihat kamera itu.
  * torque_ok: tambahan |tau_j4 statis| >= --j4-min (N.m) -- usulan anti-berburu j4 (P0 A-02: j4 0.33-1.14 N.m
    berburu pp 3.2e-3 rad); 0 = mati. Saringan lama (92 % statis) tetap, tidak dilonggarkan.
Ukuran pola dari mount square_m (0.042); 12 sudut dalam 3x4 = tata letak sama dengan board_points G63 (diskalakan).

    python3 g63b_poses.py --urdf ../live_urdf_20261011.urdf --mount mount_g63b_guess.json --out poses_g63b_guess.json \
        --rail1 1.40 --samples 8000 [--j4-min 1.5]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import g63_poses as P  # noqa: E402

J4_MIN = 0.0
R0_ANGLE_MAX = 40.0   # operator 2026-10-15 ("setuju rekomendasimu"): R0 dilihat <= 40 deg di SETIAP kamera yang melihatnya
                      # (mock 10-15: R0 rgbd 50 deg -> penjaga residu H per sudut menolak muka B)
STATS = dict(j4_reject=0, j4_vals=[])


def t12(mt):
    T = np.diag([-1.0, 1.0, -1.0, 1.0])
    T[:3, 3] = [4 * float(mt['square_m']), 0.0, float(mt['card_t'])]
    return T


_vis, _occ, _tok = P.visibility, P.occluded, P.torque_ok
_T12 = {}


def visibility2(cell, T_wb, Pts):
    oa, ca, na, Pa = _vis(cell, T_wb, Pts)
    ob, cb, nb, Pb = _vis(cell, T_wb @ _T12['cur'], Pts)
    out, Pw = {}, {}
    for c, v in oa.items():
        out[c], Pw[c] = dict(v, face=1), Pa
    for c, v in ob.items():
        if c in out:                                    # mustahil secara geometri (normal berlawanan, sudut <= 55)
            raise RuntimeError('kamera %s melihat kedua muka' % c)
        out[c], Pw[c] = dict(v, face=2), Pb
    return out, (ca + cb) / 2, na, Pw


def occluded2(cell, arm, joints, cam, Pw):
    return _occ(cell, arm, joints, cam, Pw[cam] if isinstance(Pw, dict) else Pw)


def torque_ok2(cell, arm, q):
    ok, worst = _tok(cell, arm, q)
    if ok:
        tau = cell.pin.computeGeneralizedGravity(cell.m, cell.d, q)
        t4 = abs(tau[cell.m.joints[cell.m.getJointId(f'{P.ARM_PREFIX[arm]}joint_4')].idx_v])
        STATS['j4_vals'].append(float(t4))
        if J4_MIN > 0 and t4 < J4_MIN:
            STATS['j4_reject'] += 1
            return False, worst
    return ok, worst


def run_main(a_urdf, mount_path, out_path, samples, rail1, rail2, seed):
    """= g63_poses.main with ONE change: the R0 pool requires every seeing camera at <= R0_ANGLE_MAX (then the G63 order of
    preference: both cameras, else a camera <= 1.6 m, else any). Everything else (FPS order, holdout per category,
    records) is g63_poses' own code path."""
    import tempfile
    mount = json.load(open(mount_path))
    payload = os.path.join(tempfile.mkdtemp(prefix='g63b_'), 'g63_payload.urdf')
    P.g63_urdf.build(a_urdf, mount, payload)
    P.g63_screens.install(payload)
    cell = P.Cell(payload, rail1, rail2)
    cell.mount = mount
    logs = []

    def log(s):
        print(s, flush=True)
        logs.append(s)
    rng = np.random.default_rng(seed)
    zones = P.rest_zones(cell)
    out = dict(created='g63b_poses.py (g63_poses + R0 <= %.0f deg)' % R0_ANGLE_MAX, urdf=os.path.abspath(a_urdf),
               mount=mount, mount_guess=any(v.get('guess') for k, v in mount.items() if not k.startswith('_')),
               rail1=rail1, rail2=rail2, seed=seed, samples=samples, stages=P.STAGES,
               incumbent_used_for_visibility=True, arms={})
    for board, arm in sorted(P.BOARD_ARM.items()):
        mc = P.mount_check(cell, arm)
        log(f'{arm}: cek dudukan {mc} m (kotak tunggal rakitan; upper_wrist = touch link di attach g63b)')
        good, cnt = P.generate(cell, arm, board, samples, rng, log)
        if not good:
            log(f'!!! {arm}: TIDAK ADA kandidat lolos')
            out['arms'][arm] = dict(board=board, counts=cnt, mount_check=mc, poses=[])
            continue
        qrest = np.array(P.REST)
        ok40 = [i for i, c in enumerate(good) if all(v['angle'] <= R0_ANGLE_MAX for v in c['vis'].values())]
        both = [i for i in ok40 if len(good[i]['vis']) == 2]
        near = [i for i in ok40 if min(v['dist'] for v in good[i]['vis'].values()) <= 1.6]
        pool = both or near or ok40
        if not pool:
            raise SystemExit(f'{arm}: tidak ada calon R0 dengan sudut <= {R0_ANGLE_MAX} -- TOLAK (aturan tidak dilonggarkan)')
        r0 = min(pool, key=lambda i: (np.linalg.norm(good[i]['q6'] - qrest), good[i]['tfrac']))
        log(f'{arm}: calon R0 <= {R0_ANGLE_MAX:.0f} deg: {len(ok40)} (dua-kamera {len(both)}, <= 1.6 m {len(near)})')
        sub = [good[i] for i in range(len(good)) if i != r0]
        seed_i = max(range(len(sub)), key=lambda i: np.linalg.norm(sub[i]['ctr'] - good[r0]['ctr']))
        order = P.order_fps(sub, P.STAGES[-1], seed_i)

        def rec(c, role, stage, idx):
            pos, quat = P.tool_pose(cell, arm, c['q6'])
            return dict(id=f'{board}-{idx:02d}' if role != 'R0' else f'{board}-R0', board=board, arm=arm, role=role,
                        zone=P.zone_of(zones, c['ctr']), stage=stage,
                        goal_joints={f'{P.ARM_PREFIX[arm]}joint_{k + 1}': round(float(v), 6) for k, v in enumerate(c['q6'])},
                        tool_xyz=[round(v, 5) for v in pos], tool_quat_xyzw=[round(v, 6) for v in quat],
                        board_centre=[round(float(v), 4) for v in c['ctr']],
                        board_normal=[round(float(v), 4) for v in c['nrm']],
                        cams={k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in c['vis'].items()},
                        occluded_by=c.get('occluded_by', {}), static_torque_frac=round(c['tfrac'], 4),
                        clearance=c['col'])
        poses = [rec(good[r0], 'R0', 'all', 0)]
        per_cat = {}
        for n, i in enumerate(order, start=1):
            stage = next(k for k, cut in enumerate(P.STAGES) if n <= cut)
            cat = P.category(sub[i])
            per_cat[cat] = per_cat.get(cat, 0) + 1
            role = 'HOLDOUT' if per_cat[cat] % 4 == 0 else 'FIT'
            poses.append(rec(sub[i], role, ['P0', 'T1', 'T2', 'T3'][stage], n))
        out['arms'][arm] = dict(board=board, counts=cnt, mount_check=mc, n_candidates=len(good), poses=poses)
        log(f'{arm}: R0 kamera {poses[0]["cams"]}; urutan {len(poses) - 1}')
    out['log'] = logs
    json.dump(out, open(out_path, 'w'), indent=1)
    print(f'tulis {out_path}')
    return 0


def main():
    global J4_MIN
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--j4-min', type=float, default=0.0)
    a, rest = ap.parse_known_args()
    J4_MIN = a.j4_min
    mount = json.load(open(rest[rest.index('--mount') + 1]))
    T = {t12(v).tobytes() for k, v in mount.items() if not k.startswith('_')}
    if len(T) != 1:
        raise SystemExit('rakitan A dan B harus geometri sama (satu rakitan dipindah lengan)')
    _T12['cur'] = t12(mount['A'])
    P.visibility, P.occluded, P.torque_ok = visibility2, occluded2, torque_ok2
    ap2 = argparse.ArgumentParser()
    ap2.add_argument('--urdf', required=True)
    ap2.add_argument('--mount', required=True)
    ap2.add_argument('--out', required=True)
    ap2.add_argument('--samples', type=int, default=40000)
    ap2.add_argument('--rail1', type=float, default=1.50)
    ap2.add_argument('--rail2', type=float, default=0.00)
    ap2.add_argument('--seed', type=int, default=63)
    b = ap2.parse_args(rest)
    rc = run_main(b.urdf, b.mount, b.out, b.samples, b.rail1, b.rail2, b.seed)
    out = b.out
    d = json.load(open(out))
    v = np.array(STATS['j4_vals']) if STATS['j4_vals'] else np.array([np.nan])
    d['g63b'] = dict(created='g63b_poses.py', j4_min=J4_MIN, j4_reject=STATS['j4_reject'], T_12=_T12['cur'].tolist(),
                     j4_static_quantiles=[round(float(x), 3) for x in np.nanpercentile(v, [5, 25, 50, 75, 95])])
    json.dump(d, open(out, 'w'), indent=1)
    print('g63b:', d['g63b'])
    return rc


if __name__ == '__main__':
    sys.exit(main())
