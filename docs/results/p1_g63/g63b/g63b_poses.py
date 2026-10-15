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
    sys.argv = [sys.argv[0]] + rest
    rc = P.main()
    out = rest[rest.index('--out') + 1]
    d = json.load(open(out))
    v = np.array(STATS['j4_vals']) if STATS['j4_vals'] else np.array([np.nan])
    d['g63b'] = dict(created='g63b_poses.py', j4_min=J4_MIN, j4_reject=STATS['j4_reject'], T_12=_T12['cur'].tolist(),
                     j4_static_quantiles=[round(float(x), 3) for x in np.nanpercentile(v, [5, 25, 50, 75, 95])])
    json.dump(d, open(out, 'w'), indent=1)
    print('g63b:', d['g63b'])
    return rc


if __name__ == '__main__':
    sys.exit(main())
