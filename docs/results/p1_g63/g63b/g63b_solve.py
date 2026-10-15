#!/usr/bin/env python3
"""G63b solver: g63_solve.py (G63, TIDAK diubah) atas dataset gabungan dua fase (arm_2: A2/B2, arm_3: A3/B3).

Tambalan: board_points(nama) -> geometri 12 sudut yang sama untuk semua nama muka (papan polos 4x5, skala square_m
per nama). Dataset digabung: poses disatukan, square_m disatukan, intrinsik kamera harus IDENTIK antar berkas (tolak
bila beda). Keluaran sama dengan g63_solve (solve + uji-tahan + R0_drift).

    python3 g63b_solve.py --data ds_arm_2.json [ds_arm_3.json] --out result.json
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g63_solve  # noqa: E402
from g63_common import board_points  # noqa: E402

g63_solve.board_points = lambda b, sq=None: board_points('A', sq)
SIGMA_FLOOR_PX = 0.3     # lantai derau piksel untuk kovarians (mock 10-15: fit pas-sempurna -> sigma 0 -> std 0 palsu)
MIN_RES_PER_PARAM = 2.0  # residu >= 2 x parameter, kalau tidak = UNDERDETERMINED (tolak)


def guard(ds, res, Twc, Tlb):
    """Observability re-check that a perfect (over-parameterised) fit cannot fake: full-rank numeric Jacobian at the
    solution, n_residuals >= 2 m, camera std with sigma = max(sigma_fit, 0.3 px). Only ADDS refusals to g63_solve's."""
    from scipy.optimize._numdiff import approx_derivative
    pb = g63_solve.Problem(ds, roles=('FIT',))
    x = pb.pack(Twc, Tlb)
    J = approx_derivative(pb.residuals, x)
    n, m = J.shape
    fx = min(float(np.asarray(v['K'])[0][0]) for v in ds['cameras'].values())
    sigma = max(res['sigma_norm'], SIGMA_FLOOR_PX / fx)
    sv = np.linalg.svd(J, compute_uv=False)
    rank = int((sv > sv[0] * 1e-9).sum()) if sv.size else 0
    out = dict(n_residuals=int(n), n_params=int(m), rank=rank, sigma_used_px=sigma * fx, refusals=[])
    if n < MIN_RES_PER_PARAM * m:
        out['refusals'].append(f'UNDERDETERMINED: {n} residu < {MIN_RES_PER_PARAM:g} x {m} parameter')
    if rank < m:
        out['refusals'].append(f'RANK J {rank} < {m} parameter -> sebagian parameter tak teramati')
    else:
        std = np.sqrt(np.clip(np.diag(np.linalg.inv(J.T @ J)), 0, None)) * sigma
        for k, c in enumerate(pb.cams):
            sr, st = float(np.degrees(std[6 * k:6 * k + 3]).max()), float(std[6 * k + 3:6 * k + 6].max())
            out[f'std_{c}'] = dict(rot_deg=sr, pos_m=st)
            if not (sr <= g63_solve.OBS_STD_ROT_DEG and st <= g63_solve.OBS_STD_POS_M):
                out['refusals'].append(f'{c} tak teramati (lantai {SIGMA_FLOOR_PX} px): std {sr:.3f} deg / {st * 1e3:.1f} mm')
    return out


def combine(paths):
    ds = dict(square_m={}, cameras={}, poses=[])
    for p in paths:
        d = json.load(open(p))
        for c, v in d['cameras'].items():
            if c in ds['cameras'] and np.abs(np.array(v['K']) - np.array(ds['cameras'][c]['K'])).max() > 1e-6:
                raise SystemExit(f'K {c} beda antar dataset ({p}) -- TOLAK')
            ds['cameras'].setdefault(c, v)
        for b, s in d['square_m'].items():
            if b in ds['square_m']:
                raise SystemExit(f'nama papan {b} muncul di dua dataset -- fase harus beda nama (A2/B2 vs A3/B3)')
            ds['square_m'][b] = s
        ds['poses'] += d['poses']
    return ds


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    ds = combine(a.data)
    res, Twc, Tlb = g63_solve.solve(ds)
    res['holdout'] = g63_solve.evaluate_holdout(ds, Twc, Tlb)
    res['R0_drift'] = g63_solve.drift_R0(ds, Twc)
    res['g63b'] = dict(data=[os.path.abspath(p) for p in a.data], boards=sorted(ds['square_m']))
    res['g63b_guard'] = guard(ds, res, Twc, Tlb)
    if res['g63b_guard']['refusals']:
        print('!!! PENJAGA G63b MENOLAK:', res['g63b_guard']['refusals'], file=sys.stderr)
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('converged', 'Twc_xyzrpy', 'vs_incumbent', 'unobservable')}, indent=1))
    print('G3', json.dumps(res['holdout']['G3']), "G4'", json.dumps(res['holdout']['G4p']))
    print('penjaga G63b:', {k: v for k, v in res['g63b_guard'].items() if k != 'refusals'},
          'TOLAK' if res['g63b_guard']['refusals'] else 'OK')
    return 0 if res['converged'] and not res['unobservable'] and not res['g63b_guard']['refusals'] else 2


if __name__ == '__main__':
    sys.exit(main())
