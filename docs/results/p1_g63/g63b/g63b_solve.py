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
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('converged', 'Twc_xyzrpy', 'vs_incumbent', 'unobservable')}, indent=1))
    print('G3', json.dumps(res['holdout']['G3']), "G4'", json.dumps(res['holdout']['G4p']))
    return 0 if res['converged'] and not res['unobservable'] else 2


if __name__ == '__main__':
    sys.exit(main())
