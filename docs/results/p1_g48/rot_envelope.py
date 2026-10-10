#!/usr/bin/env python3
"""HW-VERIFIED rotation envelope per gantry = the default rotation limit of pose_to_g / run_g32 (G48, operator 2026-10-10:
"sementara 10, siapkan agar default bisa diubah"). One JSON file, widened only by evidence from the stepwise sweep (6b).

    python3 rot_envelope.py show
    python3 rot_envelope.py widen --gantry 1 --lo -35 --hi 35 --evidence <sweep log>   # after an amplitude PASSED on HW

An explicit --rot-max on pose_to_g / run_g32 still overrides (symmetric, as before).
"""
import argparse
import json
import os
import sys
import time

FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rot_envelope.json')
TOL = 0.5                    # measured rotation may sit this far outside (bridge stops <= 0.3 short, G30)


def load():
    d = json.load(open(FILE))
    return {g: (float(d[g][0]), float(d[g][1])) for g in ('1', '2')}


def allowed(g, deg, env=None, tol=0.0):
    lo, hi = (env or load())[str(g)]
    return lo - tol - 1e-9 <= deg <= hi + tol + 1e-9


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('show')
    w = sub.add_parser('widen')
    w.add_argument('--gantry', choices=('1', '2'), required=True)
    w.add_argument('--lo', type=float, required=True)
    w.add_argument('--hi', type=float, required=True)
    w.add_argument('--evidence', required=True, help='log of the PASSED sweep legs (must exist)')
    a = ap.parse_args()
    d = json.load(open(FILE))
    if a.cmd == 'show':
        for g in ('1', '2'):
            print(f'g{g}: {d[g][0]:+.1f} .. {d[g][1]:+.1f} deg')
        for h in d.get('history', [])[-5:]:
            print('  ', h)
        return 0
    if not os.path.isfile(a.evidence):
        ap.error(f'bukti {a.evidence} tidak ada -- TOLAK')
    if not (-175.0 <= a.lo <= 0.0 <= a.hi <= 175.0):
        ap.error('amplop harus -175 <= lo <= 0 <= hi <= 175')
    lo, hi = min(d[a.gantry][0], a.lo), max(d[a.gantry][1], a.hi)       # only ever WIDENS
    d.setdefault('history', []).append(dict(t=time.strftime('%Y-%m-%d %H:%M'), gantry=a.gantry,
                                            old=list(d[a.gantry]), new=[lo, hi], evidence=a.evidence))
    d[a.gantry] = [lo, hi]
    json.dump(d, open(FILE, 'w'), indent=1)
    print(f'g{a.gantry}: amplop {lo:+.1f} .. {hi:+.1f} deg (bukti {a.evidence})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
