#!/usr/bin/env python3
"""Ringkas uji mandiri G63e (jsonl) + cari ambang G3 LOTO: T layak bila nominal >= 95 % nilai LOTO <= T DAN di mode neg3cm
penanda salah SELALU > T (100 %; penanda salah yang tak terdeteksi dicatat terpisah, tidak dihitung tertangkap).

    python3 g63e_selftest_summary.py dev/selftest_dev40.jsonl
"""
import json
import sys

import numpy as np


def main(path):
    rows = [json.loads(l) for l in open(path)]
    out = {}
    for mode in ('nominal', 'neg3cm'):
        R = [r for r in rows if r['mode'] == mode]
        if not R:
            continue
        print('== %s: %d seed' % (mode, len(R)))
        for c in ('rgbd', 'rgbd2'):
            rot = np.array([r[c]['rot_deg'] for r in R])
            pos = np.array([r[c]['pos_mm'] for r in R])
            ptm = np.array([r[c]['pt_mm'][0] for r in R])
            pt95 = np.array([r[c]['pt_mm'][1] for r in R])
            print('  %-5s rot med %.3f maks %.3f deg | pos med %.1f maks %.1f mm | titik med-med %.1f, p95 med %.1f maks %.1f mm'
                  % (c, np.median(rot), rot.max(), np.median(pos), pos.max(), np.median(ptm), np.median(pt95), pt95.max()))
            out['%s_%s' % (mode, c)] = dict(rot_med=float(np.median(rot)), rot_max=float(rot.max()), pos_med=float(np.median(pos)),
                                            pt_med=float(np.median(ptm)), pt95_med=float(np.median(pt95)), pt95_max=float(pt95.max()))
        bv = [r['board_views'] for r in R]
        sq = [r['sq_err_mm'] for r in R if r['sq_err_mm'] is not None]
        det = {c: [r['det'][c]['got'] for r in R] for c in ('rgbd', 'rgbd2')}
        print('  pose papan lolos dua kamera: med %d (min %d) dari 25 | galat sisi papan med %.2f maks %.2f mm | penanda terdeteksi '
              'rgbd %s rgbd2 %s | label salah %d' % (np.median(bv), min(bv), np.median(sq), max(sq), sorted(set(det['rgbd'])),
                                                     sorted(set(det['rgbd2'])),
                                                     sum(r['board_notes'].get('LABEL SALAH', 0) for r in R)))
    nom = [v for r in rows if r['mode'] == 'nominal' for v in r['loto'].values()]
    neg = [(r['loto'].get(str(r['bad'])), {k: v for k, v in r['loto'].items() if k != str(r['bad'])}) for r in rows if r['mode'] == 'neg3cm']
    if nom:
        print('LOTO nominal (%d nilai): med %.2f p95 %.2f maks %.2f mm' % (len(nom), np.median(nom), np.percentile(nom, 95), max(nom)))
    if neg:
        bad = [b for b, _ in neg if b is not None]
        miss = sum(b is None for b, _ in neg)
        others = [v for _, o in neg for v in o.values()]
        print('LOTO penanda salah (%d): min %.2f med %.2f mm; tak terdeteksi %d | penanda lain di neg: maks %.2f'
              % (len(bad), min(bad), np.median(bad), miss, max(others)))
        T_lo = np.percentile(nom, 95)
        ok = T_lo < min(bad)
        print('AMBANG G3: p95 nominal %.2f %s min salah %.2f -> %s' % (T_lo, '<' if ok else '>=', min(bad),
              'ADA (pilih T di antara, mis. %.1f mm)' % ((T_lo + min(bad)) / 2) if ok else 'TIDAK ADA'))
        caught_frac = {T: (np.mean(np.array(nom) <= T), np.mean(np.array(bad) > T)) for T in (8, 10, 12, 15, 20)}
        for T, (a, b) in caught_frac.items():
            print('  T %2d mm: nominal lolos %.1f %% | salah tertangkap %.1f %%' % (T, 100 * a, 100 * b))
        out['loto'] = dict(nom_p95=float(T_lo), bad_min=float(min(bad)), exists=bool(ok), miss=int(miss))
    return out


if __name__ == '__main__':
    main(sys.argv[1])
