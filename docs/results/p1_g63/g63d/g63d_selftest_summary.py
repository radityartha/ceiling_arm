#!/usr/bin/env python3
"""Ringkas uji mandiri: galat kamera, LOBO nominal vs kontrol negatif 3 cm, deteksi, label salah, G4.
Ambang G3 kandidat T: nominal lulus >= 95 % seed-kamera DAN kontrol negatif tertangkap 100 % (kamera yang melihat papan salah
dengan LOBO terhitung).    python3 g63d_selftest_summary.py selftest.json"""
import json
import sys

import numpy as np

rows = json.load(open(sys.argv[1]))
cams = ('rgbd', 'rgbd2')
for c in cams:
    nom = [r[c] for r in rows if r['mode'] == 'nominal' and 'rot_deg' in r[c]]
    if nom:
        print('%s nominal n=%d rot med/max %.3f/%.3f deg pos med/max %.1f/%.1f mm titik med %.1f p95(maks seed) %.1f mm sigma %.2f px'
              % (c, len(nom), np.median([x['rot_deg'] for x in nom]), max(x['rot_deg'] for x in nom),
                 np.median([x['pos_mm'] for x in nom]), max(x['pos_mm'] for x in nom),
                 np.median([x['pt_mm_med'] for x in nom]), max(x['pt_mm_p95'] for x in nom), np.median([x['sigma_px'] for x in nom])))
    det = {}
    for r in rows:
        for b, (k, n) in r[c].get('det', {}).items():
            det.setdefault(b, []).append(k / n)
    print('   deteksi frac per papan', {b: round(float(np.mean(v)), 2) for b, v in det.items()},
          'label salah', sum(len(r[c].get('wrong_label', [])) for r in rows))
nom_max = [max(v for v in r[c]['lobo'].values() if v is not None) for r in rows if r['mode'] == 'nominal' for c in cams
           if 'lobo' in r[c] and any(v is not None for v in r[c]['lobo'].values())]
nom_none = sum(1 for r in rows if r['mode'] == 'nominal' for c in cams if 'lobo' in r[c] and not any(v is not None for v in r[c]['lobo'].values()))
neg = [(max(v for v in r[c]['lobo'].values() if v is not None), r[c]['lobo'].get(r['bad']),
        max(r[c]['lobo'], key=lambda k: r[c]['lobo'][k] or -1) == r['bad'])
       for r in rows if r['mode'] == 'neg3cm' for c in cams
       if 'lobo' in r[c] and r['bad'] in r[c]['lobo'] and r[c]['lobo'][r['bad']] is not None]
print('LOBO maks nominal (seed-kamera) n=%d: p50 %.1f p95 %.1f maks %.1f mm; kamera tanpa LOBO (< 4 papan) %d'
      % (len(nom_max), np.median(nom_max), np.percentile(nom_max, 95), max(nom_max), nom_none))
if neg:
    nm = [x[0] for x in neg]
    print('kontrol neg n=%d: LOBO maks min %.1f p50 %.1f mm; papan salah = maks %d/%d'
          % (len(neg), min(nm), np.median(nm), sum(x[2] for x in neg), len(neg)))
    for T in (15, 20, 25, 30, 35, 40):
        print('  T=%2d mm: nominal lulus %d/%d, negatif tertangkap %d/%d' %
              (T, sum(m <= T for m in nom_max), len(nom_max), sum(m > T for m in nm), len(nm)))
g4 = [v.get('tri_mm', v.get('cam_cam_mm')) for r in rows if r['mode'] == 'nominal' for v in r['G4'].values() if v]
if g4:
    print('G4 nominal: p50 %.1f p95 %.1f maks %.1f mm (n=%d)' % (np.median(g4), np.percentile(g4, 95), max(g4), len(g4)))
