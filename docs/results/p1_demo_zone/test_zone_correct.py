#!/usr/bin/env python3
"""Uji unit zone_correct_node (tanpa ROS). Kenapa: node HW harus memberi angka yang SAMA dengan analisis luring (bila beda, uji
hidup DEMO-ZONE-LIVE menilai model lain), dan target di luar zona (wastafel, meja arm_2, benda tinggi) harus DITOLAK, bukan
digeser 15-20 cm ke titik yang tampak sah.    python3 test_zone_correct.py"""
import json
import os

import numpy as np

import zone_correct_node as N
import zone_data as Z

HERE = os.path.dirname(os.path.abspath(__file__))
P3 = N.load_p3()
obs = Z.load()
fails = []


def check(name, cond, info=''):
    print('%s %s %s' % ('PASS' if cond else 'FAIL', name, info))
    if not cond:
        fails.append(name)


# 1. residu 20 obs = analisis (zone_models.json P3 fit_rms, 2.6 cm); pusat = rata-rata tanda (Z0 zone_models)
e = np.array([100 * np.linalg.norm(N.correct_xy(o['trk'], *P3) - o['xy']) for o in obs])
ref = json.load(open(os.path.join(HERE, 'zone_models.json')))['P3']
rms = float(np.sqrt(np.mean(e ** 2)))
check('n_obs 20', len(obs) == 20, len(obs))
check('pusat = rata-rata tanda', np.allclose(P3[0], np.mean([o['xy'] for o in obs], 0), atol=1e-9))
check('RMS = analisis', abs(rms - ref['fit_rms']) < 1e-3 and abs(rms - 2.6) < 0.05, '%.3f vs %.3f cm' % (rms, ref['fit_rms']))
check('yaw 14.72 deg', abs(np.degrees(P3[2]) - 14.72) < 0.01, '%.3f' % np.degrees(P3[2]))

# 2. botol demo 7 cm (A, meja 0.77) di 6 tanda -> semua DITERIMA, z_out 0.805, galat = residu analisis
A = [o for o in obs if o['ds'] == 'A']
trk = [dict(tid=i, label='white bottle', x=o['trk'][0], y=o['trk'][1], z=o['trk'][2], conf=0.8) for i, o in enumerate(A)]
ok, rej = N.process(trk, P3, 0.07)
check('A 6/6 diterima', len(ok) == 6 and not rej, [r['reason'] for r in rej])
check('z_out 0.805', all(abs(t['z'] - 0.805) < 1e-9 for t in ok))
eA = [100 * np.hypot(t['x'] - o['xy'][0], t['y'] - o['xy'][1]) for t, o in zip(ok, A)]
check('A galat <= 5 cm', max(eA) <= 5.0, np.round(eA, 1).tolist())

# 3. botol 165 mm di meja (B_T0) dengan object_height 0.165 -> diterima; dengan 0.07 di kotak C/D -> z tak konsisten -> DITOLAK
B0 = [o for o in obs if o['ds'] == 'B' and o['surf'] == 0.77]
ok, rej = N.process([dict(tid=0, label='b', x=o['trk'][0], y=o['trk'][1], z=o['trk'][2]) for o in B0], P3, 0.165)
check('B_T0 diterima (H 0.165)', len(ok) == len(B0) and not rej, [r['reason'] for r in rej])
Bh = [o for o in obs if o['ds'] == 'B' and o['surf'] > 0.80]
ok, rej = N.process([dict(tid=0, label='b', x=o['trk'][0], y=o['trk'][1], z=o['trk'][2]) for o in Bh], P3, 0.07)
check('benda di kotak (z 0.87-1.00) ditolak (H 0.07)', not ok and all('z track' in r['reason'] for r in rej), len(rej))

# 4. di luar zona -> DITOLAK (data nyata DEMO-HW B1: wastafel cyan, botol depan arm_2; + tepi kotak)
out = [dict(tid=1, label='cyan bottle', x=3.0851, y=-0.3346, z=1.0123),      # wastafel di luar sel
       dict(tid=2, label='gray bottle', x=1.166, y=0.542, z=0.892),           # depan arm_2
       dict(tid=3, label='bottle', x=1.70, y=0.65, z=0.81),                   # track x < 1.78 (ekstrapolasi, bootstrap naik)
       dict(tid=4, label='bottle', x=2.20, y=0.65, z=0.81),                   # track x > 2.17
       dict(tid=5, label='bottle', x=2.00, y=0.85, z=0.81)]                   # track y > 0.81
ok, rej = N.process(out, P3, 0.07)
check('5/5 luar zona ditolak', not ok and len(rej) == 5, [r['reason'] for r in rej])
# titik di kotak track tapi koreksinya keluar zona -> ditolak oleh gerbang zona (bukan gerbang track)
for x, y in [(1.79, 0.50), (2.16, 0.50), (2.16, 0.80)]:       # pojok kotak track -> koreksi (1.69,0.38)/(2.05,0.47)/(1.98,0.76)
    ok, rej = N.process([dict(tid=6, label='bottle', x=x, y=y, z=0.81)], P3, 0.07)
    check('pojok track (%.2f, %.2f) ditolak HANYA oleh gerbang zona' % (x, y),
          not ok and len(rej) == 1 and rej[0]['reason'].startswith('xy terkoreksi'), rej[0]['reason'] if rej else ok)

print('\n%d FAIL' % len(fails) if fails else '\nSEMUA PASS')
raise SystemExit(1 if fails else 0)
