"""Tugas 3b: kamera kandidat vs jangkar lama: incumbent (= T_wc_live sesi jari?) dan incumbent + koreksi G34 (rel, awan
kedalaman lengan REST, p_urdf = R p + t). Juga: pergeseran titik meja zona arm_1 (1.75, 0.55, 0.81) per kandidat."""
import json

import numpy as np

from g63c_common import records
from g63_common import incumbent_T, inv
from g63c_kin import cam_diff

live = {c: np.array(records()[0]['cams'][c]['T_wc_live']) for c in ('rgbd', 'rgbd2')}
g34 = json.load(open('/home/user1/Documents/ceiling_arm/docs/results/p1_g34/rail_joint4_diff.json'))
C = {}
for c in live:
    M = np.eye(4); M[:3, :3] = np.array(g34[c]['R']); M[:3, 3] = g34[c]['t']
    C[c] = M @ incumbent_T(c)
    print(f'{c}: live vs incumbent {cam_diff(live[c], incumbent_T(c))[0]:.3f}°/{cam_diff(live[c], incumbent_T(c))[1]*100:.2f} cm;'
          f' G34-terkoreksi vs incumbent {cam_diff(C[c], incumbent_T(c))[0]:.2f}°/{cam_diff(C[c], incumbent_T(c))[1]*100:.1f} cm')
mod = json.load(open('models_all.json'))
P_t = {'rgbd2': np.array([1.75, 0.55, 0.81, 1]), 'rgbd': np.array([0.70, 0.0, 0.81, 1])}
print('kandidat: vs G34 [rgbd | rgbd2] ; relatif rgbd->rgbd2 vs G34 ; geser titik meja (incumbent->kandidat) cm')
rel = lambda T: inv(T['rgbd']) @ T['rgbd2']   # noqa: E731
for name in ('M0 kaku', 'Ma j1-j5/lengan', 'Mb Ma-j1A+basisB(rx,ry,t)', 'Md rot gantry+j2-j5', 'Me j1 bersama+j2-j5'):
    T = {c: np.array(v) for c, v in mod[name]['cams'].items()}
    s = ' | '.join(f'{c} {cam_diff(T[c], C[c])[0]:.2f}°/{cam_diff(T[c], C[c])[1] * 100:.1f}cm' for c in ('rgbd', 'rgbd2'))
    r = cam_diff(rel(T), rel(C))
    sh = ' '.join(f'{c}@{P_t[c][:3].round(2).tolist()} {np.round((T[c] @ inv(incumbent_T(c)) @ P_t[c] - P_t[c])[:3] * 100, 1).tolist()}'
                  for c in P_t)
    print(f'  {name:28s} {s} ; rel {r[0]:.2f}°/{r[1] * 100:.1f}cm ; {sh}')
sh = ' '.join(f'{c} {np.round((C[c] @ inv(incumbent_T(c)) @ P_t[c] - P_t[c])[:3] * 100, 1).tolist()}' for c in P_t)
print(f'  G34 sendiri: rel vs incumbent {cam_diff(rel(C), rel({c: incumbent_T(c) for c in C}))}; geser titik {sh}')
# cek pita lama (ARM1-ZONE 10-12): botol di tanda (1.75, 0.55, ~0.84) terbaca rgbd2 (+G34) di (1.858, 0.654, 0.836) = +0.151 m
# sepanjang garis pandang. Kandidat benar harus menggeser bacaan itu ke arah tanda (catatan: bias kedalaman botol +8 cm, B64).
pb = np.array([1.858, 0.654, 0.836, 1]); mark = np.array([1.75, 0.55, 0.836])
print('cek pita: bacaan botol rgbd2 (+G34) dipindah ke kandidat -> jarak ke tanda (awal %.1f cm)' % (np.linalg.norm(pb[:3] - mark) * 100))
for name in ('M0 kaku', 'Ma j1-j5/lengan', 'Mb Ma-j1A+basisB(rx,ry,t)', 'Md rot gantry+j2-j5', 'Me j1 bersama+j2-j5'):
    T = np.array(mod[name]['cams']['rgbd2'])
    p = T @ inv(C['rgbd2']) @ pb
    print(f'  {name:28s} -> {np.round(p[:3], 3).tolist()} jarak {np.linalg.norm(p[:3] - mark) * 100:.1f} cm')
