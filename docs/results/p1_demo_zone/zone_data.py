"""ZONE-C: kumpulkan semua observasi botol rgbd2 di zona meja demo arm_1 (data yang ADA, NOL HW).
A = RGBD2-ZONE 2026-10-12 (botol demo putih 7 cm, meja 0.77, 6 tanda), B = CAM-6DOF 2026-10-14 (botol putih H 165 / D 65 mm,
permukaan 0.77/0.87/0.91, tanda sama). Tanda = TCP arm_1 terukur (FK). Ekstrinsik saat rekam = incumbent launch + G34 (sama)."""
import glob
import json
import os

import numpy as np

R = '/home/user1/Documents/ceiling_arm/docs/results'
C_INC = np.array([-0.1547, -1.0197, 2.0905])          # pusat optik rgbd2 incumbent (tf2_x/y/z)
RPY_INC = (-2.2369, -0.1285, -0.9189)


def load():
    obs = []
    a = json.load(open(R + '/p1_rgbd2_zone/fit_data.json'))
    for k, m in a['marks'].items():
        obs.append(dict(ds='A', mark=k, surf=0.77, H=0.07, xy=np.array(m), trk=np.array(a['tracks'][k]), tag='A_' + k))
    for f in sorted(glob.glob(R + '/p1_cam6dof/hw_20261014/tracks_inc/*_summary.json')):
        s = json.load(open(f))
        c = s.get('cameras', {}).get('rgbd2', {})
        if 'mark' not in s or s['mark'].startswith('F') or 'median' not in c:
            continue
        obs.append(dict(ds='B', mark=s['mark'], surf=s['surf_z'], H=0.165, xy=np.array(s['xy']), trk=np.array(c['median']),
                        tag='B_%s_%s' % (s['mark'], s['height']), single=c.get('single_track'), n=c.get('n')))
    return obs
