"""Luring: deteksi ulang frame tersimpan (detektor & prior SAMA dengan capture) -> hitung frame lolos per kamera/muka."""
import json, sys, glob, collections, numpy as np, cv2
sys.path[:0] = ['/home/user1/Documents/ceiling_arm/docs/results/p1_g63/g63b', '/home/user1/Documents/ceiling_arm/docs/results/p1_g63']
import g63_detect, g63b_detect
from g63_common import T_from_xyzrpy
from g63b_capture import two_face_mount
_, mt = two_face_mount('../mount_g63b.json')
pid, cam, k = sys.argv[1], sys.argv[2], int(sys.argv[3])     # k = index kemunculan pose itu di poses.jsonl
R = [json.loads(l) for l in open('poses.jsonl') if json.loads(l)['id'] == pid][k]
c = R['cams'][cam]; K = np.array(c['K']); d = c['dist']; Twc = np.array(c['T_wc_live'])
pri = {b: g63_detect.predict_uv(b, np.array(c['T_wl'][b]['T']), T_from_xyzrpy(*mt[b]['xyz'], *mt[b]['rpy']), Twc, K, d, 0.042) for b in 'AB'}
fs = sorted(glob.glob(f'frames/{pid}/{cam}/*.png'), key=lambda f: float(f.split('_', 1)[1][:-4]))
fs = [f for f in fs if abs(float(f.split('_', 1)[1][:-4]) - R['t_wall']) < 15] or fs[30 * k:30 * k + 30]
cnt = collections.Counter()
for f in fs:
    r = g63b_detect.detect(cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY), pri, {'A': 0.042, 'B': 0.042})
    for b in 'AB':
        cnt[(b, 'OK' if r[b]['ids'] else r[b]['note'].split(' 0.')[0].split(' 1.')[0])] += 1
print(pid, cam, k, len(fs), dict(cnt))
