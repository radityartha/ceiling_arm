"""Luring: semua R0 (rgbd, muka A) -> gerak papan tak-terjelaskan FK (PnP) vs perubahan sudut jari. Uji model 'papan ikut jari'."""
import json, sys, numpy as np, cv2
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/docs/results/p1_g63')
from g63_common import board_points, T_from_xyzrpy
mt = json.load(open('../mount_finger.json'))['A']; TA = T_from_xyzrpy(*mt['xyz'], *mt['rpy'])
P = np.array([board_points('A', 0.042)[i] for i in range(12)]); ctr = np.r_[P.mean(0), 1]
R = [json.loads(l) for l in open('poses.jsonl') if json.loads(l)['id'] == 'A-R0']
rows = []
for r in R:
    c = r['cams']['rgbd']; b = c['boards']['A']
    if len(b['ids']) < 12: continue
    ok, rv, tv = cv2.solvePnP(P[b['ids']], np.array(b['uv']), np.array(c['K']), np.array(c['dist']))
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(rv)[0]; T[:3, 3] = tv.ravel()
    Tfk = np.linalg.inv(np.array(c['T_wc_live'])) @ np.array(c['T_wl']['A']['T']) @ TA
    rows.append((r['joints']['t1_a2_right_finger_bottom_joint'], T, Tfk, r['stage']))
f0, T0, F0, _ = rows[0]
print('stage  jari     dJari(deg)  tak-terjelaskan: pusat mm (x,y,z kamera)   |d| mm  rot deg')
for f, T, F, st in rows:
    E = (T @ np.linalg.inv(T0)) @ np.linalg.inv(F @ np.linalg.inv(F0))
    d = (T @ ctr - T0 @ ctr)[:3] - (F @ ctr - F0 @ ctr)[:3]
    ang = np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))
    print('%-5s %.4f  %+6.2f      %s   %5.2f   %.2f' % (st, f, np.degrees(f - f0), np.round(d * 1e3, 2), np.linalg.norm(d) * 1e3, ang))
