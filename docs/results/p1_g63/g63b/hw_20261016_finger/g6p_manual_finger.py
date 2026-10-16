"""G6' manual P0 (solve ditolak G10): PnP muka A di rgbd2 kedua R0 vs gerak papan FK (T_wl tercatat) dibawa ke frame kamera incumbent."""
import json, sys, numpy as np, cv2
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/docs/results/p1_g63')
from g63_common import board_points, T_from_xyzrpy
mt = json.load(open('../mount_finger.json'))['A']
R = [json.loads(l) for l in open('poses.jsonl') if json.loads(l)['id'] == 'A-R0']
P = np.array([board_points('A', 0.042)[i] for i in range(12)])
TA = T_from_xyzrpy(*mt['xyz'], *mt['rpy'])
out = []
for r in R:
    c = r['cams']['rgbd']; b = c['boards']['A']
    K = np.array(c['K']); d = np.array(c['dist'])
    ok, rv, tv = cv2.solvePnP(P[b['ids']], np.array(b['uv']), K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    Tcb = np.eye(4); Tcb[:3, :3] = cv2.Rodrigues(rv)[0]; Tcb[:3, 3] = tv.ravel()
    Twl = np.array(c['T_wl']['A']['T']); Twc = np.array(c['T_wc_live'])
    out.append((Tcb, np.linalg.inv(Twc) @ Twl @ TA))
ctr = np.r_[P.mean(0), 1]
d_cam = (out[1][0] @ ctr - out[0][0] @ ctr)[:3]
d_fk = (out[1][1] @ ctr - out[0][1] @ ctr)[:3]
print('gerak pusat papan (frame kamera) PnP mm', np.round(d_cam * 1e3, 2), '| FK mm', np.round(d_fk * 1e3, 2))
print("G6' tak-terjelaskan = %.2f mm (batas 2 mm); komponen lateral (tegak lurus sinar) = %.2f mm" % (
    np.linalg.norm(d_cam - d_fk) * 1e3,
    np.linalg.norm((d_cam - d_fk) - np.dot(d_cam - d_fk, out[0][0][:3, 3] / np.linalg.norm(out[0][0][:3, 3])) * out[0][0][:3, 3] / np.linalg.norm(out[0][0][:3, 3])) * 1e3))
print('jarak kamera %.3f m' % np.linalg.norm(out[0][0][:3, 3]))

# semua R0 vs R0#1: translasi + rotasi tak-terjelaskan
for k in range(1, len(R)):
    r = R[k]; c = r['cams']['rgbd']; b = c['boards']['A']
    if len(b['ids']) < 12: continue
    ok, rv, tv = cv2.solvePnP(P[b['ids']], np.array(b['uv']), np.array(c['K']), np.array(c['dist']))
    Tcb = np.eye(4); Tcb[:3, :3] = cv2.Rodrigues(rv)[0]; Tcb[:3, 3] = tv.ravel()
    Tfk = np.linalg.inv(np.array(c['T_wc_live'])) @ np.array(c['T_wl']['A']['T']) @ TA
    E = (Tcb @ np.linalg.inv(out[0][0])) @ np.linalg.inv(Tfk @ np.linalg.inv(out[0][1]))   # gerak kamera-terukur dikurangi FK
    ang = np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))
    dc = (Tcb @ ctr - out[0][0] @ ctr)[:3] - (Tfk @ ctr - out[0][1] @ ctr)[:3]
    print('R0#%d vs R0#1: tak-terjelaskan pusat %.2f mm, rotasi %.2f deg' % (k + 1, np.linalg.norm(dc) * 1e3, ang))
