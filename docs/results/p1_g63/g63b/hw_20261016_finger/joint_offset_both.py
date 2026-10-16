"""LURING: papan kaku + offset nol sendi untuk arm_2 (AJ) DAN arm_3 (BJ), data gabungan tanpa B-13 (label salah).
Uji: apakah kedua lengan sepakat pada kamera bila galat kinematik tiap lengan dimodelkan."""
import json, sys, numpy as np, pinocchio as pin
sys.path[:0] = ['/home/user1/Documents/ceiling_arm/docs/results/p1_g63']
import g63_solve as S
from g63_common import board_points
S.board_points = lambda b, sq=None: board_points('A', sq)
from scipy.optimize import least_squares
from numpy.linalg import inv
m = pin.buildModelFromUrdf('../hw_20261016/live2.urdf'); dd = m.createData()
PRE = {'AJ': 't1_a2_', 'BJ': 't2_a1_'}
ds = json.load(open(sys.argv[1] if len(sys.argv) > 1 else 'ds_finger_all_noB13.json'))
recs = [r for r in map(json.loads, open('poses.jsonl')) if r['joint_drift_during_capture_rad'] < 1e-3]
# pasangkan entri ds dengan rekaman poses.jsonl (urutan sama per id; B-13 dibuang)
byid = {}
for r in recs: byid.setdefault(r['id'], []).append(r)
use = {}
Q = []
for p in ds['poses']:
    pid = p['id'].split(':')[0]; r = byid[pid][use.get(pid, 0)]; use[pid] = use.get(pid, 0) + 1
    q = pin.neutral(m)
    for n, v in r['joints'].items():
        if m.existJointName(n): q[m.joints[m.getJointId(n)].idx_q] = v
    Q.append(q)
IDX = {b: [m.joints[m.getJointId(f'{pre}joint_{i}')].idx_q for i in range(1, 7)] for b, pre in PRE.items()}
FID = {b: m.getFrameId(f'{pre}tool_frame') for b, pre in PRE.items()}
for i, p in enumerate(ds['poses']):     # cek pasangan: FK == TF tercatat
    pin.framesForwardKinematics(m, dd, Q[i]); assert np.linalg.norm(dd.oMf[FID[p['board']]].translation - np.array(p['T_wl'])[:3, 3]) < 1e-3, p['id']
def fk(i, b, off):
    q = Q[i].copy(); q[IDX[b]] += off; pin.framesForwardKinematics(m, dd, q); return dd.oMf[FID[b]].homogeneous
class P(S.Problem):
    def __init__(s, ds, nj, roles=('FIT',)):
        super().__init__(ds, roles); s.nj = nj
    def split(s, x):
        k = len(x) - s.nj * len(s.boards)
        off = {b: (np.r_[x[k + s.nj * j:k + s.nj * j + s.nj], np.zeros(6 - s.nj)] if s.nj else np.zeros(6)) for j, b in enumerate(s.boards)}
        return s.unpack(x[:k]), off
    def res(s, x, obs=None, lateral=False):
        (Twc, Tlb), off = s.split(x); r = []
        for i, cam, ids, xy, _f in (s.obs if obs is None else obs):
            b = s.ds['poses'][i]['board']; Pb = np.array([s.P[b][k] for k in ids])
            Tcb = inv(Twc[cam]) @ (fk(i, b, off[b]) if s.nj else s.Twl[i]) @ Tlb[b]
            pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]; d = pc[:, :2] / pc[:, 2:3] - xy
            r.append(np.linalg.norm(d, axis=1) * pc[:, 2] if lateral else d.ravel())
        return np.concatenate(r)
for nj, name in ((0, 'kaku'), (5, 'offset j1-j5/lengan')):   # j6 = setara T_lb (tak teridentifikasi) -> 0
    pb = P(ds, nj); Twc0 = {c: S.incumbent_T(c) for c in pb.cams}; Tlb0 = S.initial_guess(pb, Twc0)
    x0 = np.r_[pb.pack(Twc0, Tlb0), np.zeros(nj * len(pb.boards))]
    sol = least_squares(pb.res, x0, method='lm', x_scale='jac', max_nfev=60000)
    n, k = sol.fun.size, sol.x.size; f = np.mean([o[4] for o in pb.obs]); s2 = sol.fun @ sol.fun / (n - k)
    sd = np.sqrt(np.diag(np.linalg.pinv(sol.jac.T @ sol.jac) * s2))
    ev = {}
    for role in ('HOLDOUT', 'R0'):
        pe = P(ds, nj, roles=(role,)); pe.boards = pb.boards; pe.P = pb.P
        lat = pe.res(sol.x, lateral=True); ev[role] = (np.median(lat) * 1e3, np.percentile(lat, 95) * 1e3)
    (Twc, Tlb), off = pb.split(sol.x)
    inc = {c: S.incumbent_T(c) for c in Twc}
    vs = {c: '%.2fdeg/%.1fcm' % (np.degrees(np.arccos(np.clip((np.trace(inc[c][:3, :3].T @ Twc[c][:3, :3]) - 1) / 2, -1, 1))),
                                 np.linalg.norm(inc[c][:3, 3] - Twc[c][:3, 3]) * 100) for c in Twc}
    sdc = {c: '%.3fdeg/%.1fmm' % (np.degrees(sd[6*j:6*j+3]).max(), sd[6*j+3:6*j+6].max()*1e3) for j, c in enumerate(pb.cams)}
    print('%-22s sigma %.2f px | uji-tahan med/p95 %.1f/%.1f mm | R0 med/p95 %.1f/%.1f mm | vs incumbent %s | std %s' % (
        name, np.sqrt(s2) * f, *ev['HOLDOUT'], *ev['R0'], vs, sdc))
    if nj:
        k0 = len(sol.x) - nj * len(pb.boards)
        for j, b in enumerate(pb.boards):
            print('  %s offset j1-j5 (deg): %s +- %s' % (PRE[b], np.round(np.degrees(off[b][:nj]), 2), np.round(np.degrees(sd[k0+nj*j:k0+nj*j+nj]), 2)))
