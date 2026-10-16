"""LURING: M4 = papan kaku + offset nol 6 sendi arm_2 (FK pinocchio dari sudut terekam, cocok TF <= 0.16 mm).
Bila galat 2-15 mm per pose = galat kinematik lengan, M4 harus menurunkan sigma DAN galat uji-tahan/R0 dengan kamera stabil."""
import json, sys, numpy as np, pinocchio as pin
sys.path[:0] = ['/home/user1/Documents/ceiling_arm/docs/results/p1_g63']
import g63_solve as S
from g63_common import board_points
S.board_points = lambda b, sq=None: board_points('A', sq)
from scipy.optimize import least_squares
from numpy.linalg import inv
m = pin.buildModelFromUrdf('../hw_20261016/live2.urdf'); dd = m.createData(); fid = m.getFrameId('t1_a2_tool_frame')
ds = json.load(open('ds_finger.json'))
recs = [r for r in map(json.loads, open('poses.jsonl')) if r['joint_drift_during_capture_rad'] < 1e-3]
JN = [f't1_a2_joint_{i}' for i in range(1, 7)]
Q = []
for r in recs:
    q = pin.neutral(m)
    for n, v in r['joints'].items():
        if m.existJointName(n): q[m.joints[m.getJointId(n)].idx_q] = v
    Q.append(q)
IDX = [m.joints[m.getJointId(n)].idx_q for n in JN]
def fk(i, off):
    q = Q[i].copy(); q[IDX] += off; pin.framesForwardKinematics(m, dd, q); return dd.oMf[fid].homogeneous
class P(S.Problem):
    def __init__(s, ds, nj, roles=('FIT',)):
        super().__init__(ds, roles); s.nj = nj
    def res(s, x, obs=None, lateral=False):
        Twc, Tlb = s.unpack(x[:len(x) - s.nj] if s.nj else x)
        off = np.zeros(6); off[:s.nj] = x[len(x) - s.nj:] if s.nj else []
        r = []
        for i, cam, ids, xy, _f in (s.obs if obs is None else obs):
            b = s.ds['poses'][i]['board']; Pb = np.array([s.P[b][k] for k in ids])
            Tcb = inv(Twc[cam]) @ (fk(i, off) if s.nj else s.Twl[i]) @ Tlb[b]
            pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]; d = pc[:, :2] / pc[:, 2:3] - xy
            r.append(np.linalg.norm(d, axis=1) * pc[:, 2] if lateral else d.ravel())
        return np.concatenate(r)
base = None
for nj, name in ((0, 'M0 kaku'), (6, 'M4 offset 6 sendi')):
    pb = P(ds, nj); Twc0 = {c: S.incumbent_T(c) for c in pb.cams}; Tlb0 = S.initial_guess(pb, Twc0)
    sol = least_squares(pb.res, np.r_[pb.pack(Twc0, Tlb0), np.zeros(nj)], method='lm', x_scale='jac', max_nfev=40000)
    f = np.mean([o[4] for o in pb.obs]); n, k = sol.fun.size, sol.x.size
    sig = np.sqrt(sol.fun @ sol.fun / (n - k)) * f
    J = sol.jac; cov = np.linalg.pinv(J.T @ J) * (sol.fun @ sol.fun / (n - k)); sd = np.sqrt(np.diag(cov))
    ev = {}
    for role in ('HOLDOUT', 'R0'):
        pe = P(ds, nj, roles=(role,)); pe.boards = pb.boards; pe.P = pb.P
        lat = pe.res(sol.x, lateral=True); ev[role] = (np.median(lat) * 1e3, np.percentile(lat, 95) * 1e3)
    Twc, _ = pb.unpack(sol.x[:len(sol.x) - nj] if nj else sol.x)
    if base is None: base = Twc
    dc = {c: '%.2fdeg/%.1fcm' % (np.degrees(np.arccos(np.clip((np.trace(base[c][:3, :3].T @ Twc[c][:3, :3]) - 1) / 2, -1, 1))),
                                 np.linalg.norm(base[c][:3, 3] - Twc[c][:3, 3]) * 100) for c in Twc}
    sdc = {c: '%.3fdeg/%.1fmm' % (np.degrees(sd[6*j:6*j+3]).max(), sd[6*j+3:6*j+6].max()*1e3) for j, c in enumerate(pb.cams)}
    print('%-18s sigma %.2f px | uji-tahan med/p95 %.1f/%.1f mm | R0 med/p95 %.1f/%.1f mm | kamera vs M0 %s | std %s' % (
        name, sig, *ev['HOLDOUT'], *ev['R0'], dc, sdc))
    if nj: print('  offset sendi (deg):', np.round(np.degrees(sol.x[-nj:]), 3), '+-', np.round(np.degrees(sd[-nj:]), 3))
