"""LURING (bukan alat terkunci): uji model dudukan non-kaku pada data jari arm_2.
M0 kaku (= g63_solve); M1 rotasi papan ~ (jari - f0); M2 rotasi ~ A g_link (gravitasi di frame link, 9 param);
M3 = M2 + translasi B g_link. Dinilai: sigma px (FIT), galat lateral uji-tahan + R0 (mm), estimasi kamera vs M0."""
import json, sys, numpy as np, cv2
sys.path[:0] = ['/home/user1/Documents/ceiling_arm/docs/results/p1_g63']
import g63_solve as S
from g63_common import T_from_vec, vec_from_T, board_points
S.board_points = lambda b, sq=None: board_points("A", sq)   # = g63b_solve
from scipy.optimize import least_squares
from numpy.linalg import inv
ds = json.load(open('ds_finger.json'))
# sudut jari per entri ds (urutan poses.jsonl yang lolos merge = urutan ds)
fing = {}
recs = [json.loads(l) for l in open('poses.jsonl')]
recs = [r for r in recs if r['joint_drift_during_capture_rad'] < 1e-3]
assert len(recs) == len(ds['poses']), (len(recs), len(ds['poses']))
F = np.array([r['joints']['t1_a2_right_finger_bottom_joint'] for r in recs]); f0 = F.mean()
def rotv(v):
    return cv2.Rodrigues(np.asarray(v, float).reshape(3, 1))[0]
class P(S.Problem):
    def __init__(s, ds, model, roles=('FIT',)):
        super().__init__(ds, roles); s.model = model
        s.next = {'M0': 0, 'M1': 3, 'M2': 9, 'M3': 18}[model]
    def Tlb_i(s, Tlb0, i, e):
        if s.model == 'M0': return Tlb0
        g = s.Twl[i][:3, :3].T @ np.array([0, 0, -1.0])
        if s.model == 'M1': d, t = e[:3] * (F[i] - f0), np.zeros(3)
        else: d, t = e[:9].reshape(3, 3) @ g, (e[9:18].reshape(3, 3) @ g if s.model == 'M3' else np.zeros(3))
        D = np.eye(4); D[:3, :3] = rotv(d); D[:3, 3] = t
        return D @ Tlb0          # di frame link (asal link = sumbu alat)
    def res(s, x, obs=None, lateral=False):
        Twc, Tlb = s.unpack(x[:len(x) - s.next] if s.next else x); e = x[len(x) - s.next:]
        r = []
        for i, cam, ids, xy, _f in (s.obs if obs is None else obs):
            b = s.ds['poses'][i]['board']; Pb = np.array([s.P[b][k] for k in ids])
            Tcb = inv(Twc[cam]) @ s.Twl[i] @ s.Tlb_i(Tlb[b], i, e)
            pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]
            d = pc[:, :2] / pc[:, 2:3] - xy
            r.append(np.linalg.norm(d, axis=1) * pc[:, 2] if lateral else d.ravel())
        return np.concatenate(r)
base = None
for m in ('M0', 'M1', 'M2', 'M3'):
    pb = P(ds, m)
    Twc0 = {c: S.incumbent_T(c) for c in pb.cams}; Tlb0 = S.initial_guess(pb, Twc0)
    x0 = np.r_[pb.pack(Twc0, Tlb0), np.zeros(pb.next)]
    sol = least_squares(pb.res, x0, method='lm', x_scale='jac', max_nfev=40000)
    f = np.mean([o[4] for o in pb.obs])
    sig = np.sqrt(sol.fun @ sol.fun / max(sol.fun.size - sol.x.size, 1)) * f
    ev = {}
    for role in ('HOLDOUT', 'R0'):
        pe = P(ds, m, roles=(role,)); pe.boards = pb.boards; pe.P = pb.P
        lat = pe.res(sol.x, lateral=True); ev[role] = (np.median(lat) * 1e3, np.percentile(lat, 95) * 1e3)
    Twc, _ = pb.unpack(sol.x[:len(sol.x) - pb.next] if pb.next else sol.x)
    if base is None: base = Twc
    dcam = {c: (np.degrees(np.arccos(np.clip((np.trace(base[c][:3, :3].T @ Twc[c][:3, :3]) - 1) / 2, -1, 1))),
                np.linalg.norm(base[c][:3, 3] - Twc[c][:3, 3]) * 100) for c in Twc}
    print('%s param %2d  sigma %.2f px | holdout med/p95 %.1f/%.1f mm | R0 med/p95 %.1f/%.1f mm | kamera vs M0 %s' % (
        m, sol.x.size, sig, *ev['HOLDOUT'], *ev['R0'], {c: '%.2fdeg/%.1fcm' % v for c, v in dcam.items()}))
