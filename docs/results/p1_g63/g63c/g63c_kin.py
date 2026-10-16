"""G63c LURING tugas 2-3: perbandingan model kinematik + rank/degenerasi + validasi silang antar-lengan.
Data = ds_finger_all_noB13.json (B-13 dibuang: label salah, penjaga ID g63c_guard menolaknya). Alat terkunci tidak diubah.

Parameter: kamera T_wc 6+6, T_lb papan 6 per papan (gangguan), + parameter model ('extras'):
  ('j', nama, [sendi])     offset ditambahkan ke q semua sendi di daftar (satu parameter bisa dibagi = rotasi bersama)
  ('base', nama, papan, k) gangguan basis lengan papan itu, k = 0..5 (rx ry rz tx ty tz) di bingkai basis (penempatan j1, q1=0)
Residu = piksel (normalisasi x f). Lateral = |residu normalisasi| x kedalaman (m), seperti g63_solve.pose_errors.
"""
import json
import sys

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

from g63c_common import FING, PRE, model, data, q_of, records
import g63_solve as S
from g63_common import T_from_vec, vec_from_T, inv, incumbent_T, board_points

S.board_points = lambda b, sq=None: board_points('A', sq)   # pola polos A/B identik (= joint_offset_both.py), di proses ini saja

DS = json.load(open(FING + '/ds_finger_all_noB13.json'))
BOARD_ARM = {'AJ': 'A', 'BJ': 'B'}
SIG_FLOOR_PX = 0.3

# --- pasangkan pose ds dengan rekaman (urutan sama per id; = joint_offset_both.py) + cek FK == T_wl tercatat
_recs = [r for r in records() if r['joint_drift_during_capture_rad'] < 1e-3]
_by = {}
for r in _recs:
    _by.setdefault(r['id'], []).append(r)
_use, Q = {}, []
for p in DS['poses']:
    pid = p['id'].split(':')[0]
    Q.append(q_of(_by[pid][_use.get(pid, 0)]))
    _use[pid] = _use.get(pid, 0) + 1
JIDX = {n: model.joints[model.getJointId(n)].idx_q for n in model.names[1:]}
FID = {b: model.getFrameId(PRE[BOARD_ARM[b]] + 'tool_frame') for b in BOARD_ARM}
J1 = {b: model.getJointId(PRE[BOARD_ARM[b]] + 'joint_1') for b in BOARD_ARM}


def T_base(i, b):
    """Bingkai basis lengan (penempatan sendi 1 dengan q1 = 0) di dunia untuk pose i."""
    pin.forwardKinematics(model, data, Q[i])
    j = J1[b]
    return (data.oMi[model.parents[j]] * model.jointPlacements[j]).homogeneous


TB = [{b: T_base(i, b) for b in BOARD_ARM} for i in range(len(Q))]
for i, p in enumerate(DS['poses']):
    pin.framesForwardKinematics(model, data, Q[i])
    assert np.linalg.norm(data.oMf[FID[p['board']]].translation - np.array(p['T_wl'])[:3, 3]) < 1e-3, p['id']


def jn(b, k):
    return PRE[BOARD_ARM[b]] + f'joint_{k}'


class Fit:
    def __init__(self, extras, boards=('AJ', 'BJ'), roles=('FIT',), pose_filter=None, cams_fixed=None):
        self.ex = list(extras)
        self.boards = list(boards)
        self.obs = [o for o in S._norm_obs(DS) if DS['poses'][o[0]]['role'] in roles and DS['poses'][o[0]]['board'] in
                    self.boards and (pose_filter is None or pose_filter(o[0]))]
        self.cams = sorted({o[1] for o in self.obs})
        self.P = {b: board_points('A', DS['square_m'][b]) for b in self.boards}   # pola polos identik A/B
        self.cams_fixed = cams_fixed
        self.nc = 0 if cams_fixed else 6 * len(self.cams)

    # --------------------------------------------------------------------------------------------------- model
    def T_wl(self, i, b, xe):
        q = Q[i].copy()
        D, has_base = np.eye(4), False
        for (kind, _n, *a), v in zip(self.ex, xe):
            if kind == 'j':
                for n in a[0]:          # sendi di luar rantai papan b tidak berpengaruh pada FK-nya
                    q[JIDX[n]] += v
            elif kind == 'base' and a[0] == b:
                d = np.zeros(6)
                d[a[1]] = v
                D = D @ T_from_vec(d)
                has_base = True
        pin.framesForwardKinematics(model, data, q)
        T = data.oMf[FID[b]].homogeneous
        if has_base:
            T = TB[i][b] @ D @ inv(TB[i][b]) @ T
        return T

    def unpack(self, x):
        Twc = self.cams_fixed if self.cams_fixed else {c: T_from_vec(x[6 * k:6 * k + 6]) for k, c in enumerate(self.cams)}
        o = self.nc
        Tlb = {b: T_from_vec(x[o + 6 * k:o + 6 * k + 6]) for k, b in enumerate(self.boards)}
        return Twc, Tlb, x[o + 6 * len(self.boards):]

    def pred(self, x, obs):
        Twc, Tlb, xe = self.unpack(x)
        cache = {}
        out = []
        for i, cam, ids, xy, f in obs:
            b = DS['poses'][i]['board']
            if (i, b) not in cache:
                cache[(i, b)] = self.T_wl(i, b, xe)
            Pb = np.array([self.P[b][k] for k in ids])
            Tcb = inv(Twc[cam]) @ cache[(i, b)] @ Tlb[b]
            pc = Pb @ Tcb[:3, :3].T + Tcb[:3, 3]
            out.append((pc[:, :2] / pc[:, 2:3] - xy, pc[:, 2], f))
        return out

    def res(self, x):
        return np.concatenate([(d * f).ravel() for d, _z, f in self.pred(x, self.obs)])

    # --------------------------------------------------------------------------------------------------- solve
    def solve(self, x0=None):
        if x0 is None:
            Twc0 = self.cams_fixed or {c: incumbent_T(c) for c in self.cams}
            pb = S.Problem(DS, roles=('FIT', 'HOLDOUT', 'R0'))
            pb.obs = self.obs
            pb.boards = self.boards
            pb.P = self.P
            Tlb0 = S.initial_guess(pb, Twc0)
            x0 = np.r_[[] if self.cams_fixed else np.concatenate([vec_from_T(Twc0[c]) for c in self.cams]),
                       np.concatenate([vec_from_T(Tlb0[b]) for b in self.boards]), np.zeros(len(self.ex))]
        s = least_squares(self.res, x0, method='lm', x_scale='jac', max_nfev=200000)
        self.x, self.jac = s.x, s.jac
        n, k = s.fun.size, s.x.size
        self.dof = n - k
        self.sigma = float(np.sqrt(s.fun @ s.fun / max(n - k, 1)))
        return self

    def names(self):
        nm = [] if self.cams_fixed else [f'{c}.{a}' for c in self.cams for a in ('rx', 'ry', 'rz', 'tx', 'ty', 'tz')]
        nm += [f'Tlb_{b}.{a}' for b in self.boards for a in ('rx', 'ry', 'rz', 'tx', 'ty', 'tz')]
        return nm + [e[1] for e in self.ex]

    def rank(self):
        """Rank Jacobian berskala kolom (unit norm): nilai singular relatif + arah null terlemah (parameter dominan)."""
        J = self.jac / np.maximum(np.linalg.norm(self.jac, axis=0), 1e-30)
        U, s, Vt = np.linalg.svd(J, full_matrices=False)
        rel = s / s[0]
        v = Vt[-1]
        top = np.argsort(-np.abs(v))[:4]
        return dict(n=len(s), rank=int((rel > 1e-6).sum()), rel_min=float(rel[-1]), rel_2nd=float(rel[-2]),
                    null=[(self.names()[t], round(float(v[t]), 2)) for t in top])

    def std(self):
        s2 = max(self.sigma, SIG_FLOOR_PX) ** 2
        return np.sqrt(np.diag(np.linalg.pinv(self.jac.T @ self.jac)) * s2)

    def cam_std(self):
        sd = self.std()
        return {c: (float(np.degrees(sd[6 * k:6 * k + 3]).max()), float(sd[6 * k + 3:6 * k + 6].max())) for k, c in
                enumerate(self.cams)} if not self.cams_fixed else {}

    # --------------------------------------------------------------------------------------------------- evaluate
    def lateral(self, roles, boards=None, x=None):
        """Galat lateral per sudut (m) per kamera pada pose role tertentu (T_lb + model dari fit ini)."""
        obs = [o for o in S._norm_obs(DS) if DS['poses'][o[0]]['role'] in roles and
               DS['poses'][o[0]]['board'] in (boards or self.boards)]
        out = {}
        for (i, cam, *_), (d, z, _f) in zip(obs, self.pred(self.x if x is None else x, obs)):
            out.setdefault(cam, []).extend((np.linalg.norm(d, axis=1) * z).tolist())
        return {c: np.array(v) for c, v in out.items()}

    def g6p(self):
        """G6′ (§A4): T_lb dibekukan dari R0 PERTAMA tiap papan (kamera + model tetap), tiap R0 berikut lateral RMS <= 2 mm."""
        Twc, _Tlb, xe = self.unpack(self.x)
        out = {}
        for b in self.boards:
            r0 = [i for i, p in enumerate(DS['poses']) if p['role'] == 'R0' and p['board'] == b]
            if not r0:
                continue
            first = Fit(self.ex, boards=[b], roles=('R0',), pose_filter=lambda i, f=r0[0]: i == f,
                        cams_fixed={c: Twc[c] for c in Twc})
            pb = S.Problem(DS, roles=('R0',))
            pb.obs, pb.boards, pb.P = first.obs, [b], first.P
            x0 = np.r_[vec_from_T(S.initial_guess(pb, Twc)[b]), xe]
            # hanya T_lb bebas: extras dikunci ke nilai fit
            fun = lambda v: first.res(np.r_[v, xe])           # noqa: E731
            s = least_squares(fun, x0[:6], method='lm')
            xb = np.r_[s.x, xe]
            rms = []
            for i in r0[1:]:
                o = [ob for ob in S._norm_obs(DS) if ob[0] == i]
                e = np.concatenate([np.linalg.norm(d, axis=1) * z for d, z, _f in first.pred(xb, o)])
                rms.append(float(np.sqrt(np.mean(e ** 2))))
            out[b] = rms
        return out


def cam_diff(A, B):
    R = A[:3, :3].T @ B[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))), float(np.linalg.norm(A[:3, 3] - B[:3, 3]))
