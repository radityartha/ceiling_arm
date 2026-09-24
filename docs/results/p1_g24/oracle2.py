"""G24 A1: ORACLE'' -- IK_V with an EXPLICIT tool-roll sweep. OFFLINE, no ROS, no hardware.

Per (xyz, rail L, arm): full 6-D IK at every roll psi_k = 5 deg * k (72), branches
discovered from seed rounds (16, 16, 32, 64) at 12 anchor rolls and traced by
continuation in roll; saturated when a round adds no new solution. Stores the
max |gravity torque| per joint over EVERY solution at EVERY roll, so TORQ''
and the margin / limit mutants cost nothing.

Model: the G23 URDF (sha f02e7c53...), reduced to the arm's 6 joints + its rail
(everything else locked at the G23 reference: neutral, other arms REST).
G29 (docs/p1_g29_rot.md A4): optional `rot` locks the arm's gantry rotation joint at `rot` instead of 0;
default rot=0.0 leaves the reference, and therefore every result, unchanged (K0).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g23'))
import oracle as O  # noqa: E402  (URDF, REST, PREFIX, GANTRY, LIM, OFFSET, DOWN)

GRID_DEG = 5.0
ANCHOR_DEG = 30.0
ROUNDS = (16, 16, 32, 64)
DISC_ITERS, CONT_ITERS = 300, 50
POS_TOL, ROT_TOL = 5e-4, np.radians(0.5)
SAME = 0.01
LAM = 1e-6
D0 = np.diag([1.0, -1.0, -1.0])            # quat x=1, w=0

_M = {}


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def q_ref_full(m, pin, rot=0.0, gantry=None):
    """G23 oracle.solve reference: neutral, every arm at REST (rail set later)."""
    q = pin.neutral(m)
    for p in O.PREFIX.values():
        for i, v in enumerate(O.REST, 1):
            q[m.joints[m.getJointId(f'{p}joint_{i}')].idx_q] = v
    if rot != 0.0:
        q[m.joints[m.getJointId(f't{gantry}_rotation_joint')].idx_q] = rot
    return q


def model(arm, rot=0.0):
    key = arm if rot == 0.0 else (arm, float(rot))
    if key not in _M:
        import pinocchio as pin
        full = pin.buildModelFromUrdf(O.URDF)
        qr = q_ref_full(full, pin, rot, O.GANTRY[arm])
        keep = {f't{O.GANTRY[arm]}_linear_joint'} | {f'{O.PREFIX[arm]}joint_{i}' for i in range(1, 7)}
        lock = [j for j in range(1, full.njoints) if full.names[j] not in keep]
        m = pin.buildReducedModel(full, lock, qr)
        ids = [m.getJointId(f'{O.PREFIX[arm]}joint_{i}') for i in range(1, 7)]
        _M[key] = dict(pin=pin, m=m, d=m.createData(),
                       iq=np.array([m.joints[j].idx_q for j in ids]),
                       iv=np.array([m.joints[j].idx_v for j in ids]),
                       rail=m.joints[m.getJointId(f't{O.GANTRY[arm]}_linear_joint')].idx_q,
                       tool=m.getFrameId(f'{O.PREFIX[arm]}tool_frame'),
                       lo=m.lowerPositionLimit[[m.joints[j].idx_q for j in ids]],
                       hi=m.upperPositionLimit[[m.joints[j].idx_q for j in ids]])
    return _M[key]


def seed_rounds(lo, hi, rounds_n=ROUNDS):
    rng = np.random.default_rng(24)
    out, first = [], True
    for n in rounds_n:
        b = [rng.uniform(lo, hi) for _ in range(n - 1 if first else n)]
        out.append(([np.array(O.REST)] if first else []) + b)
        first = False
    return out


def gravity(arm, q6, rail, rot=0.0):
    A = model(arm, rot)
    pin, m, d = A['pin'], A['m'], A['d']
    q = np.zeros(m.nq)
    q[A['rail']] = rail
    q[A['iq']] = q6
    return np.abs(pin.computeGeneralizedGravity(m, d, q)[A['iv']])


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


TILT_DEG = [(2, 0), (-2, 0), (0, 2), (0, -2), (2, 2), (2, -2), (-2, 2), (-2, -2)]     # A'1
ENV_TOL = 0.01                                                                     # A'2


def solve(xyz, rail, arm, grid_deg=GRID_DEG, keep_solutions=False, tilt=False, envelope=False,
          rounds_n=ROUNDS, min_rounds=2, tilt_cont=False, rot=0.0):
    """tilt/envelope = ORACLE''' (A'1/A'2); defaults = ORACLE'' exactly.
    G27 (A2 T1/T2, defaults unchanged): rounds_n / min_rounds widen discovery; tilt_cont reaches
    each tilt by continuation 1/4 -> 2/4 -> 3/4 -> 1 of the angle instead of one Newton jump.
    G29: rot = the gantry rotation the tuple is solved at (default 0 = unchanged)."""
    A = model(arm, rot)
    pin, m, d = A['pin'], A['m'], A['d']
    lo, hi, iq, iv, tool = A['lo'], A['hi'], A['iq'], A['iv'], A['tool']
    K = int(round(360.0 / grid_deg))
    anchors = range(0, K, int(round(ANCHOR_DEG / grid_deg)))
    Rt = [D0 @ _rz(np.radians(grid_deg * k)) for k in range(K)]
    pt = np.asarray(xyz, float)
    q = np.zeros(m.nq)
    q[A['rail']] = rail
    eye = LAM * np.eye(6)

    def newton(q6, k, iters, Rk=None):
        Rk = Rt[k] if Rk is None else Rk
        q[iq] = q6
        for _ in range(iters):
            pin.framesForwardKinematics(m, d, q)
            T = d.oMf[tool]
            ep = pt - T.translation
            er = pin.log3(Rk @ T.rotation.T)
            if np.linalg.norm(ep) < POS_TOL and np.linalg.norm(er) < ROT_TOL:
                return q[iq].copy()
            J = pin.computeFrameJacobian(m, d, q, tool, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, iv]
            e = np.concatenate([ep, er])
            q[iq] = np.clip(q[iq] + J.T @ np.linalg.solve(J @ J.T + eye, e), lo, hi)
        return None

    sols = [[] for _ in range(K)]
    env = np.zeros(6)
    tilt_fail = [0]
    TR = [_rx(np.radians(a)) @ _ry(np.radians(b)) for a, b in TILT_DEG]
    TRS = [[_rx(np.radians(a * f)) @ _ry(np.radians(b * f)) for f in (0.25, 0.5, 0.75, 1.0)]
           for a, b in TILT_DEG]

    def add(k, s):
        nonlocal env
        for t in sols[k]:
            if np.max(np.abs(t - s)) < SAME:
                return False
        sols[k].append(s)
        if tilt:
            env = np.maximum(env, gravity(arm, s, rail, rot))
            for M, Ms in zip(TR, TRS):
                if tilt_cont:
                    st = s
                    for Mf in Ms:
                        st = newton(st, k, CONT_ITERS, Rt[k] @ Mf)
                        if st is None:
                            break
                else:
                    st = newton(s, k, CONT_ITERS, Rt[k] @ M)
                if st is None:
                    tilt_fail[0] += 1
                else:
                    env = np.maximum(env, gravity(arm, st, rail, rot))
        return True

    def trace(s, k):
        n = 0
        for step in (1, -1):
            kk, qq = k, s
            for _ in range(K - 1):
                kk = (kk + step) % K
                r = newton(qq, kk, CONT_ITERS)
                if r is None or not add(kk, r):
                    break
                n += 1
                qq = r
        return n

    rounds, saturated = 0, False
    for r, seeds in enumerate(seed_rounds(lo, hi, rounds_n)):
        rounds = r + 1
        new = 0
        env0, roll0 = env.copy(), sum(bool(ss) for ss in sols)
        for s0 in seeds:
            for k in anchors:
                s = newton(s0, k, DISC_ITERS)
                if s is not None and add(k, s):
                    new += 1 + trace(s, k)
        if envelope:
            done = (sum(bool(ss) for ss in sols) == roll0 and np.all(env - env0 <= ENV_TOL))
        else:
            done = new == 0
        if r >= min_rounds - 1 and done:
            saturated = True
            break

    taus = [gravity(arm, s, rail, rot) for ss in sols for s in ss]
    out = dict(n_sol=len(taus), n_roll=sum(bool(ss) for ss in sols), rounds=rounds,
               saturated=saturated,
               taumax=(np.max(taus, axis=0) if taus else np.full(6, np.nan)))
    if tilt:
        out['taumax'] = env.copy() if taus else np.full(6, np.nan)
        out['tilt_fail'] = tilt_fail[0]
    if keep_solutions:
        out['solutions'] = [(k, s) for k, ss in enumerate(sols) for s in ss]
    return out


def ik_v(res):
    return res['n_sol'] > 0


def torq2(res, margin, lim=O.LIM, offset=O.OFFSET):
    """TORQ'': saturated AND every solution at every roll within limits."""
    return bool(res['saturated'] and res['n_sol'] > 0 and
                np.all(res['taumax'] + offset + margin <= lim))
