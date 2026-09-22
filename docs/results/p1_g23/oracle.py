"""G23 A1: L2-torque feasibility oracle. OFFLINE, no ROS, no hardware.

Per (xyz, rail L, arm): IK_V (tool_frame at xyz, tool z-axis = world -z, roll
free -- what reach_dwell_probe asks the planner for) from 8 fixed seeds, then
the STATIC gravity torque of every converged solution. TORQ is evaluated from
the stored torques, so margins / mutants cost nothing (A2, A3).

    python3 oracle.py build   -> g23_oracle.npz  (task nodes of seeds 0-49 x 33 rails x 4 arms)
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, 'reach_dwell_live.urdf')      # = /tmp cache G22 screened with (A1)
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
GANTRY = {'arm_1': 1, 'arm_2': 1, 'arm_3': 2, 'arm_4': 2}
ARMS = tuple(PREFIX)
# reach_dwell_probe._plan_and_screen with --tau-max 12: j != 2 -> min(rating, 12)
LIM = np.array([10.0, 14.0, 10.0, 7.0, 7.0, 7.0])
OFFSET = np.array([6.6, 7.7, 6.6, 1.98, 1.98, 1.98])      # A8-1 g22
POS_TOL, ANG_TOL = 0.002, np.radians(2.0)                  # probe _goal_constraints
N_SEEDS, ITERS, STEP, LAM = 8, 300, 0.5, 1e-6
DOWN = np.array([0.0, 0.0, -1.0])                           # quat x=1,w=0 -> R=diag(1,-1,-1)

_M = {}


def _model():
    if not _M:
        import pinocchio as pin
        m = pin.buildModelFromUrdf(URDF)
        _M.update(pin=pin, m=m, d=m.createData())
        rng = np.random.default_rng(23)
        for a, p in PREFIX.items():
            ids = [m.getJointId(f'{p}joint_{i}') for i in range(1, 7)]
            iq = np.array([m.joints[j].idx_q for j in ids])
            iv = np.array([m.joints[j].idx_v for j in ids])
            lo, hi = m.lowerPositionLimit[iq], m.upperPositionLimit[iq]
            _M[a] = dict(iq=iq, iv=iv, lo=lo, hi=hi,
                         tool=m.getFrameId(f'{p}tool_frame'),
                         rail=m.joints[m.getJointId(f't{GANTRY[a]}_linear_joint')].idx_q)
        # same 7 random seeds for every arm and tuple (limits are identical per arm)
        lo, hi = _M['arm_1']['lo'], _M['arm_1']['hi']
        _M['seeds'] = [np.array(REST)] + [rng.uniform(lo, hi) for _ in range(N_SEEDS - 1)]
    return _M


def solve(xyz, rail, arm, pos_only=False):
    """-> (tau (N_SEEDS, 6) |gravity| at each converged solution, NaN if not
    converged; q (N_SEEDS, 6))."""
    M = _model()
    pin, m, d, A = M['pin'], M['m'], M['d'], M[arm]
    q = pin.neutral(m)
    for p in PREFIX.values():
        for i, v in enumerate(REST, 1):
            q[m.joints[m.getJointId(f'{p}joint_{i}')].idx_q] = v
    q[A['rail']] = rail
    tgt = np.asarray(xyz, float)
    taus = np.full((N_SEEDS, 6), np.nan)
    qs = np.full((N_SEEDS, 6), np.nan)
    for s, q0 in enumerate(M['seeds']):
        q[A['iq']] = q0
        ok = False
        for _ in range(ITERS):
            pin.framesForwardKinematics(m, d, q)
            T = d.oMf[A['tool']]
            ep = tgt - T.translation
            z = T.rotation[:, 2]
            ang = np.arccos(np.clip(z @ DOWN, -1.0, 1.0))
            if np.linalg.norm(ep) < POS_TOL and (pos_only or ang < ANG_TOL):
                ok = True
                break
            J = pin.computeFrameJacobian(m, d, q, A['tool'],
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, A['iv']]
            if pos_only:
                e, Jr = ep, J[:3]
            else:
                e, Jr = np.concatenate([ep, np.cross(z, DOWN)]), J
            dq = Jr.T @ np.linalg.solve(Jr @ Jr.T + LAM * np.eye(len(e)), e)
            q[A['iq']] = np.clip(q[A['iq']] + STEP * dq, A['lo'], A['hi'])
        if ok:
            g = pin.computeGeneralizedGravity(m, d, q)
            taus[s] = np.abs(g[A['iv']])
            qs[s] = q[A['iq']]
    return taus, qs


def torq_ok(taus, margin, lim=LIM, offset=OFFSET):
    """TORQ: exists a converged solution with |tau_j| + offset_j + m_j <= lim_j for all j."""
    t = np.asarray(taus)
    good = np.all(t + offset + margin <= lim, axis=-1)       # NaN rows -> False
    return bool(np.any(good, axis=-1)) if t.ndim == 2 else np.any(good, axis=-1)


def torq_all(taus, margin, lim=LIM, offset=OFFSET):
    """TORQ' (B0.1): >= 1 converged solution AND every converged one passes."""
    t = np.asarray(taus)
    conv = ~np.isnan(t[..., 0])
    good = np.all(t + offset + margin <= lim, axis=-1)
    return np.any(conv, axis=-1) & np.all(good | ~conv, axis=-1)


def static_at(q6, rail, arm):
    """|gravity| (6,), tool position, tool z-axis at a GIVEN arm configuration."""
    M = _model()
    pin, m, d, A = M['pin'], M['m'], M['d'], M[arm]
    q = pin.neutral(m)
    q[A['rail']] = rail
    q[A['iq']] = q6
    pin.framesForwardKinematics(m, d, q)
    T = d.oMf[A['tool']]
    g = pin.computeGeneralizedGravity(m, d, q)
    return np.abs(g[A['iv']]), T.translation.copy(), T.rotation[:, 2].copy()


def best_solution(taus):
    """s*: converged solution minimising max_j (|tau_j| + offset_j) / lim_j (A2). Index or None."""
    f = np.max((np.asarray(taus) + OFFSET) / LIM, axis=1)
    return None if np.all(np.isnan(f)) else int(np.nanargmin(f))


def _job(args):
    xyz, rail, arm = args
    return solve(xyz, rail, arm)[0]


def build(out=os.path.join(HERE, 'g23_oracle.npz'), procs=14):
    rows = json.load(open(os.path.join(HERE, '../p1_g22/g22_candidates.json')))
    nodes = sorted({n for r in rows for n in r['nodes']})
    sys.path.insert(0, os.path.join(HERE, '../../../ros2_ws/src/reachability_gng'))
    from reachability_gng.capability import CapabilityMap
    cap = CapabilityMap.load(os.path.join(HERE, '../../../ros2_ws/src/reachability_gng/data/cap_g1_rail160.npz'))
    xyz = cap.nodes[nodes]
    lins = cap.lin
    jobs = [(tuple(x), float(L), a) for x in xyz for L in lins for a in ARMS]
    with Pool(procs) as p:
        res = p.map(_job, jobs, chunksize=64)
    taus = np.array(res).reshape(len(nodes), len(lins), len(ARMS), N_SEEDS, 6)
    np.savez_compressed(out, nodes=np.array(nodes), xyz=xyz, lins=lins, arms=np.array(ARMS),
                        taus=taus)
    print(f'{len(jobs)} tuple -> {out}')


if __name__ == '__main__':
    if sys.argv[1:] == ['build']:
        build()
