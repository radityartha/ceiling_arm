"""G27 step 3 -- offline diagnosis, section A2 of docs/p1_g27_z140_diag.md (locked 16:23,
sha 49e1d109...). IK / gravity only, no ROS, zero hardware.

    python3 g27_diag.py k0      K0: oracle2 defaults reproduce g24_oracle3_cache (72 tuples)
    python3 g27_diag.py tmodel  T-model (H4b)
    python3 g27_diag.py t0      T0: max |tau_2| over the planner goal region (E vs N)
    python3 g27_diag.py t1      T1: tilt continuation (H1)
    python3 g27_diag.py t2      T2: wider discovery, 1 deg roll (H2)
    python3 g27_diag.py t3      T3a (proxy on C') + T3b (straight-path proxy per layer)
Each writes g27_<cmd>.json next to this file.
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g24'))
import make_instance_g24 as MI4  # noqa: E402
import oracle2 as O2  # noqa: E402

O = O2.O
REF = MI4.REF
MARGIN = MI4.MARGIN
ARM_GS = {v: k for k, v in MI4.SLOT_ARM.items()}
M2 = float(MARGIN[1])
TABLE = json.load(open(os.path.join(HERE, 'g27_table.json')))['rows']
FIELDS = ('n_sol', 'n_roll', 'rounds', 'saturated', 'taumax', 'tilt_fail', 'ok')
NPROC = 15


def arm_of(key):
    return MI4.SLOT_ARM[(key[1], key[3])]


def xyz_rail(key):
    return REF.nodes[key[0]], float(REF.lin[key[2]])


def s16_rows():
    return [r for r in TABLE if r['src'] == 'G26' and r['z'] == 1.4 and r['sample'] != 'exec']


def s12():
    return sorted({(r['node'], r['g'], r['p'], r['s']) for r in s16_rows()})


def layer_controls(cache):
    """A2: 12 ok tuples per z in {1.00..1.32}; planned-PLANNED ones first, then default_rng(27)."""
    rng = np.random.default_rng(27)
    planned = {(r['node'], r['g'], r['p'], r['s']) for r in TABLE if r['verdict'] in ('PLANNED', 'SUCCESS')}
    out = {}
    for z in (1.0, 1.08, 1.16, 1.24, 1.32):
        ok = sorted(k for k, d in cache.items() if d['ok'] and abs(REF.nodes[k[0]][2] - z) < 1e-6)
        pl = [k for k in ok if k in planned]
        rest = [k for k in ok if k not in planned]
        pick = [pl[i] for i in sorted(rng.choice(len(pl), 12, replace=False))] if len(pl) >= 12 else \
            pl + [rest[i] for i in sorted(rng.choice(len(rest), 12 - len(pl), replace=False))]
        out[z] = pick
    return out


def _solve_job(args):
    key, kw = args
    xyz, rail = xyz_rail(key)
    r = O2.solve(xyz, rail, arm_of(key), tilt=True, envelope=True, **kw)
    d = dict(n_sol=r['n_sol'], n_roll=r['n_roll'], rounds=r['rounds'], saturated=r['saturated'],
             taumax=r['taumax'].tolist(), tilt_fail=r['tilt_fail'], ok=bool(O2.torq2(r, MARGIN)))
    if kw.get('keep_solutions'):
        d['solutions'] = [(int(k), s.tolist()) for k, s in r['solutions']]
    return key, d


# ------------------------------------------------------------------ K0
def _same(a, b):
    """Exact equality; NaN == NaN (taumax of an n_sol = 0 tuple is all-NaN in the cache too)."""
    if isinstance(a, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, float) and np.isnan(a) and np.isnan(b):
        return True
    return a == b


def k0():
    cache = MI4.load_cache()
    keys = sorted(cache)
    rng = np.random.default_rng(27)
    pick = [keys[i] for i in sorted(rng.choice(len(keys), 60, replace=False))] + s12()
    with Pool(NPROC) as pool:
        res = dict(pool.map(_solve_job, [(k, dict(keep_solutions=True)) for k in pick], chunksize=1))
    bad = [k for k in pick if any(not _same(res[k][f], cache[k][f]) for f in FIELDS)]
    print(f'K0: default oracle2 == g24_oracle3_cache pada {len(pick) - len(bad)}/{len(pick)} tuple; beda {bad}')
    json.dump(dict(pick=[list(k) for k in pick], bad=[list(k) for k in bad],
                   sol={json.dumps(list(k)): res[k]['solutions'] for k in pick}),
              open(os.path.join(HERE, 'g27_k0.json'), 'w'))
    return not bad


def solutions_for(keys):
    """Untilted grid solutions of oracle''' (defaults) -- from g27_k0.json, else solved now."""
    have = {}
    p = os.path.join(HERE, 'g27_k0.json')
    if os.path.exists(p):
        have = {tuple(json.loads(k)): v for k, v in json.load(open(p))['sol'].items()}
    todo = [k for k in keys if k not in have]
    if todo:
        with Pool(NPROC) as pool:
            for k, d in pool.imap_unordered(_solve_job, [(k, dict(keep_solutions=True)) for k in todo]):
                have[k] = d['solutions']
    return {k: have[k] for k in keys}


# ------------------------------------------------------------------ T-model
def tmodel():
    import pinocchio as pin
    full = pin.buildModelFromUrdf(O.URDF)
    fd = full.createData()
    rng = np.random.default_rng(27)
    worst = 0.0
    for arm in ('arm_1', 'arm_2', 'arm_3', 'arm_4'):
        A = O2.model(arm)
        ids = [full.getJointId(f'{O.PREFIX[arm]}joint_{i}') for i in range(1, 7)]
        iq = [full.joints[j].idx_q for j in ids]
        iv = [full.joints[j].idx_v for j in ids]
        for _ in range(50):
            q6 = rng.uniform(A['lo'], A['hi'])
            rail = rng.uniform(0.0, 1.6)
            q = pin.neutral(full)                   # probe predict_peak_torque: neutral + arm joints
            q[iq] = q6
            tp = np.abs(pin.rnea(full, fd, q, np.zeros(full.nv), np.zeros(full.nv))[iv])
            to = O2.gravity(arm, q6, rail)
            worst = max(worst, float(np.max(np.abs(tp - to))))
    print(f'T-model: maks |tau probe-statis - tau oracle| = {worst:.3e} N.m  (H4b benar bila > 0.01)')
    json.dump(dict(max_abs=worst), open(os.path.join(HERE, 'g27_tmodel.json'), 'w'))


# ------------------------------------------------------------------ T0
def _fk(A, q6, rail):
    pin, m, d = A['pin'], A['m'], A['d']
    q = np.zeros(m.nq)
    q[A['rail']] = rail
    q[A['iq']] = q6
    pin.framesForwardKinematics(m, d, q)
    T = d.oMf[A['tool']]
    return T.translation.copy(), T.rotation[:, 2].copy()


def _maxregion(arm, xyz, rail, starts, pos_tol, ang_deg):
    """max |tau_2| over {q in limits: |p - xyz| <= pos_tol, angle(z_tool, -z) <= ang}; SLSQP multistart.
    Constraints in mm / scaled cosine (run 1 used m^2 = 2.5e-7, below SLSQP's tolerance -> every
    end point slightly infeasible and discarded). Feasible starts count as candidates themselves.
    Returns (best, q_best, n_feasible_endpoints); best = -1 when nothing feasible was found."""
    import warnings
    from scipy.optimize import minimize
    A = O2.model(arm)
    lo, hi = A['lo'], A['hi']
    pt = np.asarray(xyz, float)
    cmin = np.cos(np.radians(ang_deg))
    tol_mm = pos_tol * 1e3

    def feas(q):
        p, z = _fk(A, q, rail)
        return np.linalg.norm(p - pt) <= pos_tol + 1e-7 and -z[2] >= cmin - 1e-9

    cons = [dict(type='ineq', fun=lambda q: tol_mm ** 2 - np.sum(((_fk(A, q, rail)[0] - pt) * 1e3) ** 2)),
            dict(type='ineq', fun=lambda q: (-_fk(A, q, rail)[1][2] - cmin) * 1e3)]
    best, qb, nf = -1.0, None, 0
    for s in starts:
        s = np.clip(np.asarray(s, float), lo, hi)
        cand = [s]
        try:
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                r = minimize(lambda q: -O2.gravity(arm, q, rail)[1], s, method='SLSQP',
                             bounds=list(zip(lo, hi)), constraints=cons,
                             options=dict(maxiter=300, ftol=1e-10))
            cand.append(np.clip(r.x, lo, hi))
        except Exception:                                    # noqa: BLE001
            pass
        for q in cand:
            if feas(q):
                nf += 1
                v = float(O2.gravity(arm, q, rail)[1])
                if v > best:
                    best, qb = v, q
    return best, qb, nf


def _random_starts(arm, xyz, rail, n=64):
    """A2: 64 default_rng(27) seeds projected by 6-D Newton (A1 g24 tolerances) at a random roll."""
    A = O2.model(arm)
    pin, m, d = A['pin'], A['m'], A['d']
    rng = np.random.default_rng(27)
    pt = np.asarray(xyz, float)
    out = []
    for _ in range(n):
        q6 = rng.uniform(A['lo'], A['hi'])
        Rk = O2.D0 @ O2._rz(rng.uniform(0, 2 * np.pi))
        q = np.zeros(m.nq)
        q[A['rail']] = rail
        q[A['iq']] = q6
        for _ in range(O2.DISC_ITERS):
            pin.framesForwardKinematics(m, d, q)
            T = d.oMf[A['tool']]
            e = np.concatenate([pt - T.translation, pin.log3(Rk @ T.rotation.T)])
            if np.linalg.norm(e[:3]) < O2.POS_TOL and np.linalg.norm(e[3:]) < O2.ROT_TOL:
                out.append(q[A['iq']].copy())
                break
            J = pin.computeFrameJacobian(m, d, q, A['tool'], pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, A['iv']]
            q[A['iq']] = np.clip(q[A['iq']] + J.T @ np.linalg.solve(J @ J.T + O2.LAM * np.eye(6), e),
                                 A['lo'], A['hi'])
    return out


def _t0_job(args):
    key, sols = args
    arm = arm_of(key)
    xyz, rail = xyz_rail(key)
    starts = [np.array(s) for _, s in sols] + _random_starts(arm, xyz, rail)
    g0, q0, n0 = _maxregion(arm, xyz, rail, starts, 5e-4, 0.5)
    gt, qt, nt = _maxregion(arm, xyz, rail, starts + ([q0] if q0 is not None else []), 5e-4, 2.83)
    go, qo, no = _maxregion(arm, xyz, rail, starts + [q for q in (q0, qt) if q is not None], 2e-3, 2.83)
    go = max(go, gt, g0)
    gt = max(gt, g0)
    return key, dict(G_0=g0, G_t=gt, G_Omega=go, n_starts=len(starts), n_feas=[n0, nt, no],
                     q_Omega=None if qo is None else qo.tolist())


def t0():
    keys = s12()
    sols = solutions_for(keys)
    with Pool(NPROC) as pool:
        res = dict(pool.map(_t0_job, [(k, sols[k]) for k in keys], chunksize=1))
    cache = MI4.load_cache()
    rows = []
    for r in s16_rows():
        k = (r['node'], r['g'], r['p'], r['s'])
        g = res[k]
        cls = None
        if r['verdict'] == 'TORQUE-UNSAFE':
            if g['G_Omega'] < 0:
                cls = 'TIDAK-DITENTUKAN'
            elif g['G_Omega'] < r['rnea2'] - M2:
                cls = 'N'
            elif g['G_0'] >= r['rnea2'] - M2:
                cls = 'E:H2'
            elif g['G_t'] >= r['rnea2'] - M2:
                cls = 'E:H1'
            else:
                cls = 'E:H4a'
        rows.append(dict(seed=r['seed'], sample=r['sample'], task=r['task'], arm=r['arm'], rail=r['rail'],
                         verdict=r['verdict'], rnea2=r['rnea2'], o_j2=cache[k]['taumax'][1], **g, cls=cls))
        print(f"s{r['seed']:2d} sm{r['sample']} t{r['task']} {r['arm']} @{r['rail']:.2f} {r['verdict']:14s} "
              f"RNEA {r['rnea2']:5.2f}  o_j2 {cache[k]['taumax'][1]:5.2f}  G_0 {g['G_0']:5.2f}  "
              f"G_t {g['G_t']:5.2f}  G_Om {g['G_Omega']:5.2f}  feas {g['n_feas']}  -> {cls}")
    un = [x for x in rows if x['verdict'] == 'TORQUE-UNSAFE']
    from collections import Counter
    print('T0 kelas TORQUE:', dict(Counter(x['cls'] for x in un)), f'/ {len(un)}')
    json.dump(dict(tuples={json.dumps(list(k)): v for k, v in res.items()}, rows=rows),
              open(os.path.join(HERE, 'g27_t0.json'), 'w'), indent=1)


# ------------------------------------------------------------------ T1 / T2
def _t12(kw, name):
    keys = s12()
    cache = MI4.load_cache()
    with Pool(NPROC) as pool:
        res = dict(pool.map(_solve_job, [(k, kw) for k in keys], chunksize=1))
    out = []
    for k in keys:
        a, b = cache[k], res[k]
        d2 = b['taumax'][1] - a['taumax'][1]
        out.append(dict(key=list(k), z=float(REF.nodes[k[0]][2]), old_j2=a['taumax'][1], new_j2=b['taumax'][1],
                        d_j2=d2, old_tilt_fail=a['tilt_fail'], new_tilt_fail=b['tilt_fail'],
                        old_n_sol=a['n_sol'], new_n_sol=b['n_sol'], rounds=b['rounds'],
                        saturated=b['saturated'], ok_new=b['ok']))
        print(f"{name} {arm_of(k)} node {k[0]} @{REF.lin[k[2]]:.2f}: j2 {a['taumax'][1]:.3f} -> {b['taumax'][1]:.3f} "
              f"(d {d2:+.3f}); tilt_fail {a['tilt_fail']} -> {b['tilt_fail']}; n_sol {a['n_sol']} -> {b['n_sol']}; "
              f"rounds {b['rounds']} sat {b['saturated']} ok {b['ok']}")
    n = sum(x['d_j2'] > 0.30 for x in out)
    print(f'{name}: amplop j2 naik > 0.30 pada {n}/{len(out)} tuple')
    json.dump(out, open(os.path.join(HERE, f'g27_{name}.json'), 'w'), indent=1)


def t1():
    _t12(dict(tilt_cont=True), 't1')


def t2():
    _t12(dict(grid_deg=1.0, rounds_n=(16, 16, 32, 64, 128, 256), min_rounds=4), 't2')


# ------------------------------------------------------------------ T3
REST = np.array(O.REST)


def path_max(arm, q_end, rail, start=REST, n=101):
    return np.max([O2.gravity(arm, start + (q_end - start) * t, rail) for t in np.linspace(0, 1, n)], axis=0)


def _t3b_job(args):
    key, sols = args
    arm = arm_of(key)
    _, rail = xyz_rail(key)
    P = [path_max(arm, np.array(s), rail) for _, s in sols]
    E = [O2.gravity(arm, np.array(s), rail) for _, s in sols]
    return key, dict(P=[p.tolist() for p in P], E=[e.tolist() for e in E])


def t3():
    v3 = json.load(open(os.path.join(R, 'p1_g24/g24_validate3.json')))
    C = v3['C']
    diff_p, diff_e, mp = [], [], np.zeros(6)
    for c in C:
        arm = c['arm']
        P = path_max(arm, np.array(c['q_final']), c['rail'])
        rn = np.array(c['rnea'])
        diff_p.append(rn[1] - P[1])
        diff_e.append(c['delta_final'][1])
        mp = np.maximum(mp, rn - P)
    mp = np.maximum(mp, 0.0)
    diff_p, diff_e = np.array(diff_p), np.array(diff_e)
    feasible = bool(diff_p.max() <= M2 + 1e-12)
    print(f"T3a C' (n={len(C)}): RNEA2 - P2 median {np.median(diff_p):+.3f} maks {diff_p.max():+.3f};  "
          f"RNEA2 - statis_akhir2 median {np.median(diff_e):+.3f} maks {diff_e.max():+.3f}  -> "
          f"proksi {'LAYAK' if feasible else 'TIDAK LAYAK'}; m_p = {np.round(mp, 4).tolist()}")
    zs = [c['xyz'][2] for c in C]
    for z in sorted(set(zs)):
        dp = diff_p[np.isclose(zs, z)]
        print(f'    z {z:.2f} n {len(dp):2d}  RNEA2-P2 median {np.median(dp):+.3f} maks {dp.max():+.3f}')

    cache = MI4.load_cache()
    ctl = layer_controls(cache)
    keys = s12() + [k for z in ctl for k in ctl[z]]
    sols = solutions_for(keys)
    with Pool(NPROC) as pool:
        res = dict(pool.map(_t3b_job, [(k, sols[k]) for k in keys], chunksize=1))
    bar = 14.0 - 7.7 - mp[1]
    print(f'T3b: proksi lintasan tak aman <=> P2 > {bar:.3f} (14 - 7.7 - m2p)')
    layers = {1.4: s12(), **ctl}
    summ = {}
    for z in sorted(layers):
        P = np.array([p[1] for k in layers[z] for p in res[k]['P']])
        E = np.array([e[1] for k in layers[z] for e in res[k]['E']])
        tup_bad = sum(any(p[1] > bar for p in res[k]['P']) for k in layers[z])
        summ[z] = dict(n_tuples=len(layers[z]), n_sol=len(P), frac_sol_unsafe=float(np.mean(P > bar)),
                       tuples_any_unsafe=tup_bad, P2_median=float(np.median(P)), P2_max=float(P.max()),
                       E2_max=float(E.max()), path_over_end_median=float(np.median(P - E)))
        print(f"  z {z:.2f}: tuple {len(layers[z]):2d}, solusi {len(P):5d}, fraksi solusi P2 > bar "
              f"{np.mean(P > bar):.3f}, tuple dgn >=1 solusi tak aman {tup_bad}/{len(layers[z])}, "
              f"P2 med {np.median(P):.2f} maks {P.max():.2f}, (P2-E2) med {np.median(P - E):.2f}")
    rows = []
    for r in s16_rows():
        k = (r['node'], r['g'], r['p'], r['s'])
        P = np.array([p[1] for p in res[k]['P']])
        inside = bool(P.min() - 1e-9 <= r['rnea2'] <= P.max() + M2)
        rows.append(dict(seed=r['seed'], sample=r['sample'], task=r['task'], verdict=r['verdict'],
                         rnea2=r['rnea2'], P2_min=float(P.min()), P2_max=float(P.max()), covered=inside))
        print(f"  s{r['seed']:2d} sm{r['sample']} t{r['task']} {r['verdict']:14s} RNEA {r['rnea2']:5.2f}  "
              f"P2 [{P.min():5.2f}, {P.max():5.2f}]  {'tercakup' if inside else 'TIDAK'}")
    json.dump(dict(t3a=dict(diff_p=diff_p.tolist(), diff_e=diff_e.tolist(), m_p=mp.tolist(), feasible=feasible),
                   bar=bar, layers={str(z): [list(k) for k in v] for z, v in layers.items()},
                   summary={str(z): v for z, v in summ.items()}, rows=rows,
                   per_tuple={json.dumps(list(k)): v for k, v in res.items()}),
              open(os.path.join(HERE, 'g27_t3.json'), 'w'))


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'k0':
        sys.exit(0 if k0() else 1)
    dict(tmodel=tmodel, t0=t0, t1=t1, t2=t2, t3=t3)[cmd]()
