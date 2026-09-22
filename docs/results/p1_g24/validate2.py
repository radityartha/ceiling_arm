"""G24 A2: ORACLE'' vs the real planner (V: 49 G22 plans, W: 3 G23 (iv) plans),
F (G23 branch_probe fallen tuples), C' (81 executed calibration plans), NC1/2/5/6/7, PC1/2/3.

    python3 validate2.py [--grid 5.0]   -> g24_validate[_grid].json + printed tables
Computed ONCE per grid (A2-fix allows exactly one re-run at 2.5 deg).
"""
import argparse
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G23 = os.path.join(HERE, '../p1_g23')
sys.path.insert(0, G23)
import oracle as O  # noqa: E402
import validate as V23  # noqa: E402  (vset, l1 -- G23 code as is)
import oracle2 as O2  # noqa: E402

MARGIN = np.array(json.load(open(os.path.join(G23, 'g23_calib2.json')))['margin'])
W = [dict(name='s0 t2', arm='arm_1', xyz=[1.0, 0.3882, 1.32], rail=0.55, verdict='PLANNED'),
     dict(name='s0 t4', arm='arm_1', xyz=[0.5714, 0.5294, 1.32], rail=0.45, verdict='TORQUE-UNSAFE'),
     dict(name='s35 t0', arm='arm_2', xyz=[0.1429, 0.6, 1.4], rail=0.40, verdict='TORQUE-UNSAFE')]
GRID = O2.GRID_DEG
O3 = False          # ORACLE''' (A'): tilt + envelope saturation, PC2'


def _job(a):
    xyz, rail, arm, keep = a
    r = O2.solve(xyz, rail, arm, grid_deg=GRID, keep_solutions=keep, tilt=O3, envelope=O3)
    r['taumax'] = r['taumax'].tolist()
    if keep:
        r['solutions'] = [(k, s.tolist()) for k, s in r['solutions']]
    return r


def _g23_64(a):
    """G23 branch_probe.job verbatim (64 seeds, 5-D IK)."""
    xyz, L, arm = a
    M = O._model()
    if len(M['seeds']) == 8:
        rng = np.random.default_rng(2323)
        lo, hi = M['arm_1']['lo'], M['arm_1']['hi']
        M['seeds'] = M['seeds'] + [rng.uniform(lo, hi) for _ in range(56)]
        O.N_SEEDS = 64
    return O.solve(xyz, L, arm)[0]


def proj_dist(c, solutions):
    A = O2.model(c['arm'])
    pin, m, dd = A['pin'], A['m'], A['d']
    q = np.zeros(m.nq)
    q[A['rail']] = c['rail']
    q[A['iq']] = c['q_final']
    pin.framesForwardKinematics(m, dd, q)
    M = O2.D0.T @ dd.oMf[A['tool']].rotation
    K = int(round(360.0 / GRID))
    k = int(round((np.degrees(np.arctan2(M[1, 0], M[0, 0])) % 360) / GRID)) % K
    Rt = O2.D0 @ O2._rz(np.radians(GRID * k))
    for _ in range(O2.CONT_ITERS):
        pin.framesForwardKinematics(m, dd, q)
        T = dd.oMf[A['tool']]
        ep = np.array(c['xyz']) - T.translation
        er = pin.log3(Rt @ T.rotation.T)
        if np.linalg.norm(ep) < O2.POS_TOL and np.linalg.norm(er) < O2.ROT_TOL:
            return min((float(np.max(np.abs(np.array(s) - q[A['iq']]))) for kk, s in solutions if kk == k),
                       default=float('inf'))
        J = pin.computeFrameJacobian(m, dd, q, A['tool'], pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, A['iv']]
        q[A['iq']] = np.clip(q[A['iq']] + J.T @ np.linalg.solve(J @ J.T + O2.LAM * np.eye(6), np.r_[ep, er]),
                             A['lo'], A['hi'])
    return float('inf')


def fset():
    o = dict(np.load(os.path.join(G23, 'g23_oracle.npz')))
    ok = O.torq_all(o['taus'], MARGIN)
    idx = np.argwhere(ok)
    rng = np.random.default_rng(0)
    pick = idx[rng.choice(len(idx), min(300, len(idx)), replace=False)]
    jobs = [(tuple(o['xyz'][i]), float(o['lins'][l]), str(o['arms'][a])) for i, l, a in pick]
    with Pool(15) as p:
        res = p.map(_g23_64, jobs)
    fell = [dict(xyz=list(map(float, j[0])), rail=j[1], arm=j[2],
                 j2max64=float(np.nanmax(t[:, 1])))
            for j, t in zip(jobs, res) if not O.torq_all(t, MARGIN)]
    print(f'F: {len(jobs)} dipilih, jatuh dengan 64 benih {len(fell)} (G23: 90)')
    return fell


def main():
    global GRID, O3
    ap = argparse.ArgumentParser()
    ap.add_argument('--grid', type=float, default=O2.GRID_DEG)
    ap.add_argument('--o3', action='store_true')
    a = ap.parse_args()
    GRID, O3 = a.grid, a.o3
    Vs = V23.vset()
    old = {(v['seed'], v['task'], v['arm']): v for v in
           json.load(open(os.path.join(G23, 'g23_validate.json')))['rows']}
    C = [r for r in json.load(open(os.path.join(G23, 'g23_calib2.json')))['rows'] if 'delta_final' in r]
    F = fset()
    jobs = ([(v['xyz'], v['rail'], v['arm'], False) for v in Vs] * 2 +
            [([v['xyz'][0], v['xyz'][1], 0.40], v['rail'], v['arm'], False) for v in Vs] +
            [(w['xyz'], w['rail'], w['arm'], False) for w in W] +
            [(f['xyz'], f['rail'], f['arm'], False) for f in F] +
            [(c['xyz'], c['rail'], c['arm'], True) for c in C])
    with Pool(15) as p:
        R = p.map(_job, jobs, chunksize=1)
    n = len(Vs)
    rv, rv2, rz = R[:n], R[n:2 * n], R[2 * n:3 * n]
    rw = R[3 * n:3 * n + 3]
    rf = R[3 * n + 3:3 * n + 3 + len(F)]
    rc = R[3 * n + 3 + len(F):]
    lim5 = np.array([10, 5, 10, 7, 7, 7.0])

    print(f'grid {GRID} deg, margin {np.round(MARGIN, 4).tolist()}')
    for v, r, r2, z in zip(Vs, rv, rv2, rz):
        l1 = V23.l1(v)
        v.update(L1=l1, n_sol=r['n_sol'], n_roll=r['n_roll'], rounds=r['rounds'],
                 saturated=r['saturated'], taumax=r['taumax'], tilt_fail=r.get('tilt_fail'),
                 ik=O2.ik_v(r), torq=O2.torq2(r, MARGIN),
                 nc1=O2.ik_v(z), nc2=O2.torq2(r, MARGIN, lim=lim5),
                 nc5=bool(all(r[k] == r2[k] for k in ('n_sol', 'n_roll', 'rounds', 'saturated')) and
                          np.array_equal(r['taumax'], r2['taumax'], equal_nan=True)))
        v['oracle'] = bool(l1 and v['ik'] and v['torq'])
        v['why'] = ('-' if v['oracle'] else 'L1' if not l1 else 'IK_V' if not v['ik'] else
                    'TAK-JENUH' if not r['saturated'] else 'TORQ')
        o = old[(v['seed'], v['task'], v['arm'])]
        v['oracle_g23'] = o['oracle']
        v['j2max_g23'] = max(o['j2_branches']) if o['j2_branches'] else None
        print(f"  s{v['seed']:2d} t{v['task']} {v['arm']} {v['xyz']} @{v['rail']:.2f} {v['verdict']:13s} "
              f"ORACLE'' {'TERIMA' if v['oracle'] else 'tolak:' + v['why']:15s} (G23 {'T' if o['oracle'] else '-'}) "
              f"sol {r['n_sol']:4d} roll {r['n_roll']:2d} rd {r['rounds']} j2max "
              f"{r['taumax'][1] if r['n_sol'] else float('nan'):.2f} (G23 {v['j2max_g23']})")
    print("\nmatriks ORACLE'' x perencana (V):")
    for acc in (True, False):
        print(f"  {'terima' if acc else 'tolak '}: " + '  '.join(
            f"{k} {sum(v['oracle'] == acc and v['verdict'] == k for v in Vs)}"
            for k in ('PLANNED', 'TORQUE-UNSAFE', 'NO-PLAN')))
    for k in ('PLANNED', 'TORQUE-UNSAFE', 'NO-PLAN'):
        print(f"  sebab tolak pada {k}: " + ', '.join(
            f"{w} {sum(v['why'] == w and v['verdict'] == k for v in Vs)}"
            for w in ('L1', 'IK_V', 'TAK-JENUH', 'TORQ')))
    acc = [v for v in Vs if v['oracle']]
    npl = sum(v['verdict'] == 'PLANNED' for v in Vs)
    tp = sum(v['verdict'] == 'PLANNED' for v in acc)
    print(f"  presisi {tp}/{len(acc)}, recall {tp}/{npl}; IK_V'' pada PLANNED "
          f"{sum(v['ik'] for v in Vs if v['verdict'] == 'PLANNED')}/{npl}; "
          f"TAK-JENUH total {sum(not v['saturated'] for v in Vs)}")
    print(f"  G23 terima -> G24: {[(v['seed'], v['task'], v['verdict'], v['oracle']) for v in Vs if v['oracle_g23']]}")

    print('\nW (rencana (iv) G23):')
    for w, r in zip(W, rw):
        w.update(n_sol=r['n_sol'], saturated=r['saturated'], taumax=r['taumax'],
                 torq=O2.torq2(r, MARGIN))
        print(f"  {w['name']} {w['arm']} {w['verdict']:13s} TORQ'' {'lolos' if w['torq'] else 'TOLAK'} "
              f"j2max {r['taumax'][1]:.2f} sol {r['n_sol']} jenuh {r['saturated']}")
    for f, r in zip(F, rf):
        f.update(n_sol=r['n_sol'], saturated=r['saturated'], taumax=r['taumax'], torq=O2.torq2(r, MARGIN))
    miss = [f for f in F if f['torq']]

    for c, r in zip(C, rc):
        sol = np.array([s for _, s in r['solutions']]) if r['solutions'] else np.zeros((0, 6))
        d = np.max(np.abs(sol - np.array(c['q_final'])), axis=1) if len(sol) else np.array([np.inf])
        if O3:                                   # PC2' (A'3): project q_final to nearest grid roll, no tilt
            d = np.array([proj_dist(c, r['solutions'])])
        c.update(g24_n_sol=r['n_sol'], g24_saturated=r['saturated'], g24_dmin=float(d.min()),
                 g24_j2max=r['taumax'][1])
    pc1 = sum(c['g24_n_sol'] > 0 for c in C)
    pc2 = sum(c['g24_dmin'] < 0.15 for c in C)
    both = [v for v in Vs if v['n_sol'] and v['j2max_g23'] is not None]
    pc3 = sum(v['taumax'][1] >= v['j2max_g23'] - 0.1 for v in both)

    print('\nKONTROL (menggerbang):')
    res = dict(
        NC1=(sum(v['nc1'] for v in Vs), 0, '=='),
        NC2=(sum(v['nc2'] for v in Vs), 0, '=='),
        NC5=(sum(v['nc5'] for v in Vs), n, '=='),
        NC6=(sum(not w['torq'] for w in W[1:]), 2, '=='),
        NC7=(len(F) - len(miss), len(F), '=='),
        PC1=(pc1, int(np.ceil(0.95 * len(C))), '>='),
        PC2=(pc2, int(np.ceil(0.95 * len(C))), '>='),
        PC3=(pc3, int(np.ceil(0.95 * len(both))), '>='))
    for k, (got, need, op) in res.items():
        ok = got == need if op == '==' else got >= need
        print(f"  {k}: {got} (harus {op} {need}) {'OK' if ok else 'GAGAL'}")
    print(f"  NC7 lolos-salah: {[(f['xyz'], f['rail'], f['arm'], round(f['j2max64'], 2), round(f['taumax'][1], 2)) for f in miss]}")
    print(f"  PC2 dmin: median {np.median([c['g24_dmin'] for c in C]):.4f}, maks {max(c['g24_dmin'] for c in C):.4f}; "
          f"gagal {[(c['sess'], c['trial'], c['arm'], round(c['g24_dmin'], 3)) for c in C if c['g24_dmin'] >= 0.15]}")
    print(f"  PC3 gagal: {[(v['seed'], v['task'], round(v['taumax'][1], 2), v['j2max_g23']) for v in both if v['taumax'][1] < v['j2max_g23'] - 0.1]}")
    out = os.path.join(HERE, 'g24_validate3.json' if O3 else f'g24_validate{"" if GRID == O2.GRID_DEG else "_" + str(GRID)}.json')
    json.dump(dict(grid=GRID, margin=MARGIN.tolist(), V=Vs, W=W, F=F, C=C,
                   controls={k: [int(g), int(nd), op] for k, (g, nd, op) in res.items()}),
              open(out, 'w'), indent=1, default=float)
    print(f'-> {out}')


if __name__ == '__main__':
    main()
