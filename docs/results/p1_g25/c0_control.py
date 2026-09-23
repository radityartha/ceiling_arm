"""G25 A3 control C0' (2) and (3): the edited sched.py at DEFAULT cost fields must
reproduce the archives bit-for-bit (float ==).

  (2) G21 E1, 140 instances x t_fold {0, 50.8, 126.8}: exact makespan and the
      per-gantry decomposition (moves, trav, dwell, finish) of run_g21.decomp.
  (3) G24 45 (i)-(iii) instances x FISIK/DINDING: makespan and the schedule json.

    python3 c0_control.py g21     python3 c0_control.py g24
"""
import json
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g21'))
os.environ.setdefault('OMP_NUM_THREADS', '1')


def g21_job(args):
    import run_g21 as RG
    from reachability_gng.sched import gen_real, solve_exact
    tf, key, rec = args
    gs = tuple(rec['gantries'])
    inst = gen_real(rec['n'], rec['seed'], rec['n_mr'], gs, maps=RG.MAPS, t_fold=tf)
    assert inst.default_costs
    sol = solve_exact(inst)
    dec = RG.decomp(inst, sol)
    bad = []
    if sol.makespan != rec['exact']:
        bad.append(('exact', sol.makespan, rec['exact']))
    for g, v in dec['per'].items():
        a = rec['exact_dec']['per'][g]
        for k in ('moves', 'trav', 'dwell', 'finish', 'n_stops'):
            if v[k] != a[k]:
                bad.append((g, k, v[k], a[k]))
    return tf, key, bad


def g21():
    jobs = []
    for tf in (0.0, 50.8, 126.8):
        arc = json.load(open(os.path.join(R, f'p1_g21/g21_e1_tf{tf:g}.json')))
        jobs += [(tf, k, v) for k, v in arc.items()]
    with Pool(14) as pool:
        res = pool.map(g21_job, jobs, chunksize=1)
    bad = [r for r in res if r[2]]
    for r in bad:
        print('MISMATCH', r)
    print(f"C0' (2) G21 E1: {len(res) - len(bad)}/{len(res)} bit-identical")
    return not bad


def g24():
    sys.path.insert(0, os.path.join(R, 'p1_g24'))
    import make_instance_g24 as M
    MI, sched = M.MI, M.sched
    cache = M.load_cache()
    cand = json.load(open(os.path.join(R, 'p1_g24/g24_candidates.json')))
    ok = n = 0
    for row in cand:
        if not row.get('ok_i'):
            continue
        inst, _ = M.instance(row['seed'], row['nodes'], row['rejects'], cache)
        assert inst.default_costs
        for k, tf in MI.T_FOLD.items():
            n += 1
            sol = sched.solve_exact(MI.with_tfold(inst, tf))
            sj = {str(g): v for g, v in MI.sched_json(inst, sol).items()}
            same_m = sol.makespan == row['makespan'][k]
            same_s = json.loads(json.dumps(sj, default=float)) == row[f'schedule_{k}']
            if same_m and same_s:
                ok += 1
            else:
                print('MISMATCH', row['seed'], k, sol.makespan, row['makespan'][k], same_s)
    print(f"C0' (3) G24: {ok}/{n} bit-identical (makespan + schedule json), "
          f"{sum(r.get('ok_ii', False) for r in cand)} instance (i)-(iii)")
    return ok == n


if __name__ == '__main__':
    fn = {'g21': g21, 'g24': g24}[sys.argv[1]]
    sys.exit(0 if fn() else 1)
