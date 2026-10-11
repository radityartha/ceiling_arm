"""G55 A1.3: score every candidate schedule with g54_sim (parallel runner model: P1'' costs, exclusive traverses +
fairness, straight-line reservations) and select per seed x class (R90, R35) the smallest; a tie keeps the OLD schedule.

    python3 g55_eval.py     -> g55_eval.json + g55_selected.json (one row per seed, variants 'R90sel' / 'R35sel') + table
"""
import json
import os
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g54'))
import g54_sim as SIM  # noqa: E402

OLD90 = [('p1_g43/g43_candidates.json', 'R180'), ('p1_g49/g49_candidates.json', 'R180'),
         ('p1_g52/g52_candidates.json', 'R90'), ('p1_g42/g42f_candidates.json', 'G42')]
OLD35 = [('p1_g42/g42f_candidates.json', 'G42')]
NEW = {'R90': ('p1_g55/g55_R90_candidates.json', 'R90'), 'R35': ('p1_g55/g55_R35_candidates.json', 'R35')}
SEEDS = {'R90': [0, 3, 7, 8, 10, 11, 12, 18], 'R35': [0, 3, 7, 8, 10, 18]}


def rows(path, var):
    p = os.path.join(R, path)
    if not os.path.exists(p):
        return {}
    return {r['seed']: r for r in json.load(open(p)) if r.get(var, {}).get('ok_i') and r[var]['ok_ii'] and r[var]['ok_iii']}


def rot_ok(v, lim):
    return all(abs(s['rot_deg']) <= lim + 1e-6 for g in ('1', '2') for s in v['schedule'][g])


if __name__ == '__main__':
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        cross = CrossGantryChecker()
    ev, sel = [], {}
    for cls, lim in (('R90', 90.0), ('R35', 35.0)):
        for s in SEEDS[cls]:
            # G55 B: a class is a ROTATION LIMIT -- every schedule inside it is a candidate (an R35 schedule is a valid R90
            # one; s10 sim: R35-P1'' 278.5 beats R90-P1'' 315.0 because the solver does not see exclusive traverses)
            cands = []
            for path, var in OLD90 + OLD35:
                r = rows(path, var).get(s)
                if r is not None and rot_ok(r[var], lim):
                    cands.append(('lama', path, var))
            for c2 in ('R90', 'R35'):
                path, var = NEW[c2]
                r = rows(path, var).get(s)
                if r is not None and rot_ok(r[var], lim):
                    cands.append(('baru', path, var))
            seen, best = set(), None
            for kind, path, var in cands:
                r = rows(path, var)[s]
                key = json.dumps([(t['rail_m'], t['rot_deg'], t['tasks']) for g in ('1', '2') for t in r[var]['schedule'][g]])
                if key in seen:
                    continue                              # same schedule from another file
                seen.add(key)
                out = SIM.sim(s, var, path, cross, log=lambda x: None)
                mk = None if out['deadlock'] else out['makespan']
                ev.append(dict(seed=s, cls=cls, kind=kind, plan=path, variant=var, sim=mk, blocks=len(out['blocks']),
                               par_solver=r[var]['makespan']))
                print(f"s{s} {cls} {kind} {path}:{var}: sim {mk if mk is None else round(mk, 2)}, blok {len(out['blocks'])}, "
                      f"solver {r[var]['makespan']:.2f}", flush=True)
                if mk is not None and (best is None or mk < best[0] - 1e-6):
                    best = (mk, kind, path, var)
            if best is not None:
                r = dict(rows(best[2], best[3])[s])
                sel.setdefault(s, {k: r[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg',
                                                     'rot_cmd') if k in r})
                sel[s][f'{cls}sel'] = dict(r[best[3]], source=f'{best[2]}:{best[3]}', origin=best[1], sim=best[0])
                print(f"  -> s{s} {cls}: {best[1]} {best[2]}:{best[3]} sim {best[0]:.2f}", flush=True)
    json.dump(ev, open(os.path.join(HERE, 'g55_eval.json'), 'w'), indent=1)
    json.dump([sel[s] for s in sorted(sel)], open(os.path.join(HERE, 'g55_selected.json'), 'w'), indent=1, default=float)
