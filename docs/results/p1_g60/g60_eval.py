"""G60 B: table per seed x {R90, R35}: G60 exclusive makespan, LB*rel, gap, |rot| max, vs G55 selected (g54_sim);
full g54_sim (WITH reservation screens) on every G60 schedule (D524); the R90-vs-R35 question (A2.3).

    python3 g60_eval.py   -> table + g60_eval.json
"""
import json
import os
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g54'))
import g54_sim as SIM  # noqa: E402

PFX = sys.argv[1] if len(sys.argv) > 1 else 'g60'
SEL = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g55/g55_selected.json')))}


def rows(rs):
    p = os.path.join(HERE, f'{PFX}_{rs}_candidates.json')
    return {r['seed']: r for r in json.load(open(p))} if os.path.exists(p) else {}


if __name__ == '__main__':
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        cross = CrossGantryChecker()
    out = []
    G = {rs: rows(rs) for rs in ('R90', 'R35')}
    for s in sorted(set(G['R90']) | set(G['R35'])):
        rec = dict(seed=s)
        for rs in ('R90', 'R35'):
            r = G[rs].get(s)
            if r is None or not r[rs].get('ok_i'):
                rec[rs] = None if r is None else dict(ok=False, why=r[rs].get('why'))
                continue
            v = r[rs]
            full = SIM.sim(s, rs, f'p1_g60/{PFX}_{rs}_candidates.json', cross, log=lambda x: None)
            sel = SEL.get(s, {}).get(f'{rs}sel')
            rec[rs] = dict(ok=v['ok_ii'] and v['ok_iii'], mk=v['makespan'], mk_json=v['makespan_json'], lb=v['lb_star'],
                           lb_ver=v['lb_star_ver'], gap=v['gap'], lb_wide=v.get('lb_wide'), wide_best=v.get('wide_best'), rot=v['rot_max_deg'], drot=v['drot_max_deg'],
                           full=None if full['deadlock'] else full['makespan'], blocks=len(full['blocks']),
                           g55=sel['sim'] if sel else None, iters=v['iters'], wall=v['wall_s'],
                           stops={g: [(st['rail_m'], st['rot_deg'], len(st['tasks'])) for st in v['schedule'][g]]
                                  for g in ('1', '2')})
        out.append(rec)
        for rs in ('R90', 'R35'):
            x = rec[rs]
            if not x or not x.get('mk'):
                print(f's{s:<2} {rs}: {x}')
                continue
            print(f"s{s:<2} {rs}: G60 {x['mk']:7.2f} (json {x['mk_json']:7.2f}, g54 penuh {x['full']}, blok {x['blocks']}) "
                  f"LB*rel {x['lb']:7.2f} LB*ver {x['lb_ver']:7.2f} celah {100 * x['gap']:5.2f} %  |rot| {x['rot']:4.0f}  "
                  f"G55 {x['g55']}  ok {x['ok']}  {x['stops']}")
        a, b = rec.get('R90'), rec.get('R35')
        if a and b and a.get('mk') and b.get('mk'):
            print(f"   R90 - R35 = {a['mk'] - b['mk']:+.2f} s ({100 * (a['mk'] / b['mk'] - 1):+.2f} %), |rot| R90 {a['rot']:.0f}")
    json.dump(out, open(os.path.join(HERE, f'{PFX}_eval.json'), 'w'), indent=1, default=float)
