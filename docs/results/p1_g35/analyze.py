#!/usr/bin/env python3
"""G35 A1/A4: HW G34b pre-send phases vs offline replay (old, new) per event; totals per variant.

    python3 analyze.py > analyze.log
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
old = json.load(open(os.path.join(HERE, 'hw_replay_old.json')))
new = {(r['variant'], r['ev']): r for r in json.load(open(os.path.join(HERE, 'hw_replay_new.json')))}
WAIT = 1.0      # task: screen_env waits wait_joints(1.0 s) + /get_planning_scene inside the HW env phase
out = dict(rows=[], totals={})
print(f"{'var':4s}{'ev':>3s} {'jenis':9s}{'n':>5s} | {'HW env':>7s} {'rep lama':>8s} {'beda':>6s} | {'rep baru':>8s} "
      f"{'hemat':>6s} | HW lain (S28/antar-lengan/muat retract)")
for r in old:
    if r.get('skipped') or 'rep_build' not in r:
        continue
    n = new.get((r['variant'], r['ev']), {})
    rep_old = r['rep_build'] + r['rep_screen'] + (WAIT if r['kind'] == 'task' else 0.0)
    rep_new = (n.get('rep_build', float('nan')) + n.get('rep_screen', float('nan'))
               + (WAIT if r['kind'] == 'task' else 0.0))
    if r['kind'] == 'retract':
        hw_env = None                     # three screens share one stamp; env part not separable on HW
        other = f"muat {r['load']:.1f} + 3 saring {r['screens']:.1f}"
    else:
        hw_env = r['env']
        other = f"S28 {r['s28']:.1f}" if r['kind'] == 'traverse' else f"antar-lengan {r['interarm']:.1f}"
    saved = rep_old - rep_new
    row = dict(variant=r['variant'], ev=r['ev'], kind=r['kind'], n=r['rep_n'], hw_env=hw_env, rep_old=rep_old,
               rep_new=rep_new, saved=saved, pre_send=r['pre_send'],
               rel_err=(rep_old - hw_env) / hw_env if hw_env else None)
    out['rows'].append(row)
    print(f"{r['variant']:4s}{r['ev']:3d} {r['kind']:9s}{r['rep_n']:5d} | "
          f"{hw_env if hw_env is None else round(hw_env, 2)!s:>7s} {rep_old:8.2f} "
          f"{'' if hw_env is None else '%+.0f%%' % (100 * row['rel_err']):>6s} | "
          f"{rep_new:8.2f} {saved:6.2f} | {other}")
for v in ('R0', 'R10'):
    rs = [x for x in out['rows'] if x['variant'] == v]
    tot = {k: sum(x[k] for x in rs) for k in ('rep_old', 'rep_new', 'saved')}
    tot['hw_env_known'] = sum(x['hw_env'] for x in rs if x['hw_env'] is not None)
    tot['rep_old_known'] = sum(x['rep_old'] for x in rs if x['hw_env'] is not None)
    tot['saved_by_kind'] = {k: sum(x['saved'] for x in rs if x['kind'] == k) for k in ('traverse', 'task', 'retract')}
    out['totals'][v] = tot
    print(v, json.dumps({k: (round(x, 2) if isinstance(x, float) else
                             {a: round(b, 2) for a, b in x.items()}) for k, x in tot.items()}))
json.dump(out, open(os.path.join(HERE, 'analyze.json'), 'w'), indent=1)
