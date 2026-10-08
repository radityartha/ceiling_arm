#!/usr/bin/env python3
"""G35 A3: merge every regress_*.json (one per worker) into regress_summary.json."""
import glob
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
rows, per = [], {}
for f in sorted(glob.glob(os.path.join(HERE, 'regress_*.json'))):
    if f.endswith(('regress_summary.json', 'regress_lipschitz.json')):
        continue
    d = json.load(open(f))
    rows += d['rows']
    per[os.path.basename(f)] = d['summary']
tags = [r['tag'] for r in rows]
dup = len(tags) - len(set(tags))
gap = max(abs(r['old'][1] - r['new'][1]) for r in rows)
s = dict(n=len(rows), duplicates=dup, bad=sum(not r['ok'] for r in rows),
         bit_identical=sum(r['bit_identical'] for r in rows), max_abs_d_diff_m=gap,
         verdicts={v: sum(r['new'][0] == v for r in rows) for v in ('CLEAR', 'MARGIN', 'COLLIDE')},
         n_samples=sum(r['n'] for r in rows), t_old=sum(r['t_old'] for r in rows),
         t_new=sum(r['t_new'] for r in rows))
s['speedup'] = s['t_old'] / s['t_new']
by = {}
for r in rows:
    k = r['tag'].split('#')[0].split(' ')[0]
    b = by.setdefault(k, dict(n=0, bad=0, t_old=0.0, t_new=0.0))
    b['n'] += 1; b['bad'] += not r['ok']; b['t_old'] += r['t_old']; b['t_new'] += r['t_new']
print(json.dumps(s, indent=1))
for k, b in sorted(by.items()):
    print(f"{k:42s} n {b['n']:4d} bad {b['bad']}  {b['t_old']:8.1f} -> {b['t_new']:6.1f} s  ({b['t_old'] / b['t_new']:.1f}x)")
json.dump(dict(summary=s, by_source=by, workers=per), open(os.path.join(HERE, 'regress_summary.json'), 'w'), indent=1)
