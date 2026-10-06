"""G28-YAML step 4: BEFORE (G28-ON, old yaml) vs AFTER (yaml = URDF), same real stack, R1 0/0.
Per z: PLANNED / NO-PLAN / TORQUE-UNSAFE / other, plan wall; (iv): seeds pass + per-task verdicts + task wall.

    python3 compare.py
"""
import gzip
import json
import os
import re
import statistics as st
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
G28 = os.path.join(HERE, '..', 'p1_g28')


def load(p):
    op = gzip.open if p.endswith('.gz') else open
    return [json.loads(ln) for ln in op(p, 'rt')]


def v28(recs):
    out = {}
    for z in (1.32, 1.4):
        rs = [r for r in recs if r['z'] == z]
        w = [r['wall'] for r in rs]
        out[z] = dict(n=len(rs), v=dict(Counter(r['verdict'] for r in rs)),
                      med=st.median(w) if w else None, sum=sum(w))
    return out


def iv(p):
    rows = json.load(open(p))
    tasks = [ln for r in rows for ln in r['log'] if '] task t' in ln]
    c = Counter(ln.rsplit(': ', 1)[1].split(' (')[0] for ln in tasks)
    w = [float(m.group(1)) for ln in tasks for m in [re.search(r'\(([\d.]+) s\)', ln)] if m]
    return dict(seeds=f'{sum(r["ok"] for r in rows)}/{len(rows)}', v=dict(c), n=len(tasks),
                med=st.median(w) if w else None, sum=sum(w))


b, a = v28(load(os.path.join(G28, 'v28_plans.jsonl.gz'))), v28(load(os.path.join(HERE, 'v28_after_plans.jsonl.gz')))
print('V28 per z            SEBELUM (G28-ON)                         SESUDAH (yaml = URDF)')
for z in (1.32, 1.4):
    for k in ('n', 'v', 'med', 'sum'):
        f = (lambda x: f'{x:.2f} s') if k in ('med', 'sum') else str
        print(f'  z {z:.2f} {k:4s}  {f(b[z][k]):40s} {f(a[z][k])}')
out = dict(v28_before=b, v28_after=a)
p_after = os.path.join(HERE, 'g28y_screen.json')
if os.path.exists(p_after):
    ib, ia = iv(os.path.join(G28, 'g28_screen.json')), iv(p_after)
    print('\n(iv) 14 seed G28')
    for k in ('seeds', 'n', 'v', 'med', 'sum'):
        f = (lambda x: f'{x:.2f} s') if k in ('med', 'sum') else str
        print(f'  {k:5s}  {f(ib[k]):40s} {f(ia[k])}')
    out.update(iv_before=ib, iv_after=ia)
json.dump(out, open(os.path.join(HERE, 'compare.json'), 'w'), indent=1, default=str)
