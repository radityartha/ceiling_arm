#!/usr/bin/env python3
"""G21 task 4 (side, offline): joint_2 offset = measured peak - RNEA prediction.

One method for g18, g19 and g20, from archived files only:
  prediction  the last 'torsi RNEA ... 2=<x>->' line before each 'armN: moveit
              MOVED' in the probe log = the plan that was EXECUTED (a refused
              plan and its retry both print one, g20 trial 6)
  measured    peak |effort| of that arm's joint_2 in /joint_states over the
              probe's run window from the batch windows file (probe start ->
              probe exit) + 3 s tail -- the g20 analyze.py definition. The probe
              log's own stamps are NOT usable: its last lines are unstamped, so
              its stamp span ends before the last arm executes (g21 B4).
              g20 stage 1b/2b (3 plans) have no windows file -> excluded.
Cross-check: on g20 this must reproduce g20_scored.json j2_meas.

Does NOT touch the filter code (g20 C: that is the operator's decision).
  python3 torque_offsets.py   -> g21_torque_offsets.json + table on stdout
"""
import csv
import gzip
import json
import re
from pathlib import Path

import numpy as np

R = Path(__file__).resolve().parents[1]
PFX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
# session: (trial log pattern, joint_states, windows file, start col, end col)
SESS = {'g18': ('p1_g18/g18_trial{}.log', 'p1_g18/g18_joint_states_all_trials.csv.gz',
                'p1_g18/g18_windows.txt', 2, 3),
        'g19': ('p1_g19/g19_trial{}.log', 'p1_g19/g19_joint_states_all.csv.gz',
                'p1_g19/g19_windows.txt', 4, 5),
        'g20': ('p1_g20/g20_trial{}.log', 'p1_g20/g20_joint_states_all.csv.gz',
                'p1_g20/g20_windows.txt', 1, 2)}


def plans_of(log):
    txt = Path(log).read_text()
    out, last = [], None
    for line in txt.splitlines():
        m = re.search(r'torsi RNEA.* 2=([0-9.]+)->', line)
        if m:
            last = float(m[1])
        m = re.search(r'(arm_\d): moveit MOVED', line)
        if m:
            out.append(dict(log=Path(log).name, arm=m[1], rnea=last))
    return out


rows = []
for sess, (pat, js, wf, c0, c1) in SESS.items():
    items = []
    for line in open(R / wf):
        c = line.split()
        win = (float(c[c0]), float(c[c1]) + 3.0)
        for p in plans_of(R / pat.format(c[0])):
            p.update(sess=sess, win=win, joint=PFX[p['arm']] + 'joint_2', meas=0.0)
            items.append(p)
    want = {p['joint'] for p in items}
    with gzip.open(R / js, 'rt') as f:
        for r in csv.DictReader(f):
            if r['joint'] not in want or not r['eff']:
                continue
            t, e = float(r['t']), abs(float(r['eff']))
            for p in items:
                if p['joint'] == r['joint'] and p['win'][0] <= t <= p['win'][1] \
                        and e > p['meas']:
                    p['meas'] = e
    for p in items:
        p['offset'] = round(p['meas'] - p['rnea'], 3)
    rows += items

# cross-check against g20_scored.json (same definition, windows file there)
sc = json.loads((R / 'p1_g20/g20_scored.json').read_text())
ref = {(f'g20_trial{o["trial"]}.log', a): m for o in sc
       for a, m in zip(PFX, o['j2_meas'])}
dev = [abs(p['meas'] - ref[(p['log'], p['arm'])]) for p in rows
       if (p['log'], p['arm']) in ref]
print(f'cross-check g20: {len(dev)} plans vs g20_scored j2_meas, '
      f'max |diff| {max(dev):.3f} N·m')

print(f'\n{"":>10} {"n":>3} {"min":>6} {"median":>7} {"mean":>6} {"sd":>5} '
      f'{"p95":>6} {"max":>6}   pred range')
groups = {a: [p for p in rows if p['arm'] == a] for a in PFX}
groups['arm_1+2'] = groups['arm_1'] + groups['arm_2']
groups['arm_3+4'] = groups['arm_3'] + groups['arm_4']
groups['ALL'] = rows
summary = {}
for k, ps in groups.items():
    o = np.array([p['offset'] for p in ps])
    pr = np.array([p['rnea'] for p in ps])
    summary[k] = dict(n=len(o), min=o.min(), median=float(np.median(o)),
                      mean=o.mean(), sd=o.std(ddof=1), p95=float(np.percentile(o, 95)),
                      max=o.max(), slope=float(np.polyfit(pr, o, 1)[0]))
    s = summary[k]
    print(f'{k:>10} {s["n"]:>3} {s["min"]:+6.2f} {s["median"]:+7.2f} '
          f'{s["mean"]:+6.2f} {s["sd"]:5.2f} {s["p95"]:+6.2f} {s["max"]:+6.2f}'
          f'   {pr.min():.2f}-{pr.max():.2f}  slope {s["slope"]:+.2f}')
print('\nper session:', {s: sum(p['sess'] == s for p in rows) for s in SESS})

# what each candidate threshold would have done to the plans we HAVE
print('\nthreshold rule "RNEA + off <= cap": plans that would be REFUSED, and '
      'plans that MEASURED above cap but were ALLOWED')
for off, cap in [(6.6, 14.0), (7.2, 14.0), (6.6, 13.0), (7.7, 14.0)]:
    refused = sum(p['rnea'] + off > cap for p in rows)
    miss = [(p['log'], p['arm'], round(p['meas'], 2)) for p in rows
            if p['rnea'] + off <= cap and p['meas'] > cap]
    over = sum(p['meas'] > p['rnea'] + off for p in rows)
    print(f'  +{off} <= {cap}: refused {refused}/{len(rows)}, '
          f'measured > prediction+off {over}, allowed yet > {cap}: {miss}')

json.dump(dict(rows=rows, summary=summary), open(Path(__file__).with_name(
    'g21_torque_offsets.json'), 'w'), indent=1, default=float)
