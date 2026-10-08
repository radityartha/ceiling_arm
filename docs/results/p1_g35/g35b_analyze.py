#!/usr/bin/env python3
"""G35b B7: per-event pre-send phases of the G35b HW R0 run vs G34b HW R0 (same seed, same tools,
only env_collision.py changed). Phases = hw_replay.phases() on the stamped event logs. Offline, no ROS.

    python3 g35b_analyze.py > g35b_analyze.log   (+ g35b_analyze.json)
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from hw_replay import phases  # noqa: E402

RUNS = {'G34b': (os.path.join(HERE, '..', 'p1_g34', 'hw'), 'g34hw_R0_'),
        'G35b': (os.path.join(HERE, 'hw'), 'g35hw_R0_')}
KEYS = {'traverse': ('start', 's28', 'env', 'motion'), 'task': ('plan', 'interarm', 'env', 'exec'),
        'retract': ('load', 'screens', 'motion')}


def load(tag):
    d, pre = RUNS[tag]
    run = json.load(open(os.path.join(d, pre + 'run.json')))
    out = []
    for i, e in enumerate(run['events']):
        path = os.path.join(d, f'{pre}ev{i:02d}_{e["kind"]}.log')
        ph = phases(path, e['kind'])
        txt = open(path).read()
        ph.update(i=i, kind=e['kind'], rc=e['rc'], dur=e['t_end'] - e['t_start'],
                  t_success=(e['t_success'] - run['t0']) if e.get('t_success') else None,
                  tau=e.get('tau_peak'), verdict=e.get('verdict_probe'))
        if e['kind'] == 'retract' and not ph.get('skipped'):
            ph['segs'] = re.findall(r"segmen \[(.*?)\]", txt)[-1:]
        if e['kind'] == 'task':
            m = re.findall(r'torsi puncak[^0-9]*([0-9.]+)', txt)
            ph['tau_exec'] = (e.get('probe') or {}).get('tau_peak') or (float(m[-1]) if m else None)
        out.append(ph)
    return run, out


def main():
    res = {}
    for tag in RUNS:
        run, evs = load(tag)
        res[tag] = dict(makespan=run.get('makespan'), complete=run.get('complete'), events=evs,
                        peak_tau=max((v, k) for k, v in run.get('peak_tau', {}).items()))
    print(f'makespan G34b {res["G34b"]["makespan"]:.2f}  G35b {res["G35b"]["makespan"]:.2f}  '
          f'delta {res["G35b"]["makespan"] - res["G34b"]["makespan"]:+.2f} s')
    for tag in RUNS:
        print(f'{tag} peak tau {res[tag]["peak_tau"]}')
    tot = {t: {} for t in RUNS}
    print(f'{"ev":>3} {"kind":9} | {"G34b dur":>8} {"G35b dur":>8} {"d":>7} | G35b phases (s)            | env mm 34b/35b | succ 34b/35b')
    for a, b in zip(res['G34b']['events'], res['G35b']['events']):
        assert a['kind'] == b['kind']
        k = a['kind']
        for t, e in (('G34b', a), ('G35b', b)):
            tot[t].setdefault(k, {}).setdefault('dur', 0.0)
            tot[t][k]['dur'] += e['dur']
            for f in KEYS[k]:
                if e.get(f) is not None:
                    tot[t][k][f] = tot[t][k].get(f, 0.0) + e[f]
        ph = ' '.join(f'{f} {b[f]:.2f}' for f in KEYS[k] if b.get(f) is not None) or 'skip'
        env = f'{a.get("env_mm", "-")}/{b.get("env_mm", "-")}'
        sc = (f'{a["t_success"]:.1f}/{b["t_success"]:.1f}' if b['t_success'] else '')
        print(f'{a["i"]:>3} {k:9} | {a["dur"]:8.2f} {b["dur"]:8.2f} {b["dur"] - a["dur"]:+7.2f} | {ph:26} | {env:14} | {sc}'
              + (f' segs {b.get("segs")}' if b.get('segs') else ''))
    print('totals per kind (s):')
    for k in KEYS:
        line = ' '.join(f'{f} {tot["G34b"][k].get(f, 0):.1f}->{tot["G35b"][k].get(f, 0):.1f}'
                        for f in ('dur',) + KEYS[k])
        print(f'  {k:9} {line}')
    res['totals'] = tot
    json.dump(res, open(os.path.join(HERE, 'g35b_analyze.json'), 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
