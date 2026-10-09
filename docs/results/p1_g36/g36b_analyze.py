#!/usr/bin/env python3
"""G36b B7: per-event phases of the G36b HW runs (R0, R10) vs G35b HW R0. Same seed, same runner;
G36 changed the probe (retract_check) and S18 (branch-and-bound). Phases = p1_g35 hw_replay.phases()
on the stamped event logs; tasks add `retract_chk` (the G36 straight-home screen, first attempt) and
`attempts` (RETRACT-BLOCKED and other retryable refusals + 1). Offline, no ROS.

    python3 g36b_analyze.py > g36b_analyze.log   (+ g36b_analyze.json)
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g35'))
from hw_replay import phases, stamps  # noqa: E402

RUNS = {'G35b-R0': (os.path.join(HERE, '..', 'p1_g35', 'hw'), 'g35hw_R0_'),
        'G36b-R0': (os.path.join(HERE, 'hw'), 'g36hw_R0_'),
        'G36b-R10': (os.path.join(HERE, 'hw'), 'g36hw_R10_')}
KEYS = {'traverse': ('start', 's28', 'env', 'motion'), 'task': ('plan', 'interarm', 'env', 'retract_chk', 'exec'),
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
                  t_success=(e['t_success'] - run['t0']) if e.get('t_success') else None)
        if e['kind'] == 'retract' and not ph.get('skipped'):
            ph['segs'] = re.findall(r"segmen \[(.*?)\]", txt)[-1:]
        if e['kind'] == 'task':
            ph['blocked'] = txt.count('JALUR PULANG TERHALANG')
            ph['attempts'] = 1 + len(re.findall(r'percobaan tugas: (?!PLANNED)', txt))
            st = stamps(path)
            t_rg = next((t for t, s in st if 'retract g' in s and t >= ph['t_env_end']), None)
            t_re = next((t for t, s in st if 'penyaring lingkungan' in s and t_rg and t >= t_rg), None)
            if t_rg and t_re:                       # G36: screen sits between env and MoveIt execute
                ph['retract_chk'] = t_re - ph['t_env_end']
                if ph.get('exec') is not None:
                    ph['exec'] -= ph['retract_chk']
        out.append(ph)
    return run, out


def main():
    res = {}
    for tag in RUNS:
        run, evs = load(tag)
        res[tag] = dict(makespan=run.get('makespan'), complete=run.get('complete'), events=evs,
                        peak_tau=max((v, k) for k, v in run.get('peak_tau', {}).items()))
        print(f'{tag:9} makespan {res[tag]["makespan"]:.2f} s  complete {res[tag]["complete"]}  '
              f'peak tau {res[tag]["peak_tau"][0]:.2f} {res[tag]["peak_tau"][1]}')
    m = {t: res[t]['makespan'] for t in RUNS}
    print(f'G36b-R0 - G35b-R0 {m["G36b-R0"] - m["G35b-R0"]:+.2f} s;  '
          f'Delta(R10 - R0) G36b {m["G36b-R10"] - m["G36b-R0"]:+.2f} s')
    tot = {t: {} for t in RUNS}
    for tag in RUNS:
        print(f'\n== {tag}')
        for e in res[tag]['events']:
            k = e['kind']
            tot[tag].setdefault(k, {}).setdefault('dur', 0.0)
            tot[tag][k]['dur'] += e['dur']
            for f in KEYS[k]:
                if e.get(f) is not None:
                    tot[tag][k][f] = tot[tag][k].get(f, 0.0) + e[f]
            ph = ' '.join(f'{f} {e[f]:.2f}' for f in KEYS[k] if e.get(f) is not None) or 'skip'
            extra = (f' env {e["env_mm"]} mm' if e.get('env_mm') is not None else '') + \
                    (f' attempts {e["attempts"]} blocked {e["blocked"]}' if k == 'task' else '') + \
                    (f' succ@{e["t_success"]:.1f}' if e.get('t_success') else '') + \
                    (f' segs {e["segs"]}' if e.get('segs') else '')
            print(f'{e["i"]:>3} {k:9} {e["dur"]:7.2f} | {ph}{extra}')
    print('\ntotals per kind (s):  G35b-R0 / G36b-R0 / G36b-R10')
    for k in KEYS:
        line = '  '.join(f'{f} ' + '/'.join(f'{tot[t][k].get(f, 0):.1f}' for t in RUNS) for f in ('dur',) + KEYS[k])
        print(f'  {k:9} {line}')
    res['totals'] = tot
    json.dump(res, open(os.path.join(HERE, 'g36b_analyze.json'), 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
