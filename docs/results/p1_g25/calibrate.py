"""G25 step 3a -- constants EXACTLY as locked in docs/p1_g25_task_cost.md A2 (median,
calibration set G19 + G20 only), then the event-by-event prediction of G24b seed 1
and its error. Writes g25_constants.json and g25_predict_g24b.json.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
V_LIN = 3000.0 / 95.4930 / 1000.0           # m/s, sched.V_LIN_MM_S


def t_cmd(d_m):
    """g22_plan.t_cmd, apa adanya."""
    return max(3.0, math.pi * abs(d_m) / (2 * 0.9 * V_LIN))


def constants():
    d = json.load(open(os.path.join(HERE, 'g25_calib_components.json')))
    probe = {(p['src'], p['trial']): p for p in d['probes']}
    first = [e for e in d['execs'] if e['first_in_process']]
    per = [probe[(e['src'], e['trial'])]['t_start'] + e['t_armspan'] + e['t_exec'] + e['t_succ']
           for e in first]
    # probes that ENDED on success: every arm executed and the last event is the
    # common window (t_end < 1 s); a TORQUE-ABORT probe keeps waiting (A1)
    ends = [p['t_end'] for p in d['probes'] if p['t_end'] is not None and p['t_end'] < 1.0]
    mv = d['moves_g19']
    c = dict(
        n_first=len(per), task_to_success=float(np.median(per)),
        n_end=len(ends), t_end=float(np.median(ends)),
        c_ret=float(np.median([m['retract_call'] for m in mv])),
        r=float(np.median([m['traverse'] / t_cmd(0.400) for m in mv])),
        c_rovh=float(np.median([m['rail_call'] - m['traverse'] for m in mv])),
        gap=0.0)
    c['c_task'] = c['task_to_success'] + c['t_end']
    c['t_fold'] = c['c_ret'] + c['c_rovh']          # sched: every later move
    c['t_fold_first'] = c['c_rovh']                 # sched: first move off p0, arms at REST
    return c


# G24b seed 1 in A3 order (g24 E1/E5); retract never skipped in this run.
EVENTS = [('task', 1, None), ('task', 1, None), ('retract', 1, None), ('traverse', 1, 0.70 - 0.55),
          ('task', 1, None), ('task', 2, None), ('task', 2, None), ('retract', 2, None),
          ('traverse', 2, 1.45 - 0.00), ('task', 2, None)]
MEASURED_MAKESPAN = 480.44
OLD = dict(P1=165.11, P2=202.49, P3=317.11, P4=354.49)   # g24 E2, serial


def predict(c):
    out = []
    for k, (kind, g, delta) in enumerate(EVENTS):
        if kind == 'task':
            p = c['task_to_success'] if k == len(EVENTS) - 1 else c['c_task']
        elif kind == 'retract':
            p = c['c_ret']
        else:
            p = c['c_rovh'] + c['r'] * t_cmd(delta)
        out.append(dict(event=k, kind=kind, gantry=g, pred=p))
    return out


if __name__ == '__main__':
    c = constants()
    json.dump(c, open(os.path.join(HERE, 'g25_constants.json'), 'w'), indent=1)
    print('konstanta', json.dumps(c, indent=1))
    meas = json.load(open(os.path.join(HERE, 'g24b_components.json')))
    ev = {e['event']: e for e in meas['events']}
    pred = predict(c)
    for p in pred:
        e = ev[p['event']]
        m = e['to_success'] if p['event'] == len(EVENTS) - 1 else e['wall']
        p.update(meas=m, err=p['pred'] - m, err_pct=100 * (p['pred'] - m) / m)
        print(f"  ev{p['event']} {p['kind']:8s} g{p['gantry']}  pred {p['pred']:7.2f}  terukur {m:7.2f}  "
              f"galat {p['err']:+7.2f} s ({p['err_pct']:+6.1f} %)")
    gaps = sum(meas['runner_gaps']) + meas['t0_to_first_cmd']
    ms = sum(p['pred'] for p in pred) + c['gap'] * len(EVENTS)
    by = {}
    for p in pred:
        by.setdefault(p['kind'], [0.0, 0.0])
        by[p['kind']][0] += p['pred']
        by[p['kind']][1] += p['meas']
    res = dict(constants=c, events=pred, makespan_pred=ms, makespan_meas=MEASURED_MAKESPAN,
               err=ms - MEASURED_MAKESPAN, err_pct=100 * (ms - MEASURED_MAKESPAN) / MEASURED_MAKESPAN,
               by_kind={k: dict(pred=v[0], meas=v[1], err=v[0] - v[1]) for k, v in by.items()},
               runner_gaps_meas=gaps, old=OLD)
    json.dump(res, open(os.path.join(HERE, 'g25_predict_g24b.json'), 'w'), indent=1)
    print(f"P1'-serial {ms:.2f} vs terukur {MEASURED_MAKESPAN} -> galat {res['err']:+.2f} s "
          f"({res['err_pct']:+.2f} %); sela runner terukur {gaps:.2f} s (model 0)")
    for k, v in res['by_kind'].items():
        print(f"  {k:8s} pred {v['pred']:7.2f} terukur {v['meas']:7.2f} galat {v['err']:+7.2f}")
    for k, v in OLD.items():
        print(f"  {k}-serial {v:7.2f} -> galat {v - MEASURED_MAKESPAN:+7.2f} s "
              f"({100 * (v - MEASURED_MAKESPAN) / MEASURED_MAKESPAN:+.1f} %)")
