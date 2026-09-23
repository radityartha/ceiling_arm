"""G26 A3: per-event P1' prediction in the A3 runner order (g22_plan.events), g25 constants
AS-IS (no recalibration). Also P1'-par (solver) and old P1 (FISIK, T_lin) for the same schedule.

    python3 g26_predict.py 1 5 9        -> g26_predict.json (those seeds) + table
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
sys.path.insert(0, os.path.join(HERE, '../p1_g25'))
import g22_plan as P  # noqa: E402
from calibrate import t_cmd  # noqa: E402

C = json.load(open(os.path.join(HERE, '../p1_g25/g25_constants.json')))
PLAN = os.path.join(HERE, 'g26_candidates.json')
V = 3000 / 95.4930          # mm/s, sched.V_LIN_MM_S


def predict(row):
    evs = P.events(row)
    worked = {1: False, 2: False}          # has any arm of g left REST in this run?
    out = []
    for k, e in enumerate(evs):
        g = e['gantry']
        if e['kind'] == 'task':
            p = C['task_to_success'] if k == len(evs) - 1 else C['c_task']
            worked[g] = True
            extra = dict(arm=e['arm'], task=e['task'], xyz=e['xyz'])
        elif e['kind'] == 'retract':
            p = C['c_ret'] if worked[g] else 0.0
            extra = dict(skip=not worked[g])
        else:
            p = C['c_rovh'] + C['r'] * t_cmd(e['to'] - e['frm'])
            extra = dict(frm=e['frm'], to=e['to'], T_cmd=t_cmd(e['to'] - e['frm']))
        out.append(dict(event=k, kind=e['kind'], gantry=g, pred=p, **extra))
    return out


def p1_old(row):
    """g22 A4 P1: per gantry sum T_lin + 50.80 m_g + 2.0 per slot; serial = sum_g."""
    F = {}
    for g in (1, 2):
        d = row['decomp']['FISIK'][str(g)]
        slots = sum(len(s['tasks']) for s in row['schedule_FISIK'][str(g)])
        F[g] = d['T_lin'] + 50.80 * d['moves'] + 2.0 * slots
    return F


if __name__ == '__main__':
    rows = {r['seed']: r for r in json.load(open(PLAN))}
    res = {}
    for s in map(int, sys.argv[1:]):
        row = rows[s]
        ev = predict(row)
        F = p1_old(row)
        res[s] = dict(seed=s, events=ev, makespan_serial=sum(e['pred'] for e in ev),
                      P1p_par=row['makespan']["P1'"], P1p_serial_solver=row['makespan']["P1'_serial"],
                      P1_serial=sum(F.values()), P1_par=max(F.values()), p0=P.p0_rails(row))
        print(f"seed {s}: p0 {P.p0_rails(row)}  P1'-serial {res[s]['makespan_serial']:.2f}  "
              f"(solver Σ_g {row['makespan'][chr(80)+'1'+chr(39)+'_serial']:.2f}); P1'-par {res[s]['P1p_par']:.2f}; "
              f"P1 serial {res[s]['P1_serial']:.2f}")
        for e in ev:
            d = {'task': f"{e.get('arm', '')} t{e.get('task', '')} {e.get('xyz', '')}",
                 'retract': 'DILEWATI (REST)' if e.get('skip') else '',
                 'traverse': f"{e.get('frm', 0):.3f}->{e.get('to', 0):.3f} T_cmd {e.get('T_cmd', 0):.2f}"}[e['kind']]
            print(f"   [{e['event']}] {e['kind']:8s} g{e['gantry']} {e['pred']:7.2f}  {d}")
    json.dump(res, open(os.path.join(HERE, 'g26_predict.json'), 'w'), indent=1)
