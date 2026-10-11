"""G53 (docs/p1_g53_p1p_serial.md): the 5 G51 HW runs (run_g32, SERIAL) against the G25 measured-cost model P1'-serial,
per event, in the runner's own event order. OFFLINE, reads only.

Model per event (g25_constants.json AS-IS, ROT_CMD 0.9235 = G31 A3; = what the G43/G49 solver charges):
  task      c_task
  retract   c_ret when the gantry's arms are off REST and a traverse of that gantry follows; 0 when skipped (REST);
            a retract with no traverse after it (end of a gantry's block) is NOT in the solver ('retract-akhir', 0)
  traverse  c_rovh + max(r * T_lin, ROT_CMD * T_rot), T = max(3, pi d / (2 * 0.9 * v))   (sched.traverse_time)
K: sum of the model over a run == the solver's 'serial' (sum of gantry finish) for that schedule.
Measured: event wall time t_end - t_start; the last task ends at its t_success (run_g32 makespan); 'jeda' = time between
events; traverse split into screen (t_start -> t_send) and motion (t_send -> t_end).

    python3 g53_p1p.py   -> g53_p1p.json + table
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
C = json.load(open(os.path.join(R, 'p1_g25/g25_constants.json')))
ROT_CMD = 0.9235
V_LIN, V_ROT = 3000.0 / 95.4930, 10.0                     # mm/s, deg/s (sched.py)
RUNS = [(7, 'R180', 'p1_g43/g43_candidates.json'), (7, 'G42', 'p1_g42/g42f_candidates.json'),
        (10, 'R180', 'p1_g43/g43_candidates.json'), (10, 'G42', 'p1_g42/g42f_candidates.json'),
        (0, 'R180', 'p1_g43/g43_candidates.json')]


def t_cmd(d, v):
    return max(3.0, math.pi * d / (2 * 0.9 * v)) if d > 1e-6 else 0.0


def model(e, nxt_trav):
    if e['kind'] == 'task':
        return 'task', C['c_task']
    if e['kind'] == 'retract':
        if e.get('skipped'):
            return 'retract', 0.0
        return ('retract', C['c_ret']) if nxt_trav else ('retract-akhir', 0.0)
    dl = abs(e['to'][0] - e['frm'][0]) * 1000.0
    dr = abs(math.degrees(e['to'][1] - e['frm'][1]))
    return 'traverse', C['c_rovh'] + max(C['r'] * t_cmd(dl, V_LIN), ROT_CMD * t_cmd(dr, V_ROT))


def one(seed, var, plan):
    run = json.load(open(os.path.join(R, f'p1_g51/hw/run/s{seed}_{var}_run.json')))
    row = {r['seed']: r for r in json.load(open(os.path.join(R, plan)))}[seed][var]
    ev, t0 = run['events'], run['t0']
    t_last = max(e['t_success'] for e in ev if e['kind'] == 'task')
    last = max(i for i, e in enumerate(ev) if e['kind'] == 'task' and e['t_success'] == t_last)
    ev = ev[:last + 1]
    meas = {k: 0.0 for k in ('task', 'traverse-layar', 'traverse-gerak', 'retract', 'retract-akhir', 'jeda')}
    pred = {k: 0.0 for k in ('task', 'traverse', 'retract', 'retract-akhir')}
    rows, prev = [], t0
    for i, e in enumerate(ev):
        nxt = next((x for x in ev[i + 1:] if x['gantry'] == e['gantry'] and x['kind'] != 'retract'), None)
        kind, p = model(e, nxt is not None and nxt['kind'] == 'traverse')
        te = e['t_success'] if i == last else e['t_end']
        m = te - e['t_start']
        meas['jeda'] += e['t_start'] - prev
        prev = te
        if kind == 'traverse':
            scr = e['rail']['t_send'] - e['t_start']
            meas['traverse-layar'] += scr
            meas['traverse-gerak'] += m - scr
        else:
            meas[kind] += m
        pred[kind] += p
        rows.append(dict(i=i, kind=kind, gantry=e['gantry'], meas=round(m, 2), pred=round(p, 2)))
    tot_p = sum(pred.values())
    return dict(seed=seed, variant=var, makespan=run['makespan'], serial_solver=row['serial'],
                par_solver=row['makespan'], pred=tot_p, k_ok=abs(tot_p - row['serial']) < 0.05,
                ratio=run['makespan'] / row['serial'], meas_parts=meas, pred_parts=pred, events=rows,
                n_task=sum(e['kind'] == 'task' for e in ev),
                n_trav=sum(e['kind'] == 'traverse' for e in ev),
                n_ret=sum(r['kind'] == 'retract' and r['pred'] > 0 for r in rows))


if __name__ == '__main__':
    out = [one(*x) for x in RUNS]
    json.dump(out, open(os.path.join(HERE, 'g53_p1p.json'), 'w'), indent=1)
    for o in out:
        mp, pp = o['meas_parts'], o['pred_parts']
        print(f"s{o['seed']} {o['variant']:4s}: terukur {o['makespan']:.2f}  P1'-serial {o['serial_solver']:.2f} "
              f"(model per event {o['pred']:.2f}, K {o['k_ok']})  rasio {o['ratio']:.3f}  P1'-par {o['par_solver']:.2f}")
        print(f"    tugas {o['n_task']}: {mp['task']:.1f} vs {pp['task']:.1f} | traverse {o['n_trav']}: layar "
              f"{mp['traverse-layar']:.1f} + gerak {mp['traverse-gerak']:.1f} = {mp['traverse-layar'] + mp['traverse-gerak']:.1f}"
              f" vs {pp['traverse']:.1f} | retract {o['n_ret']}: {mp['retract']:.1f} vs {pp['retract']:.1f} | retract-akhir "
              f"{mp['retract-akhir']:.1f} vs 0 | jeda {mp['jeda']:.1f}")


# ------------------------------------------------------------------------------- A1 (tests locked after the first pass)
def _rot_dom(e):
    dl = abs(e['to'][0] - e['frm'][0]) * 1000.0
    dr = abs(math.degrees(e['to'][1] - e['frm'][1]))
    return ROT_CMD * t_cmd(dr, V_ROT) > C['r'] * t_cmd(dl, V_LIN)


def calib(runs):
    """Mean per-event costs of `runs`: task, screen per traverse, motion / G25-traverse, retract (incl. retract-akhir)."""
    nt = sum(o['n_task'] for o in runs)
    ntr = sum(o['n_trav'] for o in runs)
    nr = sum(o['n_ret'] + 1 for o in runs)                       # + one retract-akhir per serial run
    return dict(a_t=sum(o['meas_parts']['task'] for o in runs) / nt,
                l_bar=sum(o['meas_parts']['traverse-layar'] for o in runs) / ntr,
                g=sum(o['meas_parts']['traverse-gerak'] for o in runs) / sum(o['pred_parts']['traverse'] for o in runs),
                a_r=sum(o['meas_parts']['retract'] + o['meas_parts']['retract-akhir'] for o in runs) / nr)


def p2(o, k):
    return (k['a_t'] * o['n_task'] + k['l_bar'] * o['n_trav'] + k['g'] * o['pred_parts']['traverse']
            + k['a_r'] * (o['n_ret'] + 1))


def tests(out):
    print("\nA1.1 P1''-serial LOO")
    loo = []
    for i, o in enumerate(out):
        k = calib(out[:i] + out[i + 1:])
        p = p2(o, k)
        loo.append(dict(seed=o['seed'], variant=o['variant'], pred=p, err=(p - o['makespan']) / o['makespan'], calib=k))
        print(f"  s{o['seed']} {o['variant']:4s}: P1'' {p:.2f} vs terukur {o['makespan']:.2f} -> {100 * loo[-1]['err']:+.2f} %  "
              f"(a_t {k['a_t']:.2f}, layar {k['l_bar']:.2f}, gerak x{k['g']:.3f}, a_r {k['a_r']:.2f})")
    kall = calib(out)
    print(f"  kalibrasi 5 run: {json.dumps({x: round(v, 3) for x, v in kall.items()})}")
    print('\nA1.2 delta(R180 - R35) serial')
    by = {(o['seed'], o['variant']): o for o in out}
    dl = {}
    for s in (7, 10):
        a, b = by[(s, 'R180')], by[(s, 'G42')]
        dl[s] = dict(meas=a['makespan'] - b['makespan'], p1p=a['serial_solver'] - b['serial_solver'],
                     p2=p2(a, kall) - p2(b, kall), par=a['par_solver'] - b['par_solver'])
        print(f"  s{s}: terukur {dl[s]['meas']:+.2f}, P1'-serial {dl[s]['p1p']:+.2f}, P1''-serial {dl[s]['p2']:+.2f}, "
              f"P1'-par {dl[s]['par']:+.2f}")
    print('\nA1.3 traverse dominan-rotasi')
    rr = []
    for (s, v), o in by.items():
        run = json.load(open(os.path.join(R, f'p1_g51/hw/run/s{s}_{v}_run.json')))
        for e in run['events']:
            if e['kind'] == 'traverse' and _rot_dom(e):
                _, p = model(e, True)
                m = e['t_end'] - e['rail']['t_send']
                rr.append(m / p)
                print(f"  s{s} {v} g{e['gantry']} ({e['frm'][0]:.2f},{math.degrees(e['frm'][1]):+.0f})->({e['to'][0]:.2f},"
                      f"{math.degrees(e['to'][1]):+.0f}): gerak {m:.2f} / model {p:.2f} = {m / p:.3f}; t_rot "
                      f"{e['rail'].get('t_rot')} T_cmd_rot {e['rail'].get('T_cmd_rot')}")
    rr.sort()
    med = rr[len(rr) // 2] if len(rr) % 2 else 0.5 * (rr[len(rr) // 2 - 1] + rr[len(rr) // 2])
    print(f"  n {len(rr)}, median {med:.3f}")
    json.dump(dict(loo=loo, calib_all=kall, delta=dl, rot_ratio=rr, rot_median=med),
              open(os.path.join(HERE, 'g53_tests.json'), 'w'), indent=1)


if __name__ == '__main__':
    tests(out)
