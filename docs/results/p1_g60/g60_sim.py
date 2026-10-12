"""G60 (docs/p1_g60_excl_sched.md): g54_sim's timing WITHOUT the reservation screen, on per-gantry duration lists.
Same gates as g54_sim / run_g54: a traverse starts only when the other gantry runs nothing and blocks it while it runs;
a task / retract does not start while the other gantry traverses; a traverse already waiting goes before a NEW event
of the other gantry (fairness, same expression as g54_sim). Costs = P1'' (G53 B3). With 0 reservation blocks (every
g54_sim run so far) this must equal g54_sim exactly -- control `python3 g60_sim.py k`.

    python3 g60_sim.py k     -> fastsim vs g54_sim on every schedule G54/G55 simulated
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g31'))
sys.path.insert(0, os.path.join(R, 'p1_g22'))
sys.path.insert(0, os.path.join(R, 'p1_g53'))
sys.path.insert(0, os.path.join(R, 'p1_g54'))

K = json.load(open(os.path.join(R, 'p1_g53/g53_tests.json')))['calib_all']
T_SKIP = 2.7                                   # g54_sim: retract with every arm already at REST


def fastsim(E):
    """E = {1: [(kind, dur), ...], 2: [...]} in event order. -> (makespan = last task end, finish {g}, trace)."""
    nxt, run, want = {1: 0, 2: 0}, {1: None, 2: None}, {1: None, 2: None}
    t, t_task, trace = 0.0, {1: 0.0, 2: 0.0}, []
    while True:
        started = False
        for g in (1, 2):
            o = 3 - g
            if run[g] is not None or nxt[g] >= len(E[g]):
                continue
            kind, dur = E[g][nxt[g]]
            if want[o] is not None and (kind != 'traverse' or want[o] < (want[g] or t)):
                continue
            if kind == 'traverse':
                if run[o] is not None:
                    if want[g] is None:
                        want[g] = t
                    continue
                want[g] = None
            elif run[o] is not None and run[o][0] == 'traverse':
                continue
            run[g] = (kind, t + dur)
            trace.append((t, g, nxt[g], kind, t + dur))
            nxt[g] += 1
            started = True
        if started:
            continue
        live = [g for g in (1, 2) if run[g] is not None]
        if not live:
            break
        g = min(live, key=lambda x: run[x][1])
        kind, t = run[g]
        if kind == 'task':
            t_task[g] = t
        run[g] = None
    return max(t_task.values()), t_task, trace


def durations(sched_json, p0):
    """g31_screen.events -> per-gantry (kind, dur) with g54_sim's costs (retract at REST = T_SKIP)."""
    import g31_screen as G31S
    import g53_p1p as M53
    evs = G31S.events(sched_json, p0, final_retract=True)
    E = {1: [], 2: []}
    for g in (1, 2):
        at_rest = True
        for e in (x for x in evs if x['gantry'] == g):
            if e['kind'] == 'traverse':
                E[g].append(('traverse', K['l_bar'] + K['g'] * M53.model(e, True)[1]))
            elif e['kind'] == 'task':
                E[g].append(('task', K['a_t']))
                at_rest = False
            else:
                E[g].append(('retract', T_SKIP if at_rest else K['a_r']))
                at_rest = True
    return E


def k():
    rows = []
    for x in json.load(open(os.path.join(R, 'p1_g54/g54_sim.json'))):
        plan = {7: {'R180': 'p1_g43/g43_candidates.json', 'G42': 'p1_g42/g42f_candidates.json'},
                10: {'R180': 'p1_g43/g43_candidates.json', 'G42': 'p1_g42/g42f_candidates.json'},
                0: {'R90': 'p1_g52/g52_candidates.json'}}[x['seed']][x['variant']]
        rows.append((x['seed'], x['variant'], plan, x['makespan'], len(x['blocks'])))
    for x in json.load(open(os.path.join(R, 'p1_g55/g55_eval.json'))):
        rows.append((x['seed'], x['variant'], x['plan'], x['sim'], x['blocks']))
    seen, bad, n = set(), 0, 0
    for s, v, plan, ref, nb in rows:
        if (s, v, plan) in seen:
            continue
        seen.add((s, v, plan))
        r = {q['seed']: q for q in json.load(open(os.path.join(R, plan)))}[s]
        mk, fin, _ = fastsim(durations(r[v]['schedule'], {g: r['p0'][str(g)] for g in (1, 2)}))
        same = abs(mk - ref) < 1e-6
        n += 1
        bad += not same
        print(f's{s:<2} {v:5s} {plan:30s} g54_sim {ref:8.3f} (blok {nb})  fastsim {mk:8.3f}  {"IDENTIK" if same else "BEDA"}')
    print(f'K-SIM: {n - bad}/{n} identik')


if __name__ == '__main__':
    {'k': k}[sys.argv[1]]()
