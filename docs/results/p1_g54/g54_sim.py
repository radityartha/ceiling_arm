"""G54 A1.3: offline discrete-event simulation of run_g54 (same gates, same reservation screen) with P1''-serial costs
(G53 B3: task 29.04 s, screen 12.61 s / traverse, motion 0.923 x G25 traverse model, retract 45.34 s, retract at REST
2.7 s). Task sweep = straight cosine joint line from the arm's state to its goal (61 pts, = (iv'')); retract = return_rest's
straight line. Reservation of a running event = its sweep; of an idle gantry = its state. Predicts the parallel makespan
(last task end) and any DEADLOCK / block, BEFORE the mock runs. OFFLINE, no ROS.

    python3 g54_sim.py            -> g54_sim.json + table (the five G54 runs)
"""
import json
import math
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
sys.path.insert(0, os.path.join(R, 'p1_g31'))
sys.path.insert(0, os.path.join(R, 'p1_g22'))
sys.path.insert(0, os.path.join(R, 'p1_g53'))
import g31_screen as G31S  # noqa: E402
import g22_plan as P  # noqa: E402
import g53_p1p as M53  # noqa: E402
import reserve as RSV  # noqa: E402

K = json.load(open(os.path.join(R, 'p1_g53/g53_tests.json')))['calib_all']
T_SKIP = 2.7
RUNS = [(7, 'R180', 'p1_g43/g43_candidates.json'), (7, 'G42', 'p1_g42/g42f_candidates.json'),
        (10, 'R180', 'p1_g43/g43_candidates.json'), (10, 'G42', 'p1_g42/g42f_candidates.json'),
        (0, 'R90', 'p1_g52/g52_candidates.json')]


def cos_line(s0, q1, n=60):
    s0, q1 = np.asarray(s0), np.asarray(q1)
    return [list(s0 + (q1 - s0) * 0.5 * (1.0 - math.cos(math.pi * k / n))) for k in range(0, n + 1)]


def sim(seed, var, plan, cross, log=print):
    row = {r['seed']: r for r in json.load(open(os.path.join(R, plan)))}[seed]
    p0 = {g: row['p0'][str(g)] for g in (1, 2)}
    evs = G31S.events(row[var]['schedule'], p0, final_retract=True)
    E = {g: [e for e in evs if e['gantry'] == g] for g in (1, 2)}
    state = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2], 't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
    for g in (1, 2):
        for k in (1, 2):
            state.update({f't{g}_a{k}_joint_{i}': float(x) for i, x in enumerate(P.REST, 1)})
    nxt = {1: 0, 2: 0}
    run = {1: None, 2: None}                 # (kind, t_end, sweep_states, end_state_update)
    res = {g: [RSV.own(g, state)] for g in (1, 2)}
    t, t_task, blocks, trace = 0.0, {1: 0.0, 2: 0.0}, [], []
    waiting = {1: None, 2: None}             # epoch of the other gantry we wait on
    want = {1: None, 2: None}                # time a traverse started waiting (fairness)
    epoch = {1: 0, 2: 0}
    while True:
        started = False
        for g in (1, 2):
            o = 3 - g
            if run[g] is not None or nxt[g] >= len(E[g]):
                continue
            if waiting[g] is not None and waiting[g] == epoch[o]:
                continue
            waiting[g] = None
            e = E[g][nxt[g]]
            if want[o] is not None and (e['kind'] != 'traverse' or want[o] < (want[g] or t)):
                continue                     # A1 fairness: a traverse already waiting goes first
            if e['kind'] == 'traverse':
                if run[o] is not None:
                    if want[g] is None:
                        want[g] = t
                    continue
                want[g] = None
                dur = K['l_bar'] + K['g'] * M53.model(e, True)[1]
                upd = {f't{g}_linear_joint': e['to'][0], f't{g}_rotation_joint': e['to'][1]}
                run[g] = ('traverse', t + dur, None, upd)
            else:
                if run[o] is not None and run[o][0] == 'traverse':
                    continue
                if e['kind'] == 'task':
                    names = list(e['goal'])
                    pts = cos_line([state[n] for n in names], [e['goal'][n] for n in names])
                    dur = K['a_t']
                else:
                    names = [f't{g}_a{k}_joint_{i}' for k in (1, 2) for i in range(1, 7)]
                    cur = [state[n] for n in names]
                    if max(abs(c - x) for c, x in zip(cur, list(P.REST) * 2)) < math.radians(0.5):
                        run[g] = ('retract', t + T_SKIP, None, {})
                        started = True
                        trace.append((round(t, 2), g, nxt[g], 'retract-skip'))
                        nxt[g] += 1
                        continue
                    pts = cos_line(cur, list(P.REST) * 2)
                    dur = K['a_r']
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore')
                    v, d, pair, _ = RSV.screen(cross, names, pts, state, res[o], o)
                if v != 'CLEAR':
                    blocks.append(dict(t=round(t, 2), g=g, ev=nxt[g], kind=e['kind'], d_mm=round(d * 1000, 1),
                                       pair=pair, other=run[o][0] if run[o] else 'diam'))
                    waiting[g] = epoch[o]
                    if run[o] is None and waiting[o] is not None:
                        log(f'  s{seed} {var}: DEADLOCK t={t:.1f}')
                        return dict(seed=seed, variant=var, deadlock=True, t=t, blocks=blocks, trace=trace)
                    continue
                sweep = []
                for q in pts:
                    s = dict(state)
                    s.update(zip(names, q))
                    sweep.append(RSV.own(g, s))
                res[g] = [RSV.own(g, state)] + sweep
                run[g] = (e['kind'], t + dur, sweep, dict(zip(names, pts[-1])))
            trace.append((round(t, 2), g, nxt[g], e['kind']))
            nxt[g] += 1
            started = True
        if started:
            continue
        live = [g for g in (1, 2) if run[g] is not None]
        if not live:
            if all(nxt[g] >= len(E[g]) for g in (1, 2)):
                break
            log(f'  s{seed} {var}: DEADLOCK (diam) t={t:.1f}')
            return dict(seed=seed, variant=var, deadlock=True, t=t, blocks=blocks, trace=trace)
        g = min(live, key=lambda x: run[x][1])
        kind, t, _, upd = run[g]
        state.update(upd)
        if kind == 'task':
            t_task[g] = t
        run[g] = None
        res[g] = [RSV.own(g, state)]
        epoch[g] += 1
    mk = max(t_task.values())
    return dict(seed=seed, variant=var, deadlock=False, makespan=mk, finish=t_task, blocks=blocks, trace=trace,
                par_solver=row[var]['makespan'], serial_solver=row[var]['serial'])


if __name__ == '__main__':
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        cross = CrossGantryChecker()
    out = []
    for s, v, p in RUNS:
        r = sim(s, v, p, cross)
        out.append(r)
        if r['deadlock']:
            print(f's{s} {v}: DEADLOCK, blok {r["blocks"]}')
            continue
        print(f"s{s} {v:4s}: paralel P1'' {r['makespan']:.2f} (g1 {r['finish'][1]:.1f}, g2 {r['finish'][2]:.1f}); "
              f"P1'-par solver {r['par_solver']:.2f}, P1'-serial {r['serial_solver']:.2f}; terhalang {len(r['blocks'])}"
              + ''.join(f"\n    blok t={b['t']} g{b['g']} ev{b['ev']} {b['kind']} {b['d_mm']} mm {b['pair']} (lain: {b['other']})"
                        for b in r['blocks']))
    json.dump(out, open(os.path.join(HERE, 'g54_sim.json'), 'w'), indent=1, default=str)
