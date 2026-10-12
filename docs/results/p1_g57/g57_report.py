"""G57 B: HW parallel runs (hw/p1hw_par_s<S>_<V>_run.json): completeness, makespan vs g54_sim and vs mock G58/G60,
pairs old - new, decomposition (task / traverse / retract time summed over both gantries, gate waits, time both gantries ran
an event at once), RESERVE-BLOCKED, peak torque. Real HW data.

    python3 g57_report.py  -> table + g57_report.json
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g54'))
from g54_report import overlap  # noqa: E402

# tag: (seed, variant, kind, sim g54_sim, mock G58/G60/G55)
RUNS = {'s7L': (7, 'R180', 'lama', 396.91, 383.64), 's7B': (7, 'R90sel', 'baru', 283.62, 276.40),
        's10B': (10, 'R90sel', 'baru', 278.50, 308.87), 's10L': (10, 'G42', 'lama', 360.31, 383.17),
        's0B': (0, 'R90sel', 'baru', 148.16, 163.4),
        's8B': (8, 'R90sel', 'baru', 264.44, 272.00), 's8L': (8, 'G42', 'lama', 409.42, 443.86),
        's18L': (18, 'R180', 'lama', 372.16, 431.06), 's18G': (18, 'G60', 'baru', 232.21, 257.50),
        's3G': (3, 'G60', 'baru', 295.80, 305.54), 's3L': (3, 'R180', 'lama', 426.08, 446.13),
        's12L': (12, 'R90', 'lama', 313.49, 332.32), 's12G': (12, 'G60', 'baru', 270.70, 289.55)}
PAIRS = [(7, 's7L', 's7B'), (10, 's10L', 's10B'), (8, 's8L', 's8B'), (18, 's18L', 's18G'), (3, 's3L', 's3G'),
         (12, 's12L', 's12G')]

out = {}
for tag, (s, v, kind, sim, mock) in RUNS.items():
    p = os.path.join(HERE, 'hw', f'p1hw_par_s{s}_{v}_run.json')
    if not os.path.exists(p):
        continue
    d = json.load(open(p))
    ev = [e for e in d['events'] if e.get('t_end')]
    dur = {k: sum(e['t_end'] - e['t_start'] for e in ev if e['kind'] == k and not e.get('skipped'))
           for k in ('task', 'traverse', 'retract')}
    wait = sum(e['t_start'] - e['t_ready'] for e in ev if 't_ready' in e)
    pt = sorted(d.get('peak_tau', {}).items(), key=lambda x: -abs(x[1]))
    mk = d.get('makespan')
    r = dict(tag=tag, seed=s, variant=v, kind=kind, complete=d.get('complete'), stop=d.get('stop'), makespan=mk, sim=sim,
             mock=mock, hw_sim=mk / sim if mk else None, hw_mock=mk / mock if mk else None,
             blocked=sum(e.get('n_blocked') or 0 for e in d['events']),
             rc_bad=sum(e.get('rc') not in (0, None) for e in d['events']), both_s=overlap(ev), wait_s=wait,
             n_trav=sum(e['kind'] == 'traverse' for e in ev), peak=pt[0] if pt else None, **{f'{k}_s': x for k, x in dur.items()})
    out[tag] = r
    print(f"{tag:5s} s{s:<2} {v:6s} {kind:4s} mk {mk if mk is None else round(mk, 2)} sim {sim:.2f} ({r['hw_sim'] or 0:.3f}) "
          f"mock {mock:.2f} ({r['hw_mock'] or 0:.3f}) | tugas {dur['task']:.1f} trav {dur['traverse']:.1f} (n {r['n_trav']}) "
          f"retr {dur['retract']:.1f} tunggu {wait:.1f} bersamaan {r['both_s']:.1f} s | blok {r['blocked']} rc!=0 {r['rc_bad']} "
          f"tau {r['peak'][0]} {r['peak'][1]:.2f}" + (f" STOP {r['stop']}" if r['stop'] else ''))
print('\npasangan (lama - baru):')
pairs = []
for s, a, b in PAIRS:
    if a in out and b in out and out[a]['makespan'] and out[b]['makespan']:
        A, B = out[a], out[b]
        dh, dm, ds = A['makespan'] - B['makespan'], A['mock'] - B['mock'], A['sim'] - B['sim']
        pairs.append(dict(seed=s, d_hw=dh, d_mock=dm, d_sim=ds, pct=100 * dh / A['makespan'], r_mock=dh / dm, r_sim=dh / ds,
                          d_trav=A['traverse_s'] - B['traverse_s'], d_task=A['task_s'] - B['task_s'],
                          d_both=A['both_s'] - B['both_s']))
        p = pairs[-1]
        print(f"  s{s}: HW {dh:+.2f} s ({p['pct']:+.1f} %), mock {dm:+.2f} (HW/mock {p['r_mock']:.2f}), sim {ds:+.2f} "
              f"(HW/sim {p['r_sim']:.2f}); Δtraverse {p['d_trav']:+.1f}, Δtugas {p['d_task']:+.1f}, Δbersamaan {p['d_both']:+.1f}")
json.dump(dict(runs=out, pairs=pairs), open(os.path.join(HERE, 'g57_report.json'), 'w'), indent=1)
