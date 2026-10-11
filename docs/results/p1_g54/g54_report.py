"""G54 B: per smoke run (smoke/<tag>_run.json): completeness, makespan vs g54_sim, RESERVE-BLOCKED, gate waits,
time both gantries ran an event at once, probe verdicts, minimum reservation distance (runner retract screens + probes).

    python3 g54_report.py s7r180 s7r35 s10r180 s10r35 s0r90 neg_s7r180
"""
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = {(r['seed'], r['variant']): r for r in json.load(open(os.path.join(HERE, 'g54_sim.json')))}


def overlap(evs):
    iv = {1: [], 2: []}
    for e in evs:
        iv[e['gantry']].append((e['t_start'], e['t_end']))
    tot = 0.0
    for a0, a1 in iv[1]:
        for b0, b1 in iv[2]:
            tot += max(0.0, min(a1, b1) - max(a0, b0))
    return tot


def dmin(tag):
    d = []
    for f in glob.glob(os.path.join(HERE, 'smoke', f'{tag}_ev*_task.log')) + \
            glob.glob(os.path.join(HERE, 'smoke', f'{tag}_runner.log')):
        for ln in open(f, errors='ignore'):
            m = re.search(r'(?:G54 reservasi g\d: \d+ keadaan, minimum|vs reservasi g\d:) ([0-9.]+) mm', ln)
            if m:
                d.append(float(m.group(1)))
    return (min(d), len(d)) if d else (None, 0)


for tag in sys.argv[1:]:
    p = os.path.join(HERE, 'smoke', f'{tag}_run.json')
    if not os.path.exists(p):
        print(f'{tag}: tidak ada run.json')
        continue
    r = json.load(open(p))
    ev = r['events']
    tasks = [e for e in ev if e['kind'] == 'task']
    sim = SIM.get((r['seed'], r['variant']))
    blk = sum(e.get('n_blocked', 0) for e in ev)
    wait = sum(e['t_start'] - e['t_ready'] for e in ev if 't_ready' in e)
    pv = [(e.get('probe') or {}).get('plan_verdicts', {}).get(e['arm']) for e in tasks]
    dm, nd = dmin(tag)
    mk = r.get('makespan')
    print(f"{tag}: s{r['seed']} {r['variant']} stop={r.get('stop')!r} lengkap={r.get('complete')} "
          f"tugas {sum(1 for e in tasks if e.get('t_success'))}/{len(tasks)} makespan {mk if mk is None else round(mk, 2)}"
          + (f" sim {sim['makespan']:.2f} rasio {mk / sim['makespan']:.3f}" if sim and mk else '')
          + f"\n    RESERVE-BLOCKED {blk}, tunggu gerbang {wait:.1f} s, dua gantry bersamaan {overlap(ev):.1f} s, "
          f"verdict probe {[e.get('verdict_probe') for e in tasks]}, rencana {pv}, jarak reservasi min {dm} mm (n {nd})"
          + (f"\n    inject {r.get('inject')}, gerak sesudah stop {r.get('post_stop_motion')}, berjalan saat stop "
             f"{r.get('running_at_stop')}" if r.get('stop') else ''))
