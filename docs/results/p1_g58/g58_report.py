"""G58 B: the 11 interleaved mock smokes (p1_g54/smoke/g58_<tag>_run.json + p1_g58/g58_smoke_<tag>.log): completeness,
makespan vs g54_sim, RESERVE-BLOCKED, time both gantries ran at once, wall time run / home (basis of the HW session
estimate), pairs old - new, repeats vs G54/G55 mock. Fake HW, NOT data.

    python3 g58_report.py  -> table + g58_report.json
"""
import datetime as dt
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g54'))
from g54_report import overlap  # noqa: E402

RUNS = [('s3L', 3, 'lama', 426.08), ('s3B', 3, 'baru', 335.75), ('s8B', 8, 'baru', 264.44), ('s8L', 8, 'lama', 409.42),
        ('s18L', 18, 'lama', 372.16), ('s18B', 18, 'baru', 298.54), ('s12', 12, 'lama=baru', 313.49),
        ('s7L', 7, 'lama', 396.91), ('s7B', 7, 'baru', 283.62), ('s10B', 10, 'baru', 278.50), ('s10L', 10, 'lama', 360.31)]
PREV = {'s7L': 380.16, 's7B': 280.98, 's10B': 313.24, 's10L': 393.57, 's3B': 376.18, 's8B': 267.44, 's18B': 354.37}


def clock(day, hms):
    return dt.datetime.combine(day, dt.time.fromisoformat(hms)).timestamp()


out = []
for tag, s, kind, sim in RUNS:
    p = os.path.join(R, 'p1_g54/smoke', f'g58_{tag}_run.json')
    log = os.path.join(HERE, f'g58_smoke_{tag}.log')
    if not os.path.exists(p):
        print(f'{tag}: belum ada')
        continue
    d = json.load(open(p))
    ev = d['events']
    t_start = d['t0']
    t_run_end = max(e['t_end'] for e in ev if e.get('t_end'))
    day = dt.datetime.fromtimestamp(t_start).date()
    m = re.findall(r'== SELESAI (\d\d:\d\d:\d\d)', open(log).read())
    t_home_end = clock(day, m[-1]) if m else None
    ok = [x for x in d['status_all'] if x['event'] == 'success']
    r = dict(tag=tag, seed=s, kind=kind, complete=d['complete'], stop=d['stop'], makespan=d['makespan'], sim=sim,
             ratio=d['makespan'] / sim if d['makespan'] else None, n_success=len(ok),
             blocked=sum(e.get('n_blocked', 0) or 0 for e in ev), rc_bad=sum(e.get('rc') not in (0, None) for e in ev),
             both_s=overlap([e for e in ev if e.get('t_end')]), run_wall=t_run_end - t_start,
             home_wall=(t_home_end - t_run_end) if t_home_end else None, prev=PREV.get(tag))
    out.append(r)
    print(f"{tag:5s} s{s:<2} {kind:9s} makespan {r['makespan']:7.2f} sim {sim:7.2f} rasio {r['ratio']:.3f} sukses "
          f"{r['n_success']}/6 blok {r['blocked']} rc!=0 {r['rc_bad']} bersamaan {r['both_s']:6.1f} s  run {r['run_wall']:6.1f} s"
          f"  pulang {r['home_wall'] if r['home_wall'] is None else round(r['home_wall'], 1)} s"
          + (f"  sebelumnya {r['prev']} ({100 * (r['makespan'] / r['prev'] - 1):+.1f} %)" if r['prev'] else ''))
by = {r['tag']: r for r in out}
print('\npasangan (lama - baru):')
for s in (3, 8, 18, 7, 10):
    a, b = by.get(f's{s}L'), by.get(f's{s}B')
    if a and b:
        dm, ds = a['makespan'] - b['makespan'], a['sim'] - b['sim']
        print(f'  s{s}: mock {dm:+7.2f} s ({100 * dm / a["makespan"]:+.1f} %), sim {ds:+7.2f}, rasio {dm / ds:.2f}')
json.dump(out, open(os.path.join(HERE, 'g58_report.json'), 'w'), indent=1)
