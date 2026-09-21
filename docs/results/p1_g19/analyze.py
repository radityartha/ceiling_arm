"""G19 A3 setup-cost components per trial, from the recorder + move_group log + scorer.
Reads /tmp/g19_js.csv, the launch log, g19_windows.txt, g19_scored.json. Writes g19_components.json.
Definitions (A3, with the B-side correction for t_traverse -- see p1_g19_hw.md B1):
  retract    T0 (runner calls return_rest) -> both arms max|q-REST| < 0.5 deg
  traverse   rail encoder moves > 0.5 mm -> rail last changes (> 0.05 mm step)
  plan+screen per arm: first planning request (arm_1) / previous exec end (arm_2) -> exec start
  exec       move_group Starting -> Completed trajectory execution
  to_conc    arm_2 exec complete -> scorer CONCURRENT
  T_setup    T0 -> arm_2 exec complete (wall clock)
  drift      max-min of every arm joint while the rail moves, vs the same measure in the
             20 s AFTER the rail stops (baseline ripple with the rail still)"""
import collections, csv, json, math, re, sys
LAUNCH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/g18_t3.log'
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
W = [(int(a), float(r), float(b), float(c), float(d), float(e))
     for a, r, b, c, d, e in (l.split() for l in open('g19_windows.txt'))]
S = json.load(open('g19_scored.json'))
rail, arm, eff = [], collections.defaultdict(list), collections.defaultdict(list)
t_lo = W[0][2] - 10
for r in csv.DictReader(open('/tmp/g19_js.csv')):
    t = float(r['t'])
    if t < t_lo: continue
    if r['joint'] == 't1_linear_joint': rail.append((t, float(r['pos'])))
    elif r['joint'].startswith('t1_a'):
        arm[r['joint']].append((t, float(r['pos'])))
        if r['eff']: eff[r['joint']].append((t, abs(float(r['eff']))))
mg = []
for l in open(LAUNCH):
    if 'move_group' not in l: continue
    m = re.search(r'\[(\d+\.\d+)\]', l)
    if not m: continue
    t = float(m.group(1))
    if t < t_lo: continue
    if 'Planning request received for MoveGroup' in l: mg.append((t, 'P'))
    elif 'Starting trajectory execution' in l: mg.append((t, 'S'))
    elif 'Completed trajectory execution' in l: mg.append((t, 'C'))
def span(lo, hi):
    m, jj = 0.0, ''
    for j, v in arm.items():
        w = [p for t, p in v if lo <= t <= hi]
        if w and math.degrees(max(w) - min(w)) > m: m, jj = math.degrees(max(w) - min(w)), j
    return round(m, 4), jj
out = []
for i, goal, t0, t1, t2, t3 in W:
    row = dict(trial=i, rail=goal, verdict=S[str(i)]['verdict'])
    if i > 0:
        import numpy as np
        ts = np.arange(t0, t1 + 5, 0.02)
        dev = np.zeros_like(ts)
        for a in (1, 2):
            for j in range(1, 7):
                v = np.array(arm[f't1_a{a}_joint_{j}'])
                dev = np.maximum(dev, np.abs(np.interp(ts, v[:, 0], v[:, 1]) - REST[j - 1]))
        ok = np.nonzero(np.degrees(dev) < 0.5)[0]
        at = float(ts[ok[0]]) if len(ok) else None
        row['retract'] = round(at - t0, 2) if at else None
        R = [(t, p) for t, p in rail if t1 <= t <= t2]
        s = R[0][1]; mv = [t for t, p in R if abs(p - s) > 0.0005]
        stop = mv[0]
        for (ta, pa), (tb, pb) in zip(R, R[1:]):
            if abs(pb - pa) > 0.00005: stop = tb
        row.update(traverse=round(stop - mv[0], 2), rail_end_mm=round(R[-1][1] * 1000, 2),
                   rail_err_mm=round((R[-1][1] - goal) * 1000, 2), T_lin=round(0.29 + abs(goal - s) * 1000 / 31.416, 2),
                   gap_retract_to_rail=round(mv[0] - at, 2) if at else None, gap_rail_to_task=round(t2 - stop, 2),
                   drift_moving=span(mv[0], stop), drift_after=span(stop, stop + 20),
                   tau_arm_traverse=round(max((v for L in eff.values() for t, v in L if mv[0] <= t <= stop), default=float('nan')), 3))
    E = [(t, k) for t, k in mg if t2 <= t <= t3]
    st = [t for t, k in E if k == 'S']; co = [t for t, k in E if k == 'C']; pl = [t for t, k in E if k == 'P']
    if len(st) >= 2 and len(co) >= 2 and pl:
        row.update(plan1=round(st[0] - pl[0], 2), exec1=round(co[0] - st[0], 2),
                   plan2=round(st[1] - co[0], 2), exec2=round(co[1] - st[1], 2),
                   n_plan_requests=len(pl))
        row['extend'] = round(co[1] - pl[0], 2)
        if S[str(i)].get('conc'): row['to_conc'] = round(S[str(i)]['conc'] - co[1], 2)
        if i > 0: row['T_setup_wall'] = round(co[1] - t0, 2)
    row['tau_task_peak'] = round(max((v for L in eff.values() for t, v in L if t2 <= t <= t3), default=float('nan')), 3)
    out.append(row)
    print(json.dumps(row))
json.dump(out, open('g19_components.json', 'w'), indent=1)
