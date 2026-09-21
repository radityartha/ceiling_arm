"""g20 step 5 scoring, from archived files only (A1 verdict = MONITOR events, not probe labels).
   python3 analyze.py  -> g20_scored.json + table on stdout"""
import csv, gzip, json, re
W = {int(l.split()[0]): (float(l.split()[1]), float(l.split()[2])) for l in open('g20_windows.txt')}
conc = [float(m) for m in re.findall(r'\[(\d+\.\d+)\] \[reach_dwell_monitor\]: >>> 4-ARM CONCURRENT', open('g20_monitor.log').read())]
rows = list(csv.DictReader(open('g20_step5_summary.csv')))[3:]          # 3 = stage 1b, 2b
assert len(rows) == 40, len(rows)
ends = sorted(W)
js = {i: {} for i in W}
pred = {}
for i in W:
    log = open(f'g20_trial{i}.log').read()
    # prediction of the plan that was EXECUTED: the last one before each 'moveit MOVED'
    # (a refused plan and its retry both print a prediction -- trial 6).
    p2, last = [], None
    for line in log.splitlines():
        m = re.search(r'torsi RNEA.* 2=([0-9.]+)->', line)
        if m: last = float(m[1])
        if 'moveit MOVED' in line: p2.append(last)
    assert len(p2) == 4, (i, p2)
    d = re.findall(r'rencana: ([\-0-9.]+) mm di titik \d+ \((\S+) <-> (\S+)\)', log)
    pred[i] = dict(rnea_j2=p2, interarm=[(float(a), b, c) for a, b, c in d])
nxt = {i: (W[i + 1][0] if i + 1 in W else 1e20) for i in W}
with gzip.open('g20_joint_states_all.csv.gz', 'rt') as f:
    for r in csv.DictReader(f):
        t = float(r['t']); j = r['joint']
        if '_a' not in j or not r['eff']:
            continue
        for i, (t0, t1) in W.items():
            if t0 <= t < t1 + 3.0:          # task + dwell tail, before retract
                e = abs(float(r['eff']))
                if e > js[i].get(j, 0): js[i][j] = e
out = []
arms = ['arm_1', 'arm_2', 'arm_3', 'arm_4']; pfx = ['t1_a1_', 't1_a2_', 't2_a1_', 't2_a2_']
for k, i in enumerate(sorted(W)):
    t0, t1 = W[i]
    ev = [c for c in conc if t0 <= c < nxt[i]]
    rr = rows[4 * k: 4 * k + 4]
    assert [r['arm'] for r in rr] == arms
    j2 = [js[i].get(p + 'joint_2', 0) for p in pfx]
    peak = max(js[i].items(), key=lambda x: x[1])
    cross = [d for d in pred[i]['interarm'] if d[1][:2] != d[2][:2]]
    out.append(dict(trial=i, verdict='CONCURRENT' if ev else 'NOT-CONCURRENT', monitor_event_t=ev[:1],
                    settled_mm=[float(r['pos_err_max_settled_mm']) for r in rr],
                    pos_max_mm=[float(r['pos_err_max_mm']) for r in rr],
                    ori_max_deg=max(float(r['ori_err_max_deg']) for r in rr),
                    n_window=[int(r['n_window_samples']) for r in rr],
                    t_to_dwell_s=[float(r['time_to_dwell_s']) for r in rr],
                    j2_meas=[round(x, 3) for x in j2], j2_rnea=pred[i]['rnea_j2'],
                    j2_offset=[round(m - p, 2) for m, p in zip(j2, pred[i]['rnea_j2'])],
                    peak=[peak[0], round(peak[1], 3)],
                    min_interarm_logged_mm=min(d[0] for d in pred[i]['interarm']),
                    min_crossgantry_logged_mm=min((d[0] for d in cross), default=None)))
json.dump(out, open('g20_scored.json', 'w'), indent=1)
print('i | vonis | settled a1..a4 mm | ori maks | j2 terukur a1..a4 | offset vs RNEA | puncak | min antar-lengan / antar-gantry (log)')
for o in out:
    print(o['trial'], o['verdict'], o['settled_mm'], round(o['ori_max_deg'], 2), o['j2_meas'], o['j2_offset'], o['peak'],
          o['min_interarm_logged_mm'], o['min_crossgantry_logged_mm'], o['n_window'])
