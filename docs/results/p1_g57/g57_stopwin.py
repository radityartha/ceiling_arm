"""G57 B2: recorder window around an AUTO-STOP — per arm, range of every joint from t_stop+a to t_stop+b (deg) and motion
in the last 1 s before the stop.   python3 g57_stopwin.py <joint_states.csv> <t_stop>"""
import collections
import csv
import math
import sys

f, ts = sys.argv[1], float(sys.argv[2])
rows = collections.defaultdict(list)
with open(f) as fh:
    r = csv.reader(fh)
    next(r)
    for t, j, p, e in r:
        if not p:
            continue
        t = float(t)
        if ts - 3 <= t <= ts + 8:
            rows[j].append((t, float(p)))


def rng(j, a, b):
    w = [p for t, p in rows[j] if ts + a <= t <= ts + b]
    return math.degrees(max(w) - min(w)) if len(w) > 1 else None


for pre in ('t2_a1', 't2_a2', 't1_a1', 't1_a2'):
    after = max(rng(f'{pre}_joint_{k}', 1, 4) or 0 for k in range(1, 7))
    after05 = max(rng(f'{pre}_joint_{k}', 0.5, 4) or 0 for k in range(1, 7))
    before = max(rng(f'{pre}_joint_{k}', -1, 0) or 0 for k in range(1, 7))
    print(f'{pre}: rentang +1..+4 s {after:.4f} deg | +0.5..+4 s {after05:.4f} | 1 s sebelum stop {before:.3f}')
for k in range(1, 7):
    j = f't2_a1_joint_{k}'
    print(j, [(round(t - ts, 1), round(math.degrees(p), 2)) for t, p in rows[j] if -1 <= t - ts <= 3][::5])
for j in ('t1_linear_joint', 't1_rotation_joint', 't2_linear_joint', 't2_rotation_joint'):
    w = [p for t, p in rows[j] if ts <= t <= ts + 6]
    print(j, (round(min(w), 5), round(max(w), 5)) if w else None)
