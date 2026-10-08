"""G36 B: plan-only mock tally from g36_planonly.log + g36_planonly_<tag>_<V>.json + the plans jsonl.
Per variant: samples LOLOS / n with Clopper-Pearson 95 % on the failure rate; task attempts per verdict; RETRACT-BLOCKED
per event; every retract event verdict (lurus / moveit); R10 ev 10 accepted arm_3 joint_1 at the end (branch).
    python3 g36_analyze.py [tag] > g36_analyze.log"""
import json
import math
import re
import sys
from collections import Counter

from scipy.stats import beta

D = '/home/user1/Documents/ceiling_arm/docs/results/p1_g36/'
TAG = sys.argv[1] if len(sys.argv) > 1 else 'g36'


def cp(k, n):
    return (0.0 if k == 0 else beta.ppf(0.025, k, n - k + 1), 1.0 if k == n else beta.ppf(0.975, k + 1, n - k))


log = open(D + 'g36_planonly.log').read()
blocks = re.split(r'^== (R10|R0) sampel ', log, flags=re.M)[1:]
for V in ('R10', 'R0'):
    try:
        runs = json.load(open(D + f'g36_planonly_{TAG}_{V}.json'))
    except FileNotFoundError:
        print(f'{V}: belum ada')
        continue
    ok = sum(r['ok'] for r in runs)
    lo, hi = cp(len(runs) - ok, len(runs))
    print(f'{V}: LOLOS {ok}/{len(runs)}  laju gagal {(len(runs) - ok) / len(runs):.2f}, CI95 [{lo:.3f}, {hi:.3f}]')
    for r in runs:
        if not r['ok']:
            print('   ditolak:', r['samples'])
    att, blk, ret = Counter(), Counter(), Counter()
    for v, b in zip(blocks[0::2], blocks[1::2]):
        if v != V:
            continue
        for m in re.finditer(r'\[(\d+)\]   percobaan tugas: (\S+)', b):
            att[m.group(2)] += 1
            if m.group(2) == 'RETRACT-BLOCKED':
                blk[int(m.group(1))] += 1
        for m in re.finditer(r'\[(\d+)\] retract g(\d) @rot ([+-]\d+): (\S+)(?: \((\S+)\))?', b):
            ret[(int(m.group(1)), m.group(4), m.group(5))] += 1
    print(f'   percobaan tugas: {dict(att)}')
    print(f'   RETRACT-BLOCKED per event: {dict(sorted(blk.items()))}')
    print(f'   retract per event: {dict(sorted(ret.items()))}')
    if V == 'R10':
        j1 = []
        for r in map(json.loads, open(D + f'g36_planonly_plans_{TAG}_R10.jsonl')):
            if r['k'] == 10 and r.get('verdict') == 'PLANNED':
                t = r['traj']
                j1.append(round(math.degrees(t['pos'][-1][t['joint_names'].index('t2_a1_joint_1')]), 1))
        print(f'   ev 10 diterima: arm_3 joint_1 akhir {j1} -> cabang +100: {sum(x > 60 for x in j1)}/{len(j1)}')
