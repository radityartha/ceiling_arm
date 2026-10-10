"""G44 A1/A3 (docs/p1_g44_start_jump.md): |dq| distribution of every PLANNED plan in the plan-only archives, through
reach_dwell_probe.start_jump (the function the probe uses). OFFLINE.

    python3 g44_measure.py dist           -> quantiles per group (MOCK / NYATA)
    python3 g44_measure.py check T01 TIN  -> K-ARSIP: refusals per archive with the given limits
"""
import glob
import gzip
import json
import os
import sys

import numpy as np

R = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R + '/scripts')
from reach_dwell_probe import start_jump  # noqa: E402

D = R + '/docs/results/'
NYATA = [D + 'p1_g34/g34_planonly_plans_g34b_hw_R0.jsonl', D + 'p1_g34/g34_planonly_plans_g34b_hw_R10.jsonl',
         D + 'p1_g34/g34_planonly_plans_g35b_hw_R0.jsonl', D + 'p1_g34/g34_planonly_plans_g35b_hw_R10.jsonl',
         D + 'p1_g36/g36_planonly_plans_g36b_hw_R0.jsonl', D + 'p1_g36/g36_planonly_plans_g36b_hw_R10.jsonl']


def mock_files():
    out = []
    for g in range(31, 42):
        out += glob.glob(D + f'p1_g{g}*/*plans*.jsonl') + glob.glob(D + f'p1_g{g}*/*plans*.jsonl.gz')
    return sorted(f for f in set(out) if '_hw_' not in f and 'hw_screen' not in f)


def rows(path):
    op = gzip.open if path.endswith('.gz') else open
    with op(path, 'rt') as f:
        for ln in f:
            d = json.loads(ln)
            if d.get('verdict') != 'PLANNED' or not d.get('traj'):
                continue
            tr = d['traj']
            st = d.get('start') or {}
            start = [st[j] for j in tr['joint_names']] if all(j in st for j in tr['joint_names']) else None
            yield d, start_jump(tr['pos'], start)


def dist():
    for name, files in (('MOCK', mock_files()), ('NYATA', NYATA)):
        a = []
        for f in files:
            a += [(m, os.path.basename(f), d) for d, m in rows(f)]
        m = np.array([x[0] for x in a])
        print(f'{name}: {len(a)} rencana PLANNED dari {len(files)} arsip')
        for i, lab in enumerate(('d01', 'd_in', '|p0-start|')):
            v = m[:, i]
            print(f'  {lab:11s} p50 {np.median(v):.5f}  p99 {np.percentile(v, 99):.5f}  p99.9 '
                  f'{np.percentile(v, 99.9):.5f}  maks {v.max():.5f} rad')
        print(f'  d01 > d_in: {int((m[:, 0] > m[:, 1]).sum())}')
        top = sorted(a, key=lambda x: -x[0][0])[:8]
        for (d01, din, d0s), fn, d in top:
            print(f"    d01 {d01:.4f} d_in {din:.4f} p0-start {d0s:.4f}  {fn} seed {d.get('seed')} task {d.get('task')} "
                  f"sample {d.get('sample')} {d.get('arm')} n {len(d['traj']['pos'])}")


def check(t01, tin):
    for name, files in (('MOCK', mock_files()), ('NYATA', NYATA)):
        tot = bad = 0
        for f in files:
            for d, (d01, din, d0s) in rows(f):
                tot += 1
                if d01 > t01 or din > tin or d0s > t01:
                    bad += 1
                    print(f"  TOLAK {os.path.basename(f)} seed {d.get('seed')} task {d.get('task')} sample "
                          f"{d.get('sample')} {d.get('arm')}: d01 {d01:.4f} d_in {din:.4f} p0-start {d0s:.4f}")
        print(f'{name}: ditolak {bad}/{tot}, diterima {tot - bad}/{tot}')


if __name__ == '__main__':
    if sys.argv[1] == 'dist':
        dist()
    else:
        check(float(sys.argv[2]), float(sys.argv[3]))
