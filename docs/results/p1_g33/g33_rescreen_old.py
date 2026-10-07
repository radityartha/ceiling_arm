#!/usr/bin/env python3
"""G33 step 4: re-screen every SAVED trajectory of G28..G32 against the frozen map.
(G22..G27 saved no trajectories -- they cannot be re-screened, reported as such.)

    python3 g33_rescreen_old.py --map env_map_reg3.npz --out g33_rescreen_old.json
"""
from __future__ import annotations

import argparse
import collections
import glob
import gzip
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
from env_collision import EnvChecker  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    ec = EnvChecker(map_file=a.map)
    rep = {}
    for f in sorted(glob.glob(os.path.join(HERE, '..', 'p1_g*', '*plans*.jsonl.gz'))):
        tally, rows, skipped = collections.Counter(), [], 0
        for ln in gzip.open(f, 'rt'):
            r = json.loads(ln)
            tr = r.get('traj')
            if not tr or not tr.get('pos'):
                skipped += 1
                continue
            v, d, g, k = ec.screen_trajectory(tr['joint_names'], tr['pos'], r['start'])
            tally[v] += 1
            rows.append({'arm': r.get('arm'), 'xyz': r.get('xyz'), 'verdict': v,
                         'd_min': round(d, 4), 'geom': g,
                         'g': [r['start'][j] for j in ('t1_linear_joint', 't1_rotation_joint',
                                                       't2_linear_joint', 't2_rotation_joint')]})
        name = os.path.relpath(f, os.path.join(HERE, '..'))
        rep[name] = {'tally': dict(tally), 'no_traj': skipped, 'rows': rows}
        print(name, dict(tally), 'tanpa lintasan', skipped, flush=True)
    json.dump(rep, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
