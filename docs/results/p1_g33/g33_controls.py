#!/usr/bin/env python3
"""G33 A4 controls against the frozen map, offline (no ROS graph, no motion).

(P+) the EXECUTED arm_3 path of G32-HW R10 ev 8 (joint_states1) up to the contact
     torque peak must reach d < margin  -> gate.
(N-) every EXECUTED event of G32-HW R0 (6/6, no contact) must stay CLEAR.
plus R10 ev 0..7 (no contact) and the saved plan samples (reported).

    python3 g33_controls.py --map env_static_map.npz --out g33_controls.json
"""
from __future__ import annotations

import argparse
import bisect
import csv
import gzip
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
from env_collision import CONFIG_JOINTS, EnvChecker  # noqa: E402

G32 = os.path.join(HERE, '..', 'p1_g32hw')
T_CONTACT = 1791339437.6981     # t2_a1_joint_2 peak 12.2994 N.m (joint_states1)
STEP = 0.25


class States:
    def __init__(self, path):
        self.t, self.p = {}, {}
        for r in csv.DictReader(gzip.open(path, 'rt')):
            if r['joint'] in CONFIG_JOINTS and r['pos']:
                self.t.setdefault(r['joint'], []).append(float(r['t']))
                self.p.setdefault(r['joint'], []).append(float(r['pos']))
        miss = [j for j in CONFIG_JOINTS if j not in self.t]
        if miss:
            raise KeyError(f'{path}: tanpa {miss} -- TOLAK')

    def at(self, t):
        out = {}
        for j in CONFIG_JOINTS:
            k = bisect.bisect_right(self.t[j], t) - 1
            if k < 0 or t - self.t[j][k] > 2.0:
                raise ValueError(f'{j} basi/tidak ada pada t={t:.2f} -- TOLAK')
            out[j] = self.p[j][k]
        return out


def screen_window(ec, st, t0, t1, step=STEP):
    best = (float('inf'), None, None)
    trace = []
    for t in np.arange(t0, t1 + 1e-9, step):
        d, g = ec.check(ec.q_from(st.at(t)))
        trace.append((round(float(t), 2), round(float(d), 4), g))
        if d < best[0]:
            best = (d, g, float(t))
    return best, trace


def verdict(d, m):
    return 'COLLIDE' if d <= 0 else ('MARGIN' if d < m else 'CLEAR')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    ec = EnvChecker(map_file=a.map)
    st = States(os.path.join(G32, 'g32hw_joint_states1.csv.gz'))
    rep = {'map': a.map, 'n_voxels': ec.n_voxels, 'margin': ec.margin}

    # (P+) ev 8 executed path
    r10 = json.load(open(os.path.join(G32, 'g32hw_R10_run.json')))
    e8 = r10['events'][8]
    (d, g, t), tr = screen_window(ec, st, e8['t_start'], T_CONTACT + 0.5, 0.1)
    dc, gc = ec.check(ec.q_from(st.at(T_CONTACT)))
    rep['P+'] = {'kind': e8['kind'], 'd_min': d, 'geom': g, 't_rel_contact': t - T_CONTACT,
                 'verdict': verdict(d, ec.margin), 'd_at_contact': dc, 'geom_at_contact': gc,
                 'first_reject_t_rel': next((x[0] - T_CONTACT for x in tr if x[1] < ec.margin), None),
                 'trace_tail': tr[-60:]}
    print(f"(P+) ev8 {e8['kind']}: d_min {d:+.3f} {g} @ {t - T_CONTACT:+.2f}s -> "
          f"{rep['P+']['verdict']}; at contact {dc:+.3f} {gc}; first reject "
          f"{rep['P+']['first_reject_t_rel']}")

    # (N-) R0 executed events + R10 ev 0..7
    for name, run, upto in (('R0', 'g32hw_R0_run.json', None), ('R10', 'g32hw_R10_run.json', 8)):
        evs = json.load(open(os.path.join(G32, run)))['events'][:upto]
        rows = []
        for i, e in enumerate(evs):
            (d, g, t), _ = screen_window(ec, st, e['t_start'], e['t_end'])
            rows.append({'ev': i, 'kind': e['kind'], 'skipped': e.get('skipped', False),
                         'd_min': d, 'geom': g, 'verdict': verdict(d, ec.margin)})
            print(f"({name}) ev{i:02d} {e['kind']:9s} d_min {d:+.3f} {g:32s} {rows[-1]['verdict']}")
        rep[name] = rows

    # saved plan samples (plan-only, R0 and R10): each trajectory from its own start
    for v in ('R0', 'R10'):
        rows = []
        for ln in gzip.open(os.path.join(G32, f'g32hw_screen_plans_{v}.jsonl.gz'), 'rt'):
            r = json.loads(ln)
            vv, d, g, k = ec.screen_trajectory(r['traj']['joint_names'], r['traj']['pos'],
                                               r['start'])
            rows.append({'sample': r['sample'], 'k': r['k'], 'arm': r['arm'], 'verdict': vv,
                         'd_min': d, 'geom': g, 'wp': k,
                         'g2': [r['start']['t2_linear_joint'], r['start']['t2_rotation_joint']]})
        rep[f'plans_{v}'] = rows
        print(v, 'plans', [(x['k'], x['arm'], x['verdict'], round(x['d_min'], 3)) for x in rows])
    json.dump(rep, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
