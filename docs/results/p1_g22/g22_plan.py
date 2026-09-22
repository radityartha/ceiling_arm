"""G22 shared: A3 event order + S24 traverse sweep screen.

ONE definition used by BOTH the plan-only screen (sched_screen.py, A2 (iv)) and
the hardware runner (run_g22.py), so the order that was screened is the order
that runs. docs/p1_g22_hw.md A3 / A5.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
ARMS = {1: ('arm_1', 'arm_2'), 2: ('arm_3', 'arm_4')}
SCHED_ARM = {'arm1': 'arm_1', 'arm2': 'arm_2', 'arm3': 'arm_3', 'arm4': 'arm_4'}
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
V_LIN = 3000 / 95.4930 / 1000.0          # m/s, p1_state 5.6
RAIL_MAX = 1.600                         # p1_state 5.7
SWEEP_STEP = 0.010                       # S24: <= 10 mm between sampled rail positions
MARGIN = 0.05                            # S18/S24


def load_row(seed, path=None):
    rows = json.load(open(path or os.path.join(HERE, 'g22_candidates.json')))
    r = [x for x in rows if x['seed'] == seed]
    assert len(r) == 1, seed
    return r[0]


def p0_rails(row):
    """R1 as it was applied when the row was made (first stop start 0 at p0, else
    from the first leg). Read from decomp legs, never re-derived."""
    out = {}
    for g in (1, 2):
        legs = row['decomp']['FISIK'][str(g)]['legs']
        st = row['schedule_FISIK'][str(g)]
        out[g] = legs[0]['from_mm'] / 1000.0 if legs else st[0]['rail_m']
    return out


def events(row):
    """A3, verbatim: block g=1 then g=2; per stop: [retract, traverse] if the
    rail changes, then tasks -- per arm by task index, arms alternating, slot-0
    arm (arm_1 / arm_3) first."""
    ev = []
    rail = p0_rails(row)
    for g in (1, 2):
        for k, st in enumerate(row['schedule_FISIK'][str(g)]):
            if abs(st['rail_m'] - rail[g]) > 1e-9:
                ev.append(dict(kind='retract', gantry=g, stop=k))
                ev.append(dict(kind='traverse', gantry=g, stop=k,
                               frm=rail[g], to=st['rail_m']))
                rail[g] = st['rail_m']
            per = {a: sorted(int(t) for t, who in st['tasks'].items()
                             if SCHED_ARM[who] == a) for a in ARMS[g]}
            for j in range(max(len(v) for v in per.values())):
                for a in ARMS[g]:
                    if j < len(per[a]):
                        t = per[a][j]
                        ev.append(dict(kind='task', gantry=g, stop=k, arm=a,
                                       task=t, xyz=st['xyz'][str(t)]))
    return ev


def allowed_rails(row, g):
    return sorted({round(p0_rails(row)[g], 3)} |
                  {round(s['rail_m'], 3) for s in row['schedule_FISIK'][str(g)]})


def t_cmd(d_m):
    """g19 rail_to: cosine, peak setpoint speed 0.9 v_lin, >= 3 s."""
    return max(3.0, math.pi * abs(d_m) / (2 * 0.9 * V_LIN))


def sweep_screen(checker, g, frm, to, state):
    """S24. Rail of gantry g swept frm -> to (<= 10 mm steps) with its arms
    as given in `state` (REST when S12 holds), everything else HELD at `state`.
    CrossGantryChecker, all arm-bearing t1 x t2 pairs. (verdict, d, pair, k)."""
    n = max(2, int(math.ceil(abs(to - frm) / SWEEP_STEP)) + 1)
    rj = f't{g}_linear_joint'
    pts = [[x] for x in np.linspace(frm, to, n)]
    held = {k: v for k, v in state.items() if k != rj}
    return checker.screen_trajectory([rj], pts, held)
