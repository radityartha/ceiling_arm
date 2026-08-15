#!/usr/bin/env python3
"""G13 diagnostics: A1 step 4 (why n6_s4_mr0 is caged) and step 6 (U4).

Both are DIAGNOSTICS, not headline numbers (A2.4): neither is allowed to move a
Delta, and neither runs at a budget other than the one it names.

  n6    p1_g12 B10.3: n6_s4_mr0 is the ONE S1 instance still bracketed at BOTH
        120 s and 600 s, 45 nodes in 600 s = 13 s/node, 38 % of wall in the arm
        gate. The question A1 asks is WHERE THE OTHER 62 % GOES before anyone
        raises the budget a third time. If it is `_dive`, that is FROZEN code
        and the finding is bigger than the instance.

  u4    p1_g10 B9: on 9 of 31 W2 instances `solver < W2`, and halving the grid
        closed 0 of 9 -- so the gap is not the grid. The stated suspect is
        `max_evade = 1` per gantry in the enumerator while the solver uses up to
        3 evasive moves (Q3). UNMEASURED since G10. This measures it: re-run W2
        on exactly those instances with max_evade = 2, and ask whether W2 comes
        DOWN to the solver.

Run:  python3 test/diag_g13.py n6 | u4
"""

from __future__ import annotations

import cProfile
import pstats
import sys
import time
from io import StringIO
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
from reachability_gng import sched_armfull as af            # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from verify_sched_coupled import Brute, DS, s5_instances     # noqa: E402


def n6(key='n6_s4_mr0', n=6, seed=4, mr=0, c_arm=0.05, budget=120.0):
    inst = sched.gen_real(n, seed, mr, (1, 2))
    geom = sa.ArmGeom(inst)
    pr = cProfile.Profile()
    t0 = time.time()
    pr.enable()
    s, ctx = af.solve_armfull(inst, c_arm=c_arm, geom=geom, c_clear=0.0,
                              time_budget=budget)
    pr.disable()
    wall = time.time() - t0
    print(f'{key} c_arm={c_arm} budget={budget}s')
    print(f'  makespan {s.makespan:.4f}  lb {s.lb:.4f}  route {s.route}  '
          f'proved {s.proved}  nodes {s.n_nodes}')
    print(f'  wall {wall:.1f}s   arm gate {ctx.calls} calls {ctx.t_arm:.1f}s '
          f'= {100 * ctx.t_arm / max(wall, 1e-9):.1f}%')
    print(f'  budget counters: {s.budget}')
    buf = StringIO()
    st = pstats.Stats(pr, stream=buf).sort_stats('cumulative')
    st.print_stats(28)
    lines = buf.getvalue().splitlines()
    print('\n  --- cumulative, top frames (the 62 % question) ---')
    for ln in lines:
        if ('sched' in ln or 'irm_sweep' in ln or 'ncalls' in ln
                or '{built-in' in ln):
            print('  ' + ln.rstrip()[:150])
    return 0


def u4(evades=(2,), budget=240.0, ds=DS):
    """W2 with a bigger evade cap on the instances where solver < W2."""
    rows = []
    for label, inst in s5_instances():
        s = sk.solve_coupled2(inst, time_budget=120.0)
        b1 = Brute(inst, ds=ds, max_evade=1, budget=budget)
        v1, _ = b1.solve()
        if not (s.makespan < v1 - 1e-9):
            continue                      # not one of the 9
        row = dict(label=label, solver=float(s.makespan), w2_1=float(v1),
                   to1=bool(b1.timeout), nodes1=int(b1.nodes))
        for me in evades:
            b = Brute(inst, ds=ds, max_evade=me, budget=budget)
            t0 = time.time()
            v, _ = b.solve()
            row[f'w2_{me}'] = float(v)
            row[f'to{me}'] = bool(b.timeout)
            row[f'nodes{me}'] = int(b.nodes)
            row[f'wall{me}'] = time.time() - t0
        rows.append(row)
        e = evades[-1]
        print(f'  {label:22s} solver {row["solver"]:9.4f}  W2(me=1) '
              f'{row["w2_1"]:9.4f}{" TO" if row["to1"] else "   "}  '
              f'W2(me={e}) {row[f"w2_{e}"]:9.4f}'
              f'{" TO" if row[f"to{e}"] else "   "}  '
              f'nodes {row["nodes1"]} -> {row[f"nodes{e}"]}  '
              f'{row[f"wall{e}"]:.0f}s  '
              f'{"CLOSED" if row[f"w2_{e}"] <= row["solver"] + 1e-9 else "still above"}',
              flush=True)
    if not rows:
        print('U4: 0 instances with solver < W2 -- nothing to close')
        return 0
    e = evades[-1]
    closed = sum(r[f'w2_{e}'] <= r['solver'] + 1e-9 for r in rows)
    down = sum(r[f'w2_{e}'] < r['w2_1'] - 1e-9 for r in rows)
    to = sum(r[f'to{e}'] for r in rows)
    print(f'\nU4: {len(rows)} instances with solver < W2 at max_evade = 1')
    print(f'    max_evade = {e}: W2 came DOWN on {down}, CLOSED to the solver '
          f'on {closed}, timed out on {to}')
    print('    (a timeout is NOT a closure -- reported separately, p1_g12 A2.4)')
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['n6']
    if a[0] == 'u4':
        return u4(evades=tuple(int(x) for x in a[1:]) or (2,))
    return n6(budget=float(a[1]) if len(a) > 1 else 120.0)


if __name__ == '__main__':
    raise SystemExit(main())
