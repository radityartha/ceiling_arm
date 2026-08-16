#!/usr/bin/env python3
"""G16: the 2x2x2 corner lattice. docs/p1_g16_interact.md A2.0-A2.6.

G15 ruled out the third and last single factor: task crowding gave 61 two-sided
proofs of Delta_arm = 0 on S2's TASKS in S1's FULL pose space. All three ways S2
differs from S1 are now null ON THEIR OWN -- 493 two-sided proofs across three
instance families, zero counterexamples, against the one proved positive that
has ever existed (n6_s0_mr1 on S2, +5.40 %).

A2.0 fixes the lattice. A corner is (POSE set, TASK pool, p0 location), and P1
has so far touched four of the eight:

    (S1,S1,S1) = S1               432 proofs        G13 + G14
    (S2,S1,S1) = GEO-S2           0 pos / 14        G14 B7
    (S1,S2,S1) = DENSE            0 pos / 61        G15 B3
    (S2,S2,S2) = S2               ONE positive      G13 B5.4
    (S1,S1,S2) = P0XC             this session, task 2   <- the CLEAN p0 axis
    (S1,S2,S2) = DENSE-P0XC       this session, task 1
    (S2,S1,S2) = GEO-P0XC         this session, task 3
    (S2,S2,S1) = GEO-DENSE        this session, task 3

A2.1: a corner is comparable only if it comes from ONE generator family and
differs from its neighbours only on the axis that names it. There is no new
generator here. Every corner is

    dense_g15.gen_real_dense(band, reject_at_p0, p0_mode)  [+ psweep_g14.shrink]

and R4 already PASSED in G15: corner(S1,S1,S1) == sched.gen_real on 40/40 keys.
So the lattice's origin is provably the same family that produced the 432
proofs, and every other corner is S1 plus a named argument.

A2.1.1 adds R5, the bar this file exists to run FIRST. Reading
gen_real_rotcrowded:576-625 beside gen_real_dense:121-139 shows the lattice's
(S2,S2,S2) corner is a RECONSTRUCTION of S2, not S2: rotcrowded tests the task
acceptance predicate INSIDE the pose mask (`hd[0][keep]`) and filters duplicate
draws, while gen_real_dense tests on the FULL grid and (deliberately, for R4)
does not filter. If the reconstruction does not carry the counterexample, then
no corner of the lattice can explain it -- and A2.1.1 locks, in advance, that
the word "interaction" is then FORBIDDEN.

A2.0 also states the honesty this design needs: the TASK axis and the p0 axis
are NOT independent, because gen_real_dense's second change rejects candidates
doable at p0. G15 B1 measured the size of that coupling: 246 usable SR nodes
with S1's p0 against 38 with S2's. So the only CLEAN p0 contrast is S1 <-> P0XC,
where the at_p0 rejection is off and node_idx is identical by R4.

The measurement loop, the gates, the JSON schema and the report are NOT rewritten
here -- p1_g13 B3 measured what a second instrument of the same shape costs. This
file supplies `build` and `_path` and hands them to dense_g15.cell / .report /
.probe / .park, which are FROZEN this session (A0).

Run:  python3 test/corner_g16.py gates                 # R4' + R5
      python3 test/corner_g16.py probe DENSE-P0XC      # A2.5, cost first
      python3 test/corner_g16.py cell DENSE-P0XC 0.15
      python3 test/corner_g16.py report
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
# A0/A5: imported, never copied. dense_g15 and psweep_g14 are FROZEN.
import dense_g15 as dg                                      # noqa: E402
from psweep_g14 import keys40, s2_mask, shrink              # noqa: E402

# ------------------------------------------------------------------ lattice
# (pose, task, p0) -> generator arguments. A2.1, locked before this ran.
CORNERS = {
    #                     pose  task  p0     band  reject_at_p0  p0_mode
    'S1':               dict(band=2.00, reject_at_p0=False, p0_mode='s1', mask=False),
    'P0XC':             dict(band=2.00, reject_at_p0=False, p0_mode='xc', mask=False),
    'DENSE':            dict(band=0.30, reject_at_p0=True,  p0_mode='s1', mask=False),
    'DENSE-P0XC':       dict(band=0.30, reject_at_p0=True,  p0_mode='xc', mask=False),
    'GEO-S2':           dict(band=2.00, reject_at_p0=False, p0_mode='s1', mask=True),
    'GEO-P0XC':         dict(band=2.00, reject_at_p0=False, p0_mode='xc', mask=True),
    'GEO-DENSE':        dict(band=0.30, reject_at_p0=True,  p0_mode='s1', mask=True),
    'GEO-DENSE-P0XC':   dict(band=0.30, reject_at_p0=True,  p0_mode='xc', mask=True),
}
TRIPLE = {
    'S1': ('S1', 'S1', 'S1'), 'P0XC': ('S1', 'S1', 'S2'),
    'DENSE': ('S1', 'S2', 'S1'), 'DENSE-P0XC': ('S1', 'S2', 'S2'),
    'GEO-S2': ('S2', 'S1', 'S1'), 'GEO-P0XC': ('S2', 'S1', 'S2'),
    'GEO-DENSE': ('S2', 'S2', 'S1'), 'GEO-DENSE-P0XC': ('S2', 'S2', 'S2'),
}
# Corners already measured elsewhere; never recomputed (A0).
DONE = {'S1': 'g13/g14', 'DENSE': 'g15 B3', 'GEO-S2': 'g14 B7'}

NEW = ('DENSE-P0XC', 'P0XC', 'GEO-DENSE', 'GEO-P0XC')


# ------------------------------------------------------------------- build
def build(arm, n, seed, mr, band=None):
    """(instance, note). Signature matches dense_g15.build so cell() can use it.

    `band` is ignored on purpose: the band is part of the corner's identity
    (A2.1), not a free axis, so it comes from CORNERS and never from the caller.
    """
    if arm not in CORNERS:
        return None, f'unknown corner {arm}'
    kw = dict(CORNERS[arm])
    mask = kw.pop('mask')
    try:
        inst = dg.gen_real_dense(n, seed, mr, **kw)
    except (RuntimeError, ValueError) as e:
        return None, f'generator: {e}'
    if mask:
        keep = np.union1d(s2_mask(), [int(inst.p0[inst.gantries[0]])])
        inst = shrink(inst, keep, arm)
    bad = dg.r3(inst)                      # R3, imported not rewritten
    return (None, bad) if bad else (inst, None)


def _path(arm, c, band=None):
    return Path(f'/tmp/g16_corner_{arm}_{c:.2f}.json')


# A0/A5: dense_g15's measurement loop is USED, not copied. The only things this
# file supplies are the corner build and the artefact path.
dg.build = build
dg._path = _path
dg.ARMS = {k: {} for k in CORNERS}


# ------------------------------------------------------------------- gates
def _grid_p0(inst):
    """p0 as a (lin, rot) pair, so masked and unmasked corners are comparable."""
    g = inst.gantries[0]
    return tuple(np.round(inst.poses[g][int(inst.p0[g])], 6))


def _pose_set(inst):
    g = inst.gantries[0]
    return {tuple(np.round(p, 6)) for p in inst.poses[g]}


def r4prime():
    """A2.1: each new corner differs from its neighbour ONLY on its own axis."""
    print('R4\' -- a corner differs from its neighbour only on the axis that '
          'names it\n     (A2.1, locked before this ran)')
    pairs = [('S1', 'P0XC', 'p0'), ('GEO-S2', 'GEO-P0XC', 'p0'),
             ('DENSE', 'GEO-DENSE', 'pose'),
             ('DENSE-P0XC', 'GEO-DENSE-P0XC', 'pose')]
    rc = 0
    for a, b, axis in pairs:
        bad, nodes_same, p0_same, pose_ok = [], 0, 0, 0
        for key, n, seed, mr in keys40():
            ia, na = build(a, n, seed, mr)
            ib, nb = build(b, n, seed, mr)
            if ia is None or ib is None:
                bad.append((key, na or nb))
                continue
            if ia.meta['nodes'] == ib.meta['nodes']:
                nodes_same += 1
            if _grid_p0(ia) == _grid_p0(ib):
                p0_same += 1
            sa_, sb = _pose_set(ia), _pose_set(ib)
            if axis == 'p0':
                # the mask keeps s2_mask U {p0}, so the pose sets may differ by
                # exactly the two p0 elements -- nothing else.
                pose_ok += (sa_ - {_grid_p0(ia)}) == (sb - {_grid_p0(ib)})
            else:
                pose_ok += sb <= sa_
        n_ok = 40 - len(bad)
        need_nodes = n_ok
        need_p0 = 0 if axis == 'p0' else n_ok
        ok = (nodes_same == need_nodes and pose_ok == n_ok
              and (p0_same == need_p0))
        rc |= 0 if ok else 1
        print(f'  {a:16s} <-> {b:16s}  axis={axis:4s}  '
              f'node_idx sama {nodes_same}/{n_ok}  '
              f'p0 sama {p0_same}/{n_ok}  pose ok {pose_ok}/{n_ok}  '
              f'{"OK" if ok else "GAGAL"}'
              + (f'  [{len(bad)} tak-terbangun]' if bad else ''))
    print('  ' + ('OK R4\' LULUS' if not rc else
                  'R4\' GAGAL -- A3: sudut itu TIDAK DIJALANKAN'))
    return rc


def r5():
    """A2.1.1: is the lattice's (S2,S2,S2) corner the SAME thing as real S2?

    The verdict table is locked in A2.1.1 before this ran. This measures the
    generator side (node_idx / |P| / p0); the Delta_arm side is measured by
    `r5arm`, which needs the solver.
    """
    print('\nR5 -- corner(S2,S2,S2) vs sched_arm.gen_real_rotcrowded, 40 keys')
    print('     A2.1.1: this decides whether the lattice CONTAINS the one '
          'positive P1 has')
    same_nodes = same_np = same_p0 = built = 0
    jac, ex = [], []
    for key, n, seed, mr in keys40():
        ia, note = build('GEO-DENSE-P0XC', n, seed, mr)
        if ia is None:
            continue
        ib = sa.gen_real_rotcrowded(n, seed, mr)
        built += 1
        A, B = ia.meta['nodes'], ib.meta['nodes']
        same_nodes += A == B
        same_np += len(ia.poses[1]) == len(ib.poses[1])
        same_p0 += _grid_p0(ia) == _grid_p0(ib)
        jac.append(len(set(A) & set(B)) / len(set(A) | set(B)))
        if len(ex) < 3 and A != B:
            ex.append((key, A, B))
    print(f'  dibangun {built}/40   node_idx identik {same_nodes}/{built}   '
          f'|P| identik {same_np}/{built}   p0 identik {same_p0}/{built}')
    print(f'  Jaccard node_idx: mean {np.mean(jac):.3f}  '
          f'median {np.median(jac):.3f}  max {max(jac):.3f}')
    print(f'  |P|: kisi {len(build("GEO-DENSE-P0XC", 6, 0, 1)[0].poses[1])}  '
          f'S2 {len(sa.gen_real_rotcrowded(6, 0, 1).poses[1])}')
    for key, A, B in ex:
        print(f'    {key:12s} kisi {A}\n    {"":12s} S2   {B}')
    if same_nodes == built:
        print('  OK R5: rekonstruksi == S2. Kisi memuat fenomenanya.')
    else:
        print(f'  R5: rekonstruksi != S2 pada {built - same_nodes}/{built}. '
              'A2.1.1 baris 2/3 berlaku -- vonis menunggu r5arm.')
    return 0


def r5arm(key='n6_s0_mr1', c=0.15):
    """A2.1.1 second half: does the RECONSTRUCTION carry the counterexample?

    G13 B5.4 measured Delta_arm = +0.5000 (+5.40 %) for this key on real S2 at
    c_arm 0.15 and 0.20. If the reconstruction gives 0 here, the lattice cannot
    reach the only positive P1 has, and A2.1.1 row 3 forbids the word
    "interaction".
    """
    from reachability_gng import sched_armfull as af
    from reachability_gng import sched_coupled as sk
    n, seed, mr = (int(key.split('_')[0][1:]), int(key.split('_')[1][1:]),
                   int(key.split('_')[2][2:]))
    print(f'\nR5-ARM -- {key} c_arm={c}: does the reconstruction carry the '
          f'+5.40 % counterexample?')
    for tag, inst in (('S2 sungguhan', sa.gen_real_rotcrowded(n, seed, mr)),
                      ('rekonstruksi', build('GEO-DENSE-P0XC', n, seed, mr)[0])):
        if inst is None:
            print(f'  {tag:14s} TIDAK DAPAT DIBANGUN')
            continue
        geom = sa.ArmGeom(inst)
        t0 = time.time()
        ss = sk.solve_coupled2(inst, c_clear=0.0, time_budget=dg.SEARCH_BUDGET)
        sf, _ = af.solve_armfull(inst, c_arm=c, geom=geom, c_clear=0.0,
                                 time_budget=dg.SEARCH_BUDGET)
        d = sf.makespan - ss.makespan
        print(f'  {tag:14s} |P|={len(inst.poses[1]):4d} lb {sf.lb:8.3f} '
              f'str {ss.makespan:9.4f}({ss.route},pr={int(ss.proved)}) '
              f'full {sf.makespan:9.4f}({sf.route},pr={int(sf.proved)}) '
              f'D_arm {d:+.4f} ({100.0*d/ss.makespan:+.2f} %) '
              f'{time.time()-t0:.1f}s', flush=True)
    return 0


def gates():
    print('KISI 2x2x2 -- (pose, tugas, p0). A2.0, dikunci sebelum kode.\n')
    for k, t in TRIPLE.items():
        print(f'  {k:16s} ({t[0]},{t[1]},{t[2]})  '
              f'{DONE.get(k, "SESI INI") if k in DONE or k in NEW else "-"}')
    print()
    rc = r4prime()
    r5()
    return rc


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['report']
    if a[0] == 'gates':
        return gates()
    if a[0] == 'r5arm':
        return r5arm(*(a[1:2] or []), **({'c': float(a[2])} if len(a) > 2
                                         else {}))
    if a[0] == 'probe':
        return dg.probe(arm=a[1] if len(a) > 1 else 'DENSE-P0XC')
    if a[0] == 'park':
        return dg.park(a[2] if len(a) > 2 else 'n6_s0_mr1',
                       float(a[3]) if len(a) > 3 else 0.20,
                       arm=a[1] if len(a) > 1 else 'DENSE-P0XC')
    if a[0] == 'cell':
        return dg.cell(a[1], float(a[2]))
    if a[0] == 'report':
        return dg.report(list(a[1:]) or list(NEW))
    raise SystemExit(f'unknown: {a[0]}')


if __name__ == '__main__':
    raise SystemExit(main())
