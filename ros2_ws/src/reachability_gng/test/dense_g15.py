#!/usr/bin/env python3
"""G15 tasks 1-3: TASK CROWDING, the last candidate standing.
docs/p1_g15_dense.md A2.0-A2.6.

G14 B7 exhausted POSE SPACE as the explanation: 432 proofs of Delta_arm = 0 on
S1 (285 in G14 + 147 in G13), zero counterexamples, across |P| 2376 -> 148 AND
across S2's own pose mask. The only proved positive price in P1 is still one:
n6_s0_mr1 on S2, +5.40 %.

Of the three ways S2 differs from S1 that G14 A2.2 named, two are now ruled out
and the third was uncontrolled by design:

  (1) CARDINALITY   ruled out -- 8 CARD cells, 208 two-sided proofs, 0 positive
  (2) GEOMETRY      ruled out -- 4 GEO cells, 0 positive, including the |P|-
                    identical twins CARD-507 <-> GEO-S2
  (3) TASK CROWDING NOT MEASURED -- both G14 arms carried S1's tasks

This builds the one combination G14 could not: S2's TASKS (pool x_c +- 0.30,
rejected if doable at p0) on S1's FULL pose space (|P| = 2376).

A2.0 names a FOURTH difference that G14 did not: gen_real puts p0 at
(lin = 0.00, rot = 0) while gen_real_rotcrowded puts it at (lin = 0.80, rot = 0)
-- 0.80 m of rail, ~25 s of traverse, on a problem that is 74-92 % gantry
motion. So there are two arms:

  DENSE        p0 at S1's rail zero   -- isolates (3) alone     [priority 1]
  DENSE-P0XC   p0 at x_c = 0.80       -- adds (4)               [priority 2]

gen_real_dense is `sched.gen_real` with EXACTLY TWO changes (A2.1), and R4 is
the gate that proves it: with band = 2.00 and the at_p0 rejection disabled it
must reproduce gen_real's node_idx exactly on all 40 keys. Without R4 a
Delta_arm difference cannot be attributed to task crowding -- that is the M3
trap of p1_g12.

A2.2 states in advance that G14's 10 % lb-drift bar does NOT apply here: the
tasks are different BY CONSTRUCTION, that being the knob. Delta_arm is and
always was a WITHIN-instance quantity (two solvers, one instance), so it needs
no cross-instance comparability. lb is reported as description, not as a gate.

Run:  python3 test/dense_g15.py gates            # R4 + R3 + pool census
      python3 test/dense_g15.py probe            # A2.5, cost BEFORE the cells
      python3 test/dense_g15.py cell DENSE 0.15
      python3 test/dense_g15.py report
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
from reachability_gng import sched_armfull as af            # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from reachability_gng.capability import CapabilityMap       # noqa: E402
from gate_sched_coupled import validate_coupled             # noqa: E402
# A0: psweep_g14 is FROZEN and is IMPORTED, never copied. p1_g13 B3 measured
# the cost of the alternative: a second instrument of the same SHAPE inherits
# the same defect.
from psweep_g14 import keys40, verdict, _exact              # noqa: E402

SEARCH_BUDGET = 120.0          # A2.1 G14: SEARCH budget. The wall is not capped.
THRESH = 5.0                   # p1_g13 A2.4: not shifted.
C_TASK2 = (0.15, 0.20)         # where the S2 counterexample lives.
MAPS = ('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz')
X_C = 0.80
BAND = 0.30
BANDS = (0.30, 0.60, 1.00, 2.00)   # A2.6 separator ladder, locked in advance.

_CAPS = None


def caps(gantries=(1, 2)):
    global _CAPS
    if _CAPS is None:
        _CAPS = {g: CapabilityMap.load(MAPS[g - 1]) for g in gantries}
    return _CAPS


# ================================================================= generator
def gen_real_dense(n_tasks, seed, n_mr=0, gantries=(1, 2), x_c=X_C, band=BAND,
                   p0_mode='s1', reject_at_p0=True, t_fold=0.0,
                   max_reject=200_000):
    """`sched.gen_real` with EXACTLY TWO changes (A2.1). Nothing else moves.

      1. candidates are drawn from  pool = {node : |x_node - x_c| <= band}
         instead of from all ref.nodes;
      2. a candidate doable AT p0 is rejected (`at_p0 -> reject`), tested at the
         p0 pose itself on the FULL grid, with the same predicate gen_real
         accepts with: `hand` for MR, `reach` for SR.

    p0 itself is the A2.0 knob: 's1' = gen_real's (lin = ref.lin[0], rot = 0),
    'xc' = gen_real_rotcrowded's (lin = x_c, rot = 0).

    Duplicate draws are NOT filtered, because gen_real does not filter them
    either and R4 demands the draw sequence be identical. The duplicate count is
    returned in meta so it is a reported number rather than a silent property.
    """
    cp = caps(gantries)
    ref = cp[gantries[0]]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    anchor = ref.lin[0] if p0_mode == 's1' else x_c
    p0_idx = int(np.argmin(np.hypot(poses[:, 0] - anchor, poses[:, 1])))

    pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= band)   # change 1
    if not len(pool):
        raise ValueError(f'dense band {band} at x_c {x_c} contains no nodes')

    rng = np.random.default_rng(seed)                  # A2.1 condition 3
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n_tasks - n_mr), dtype='<U2')
    chosen, rejects, rej_p0 = [], 0, 0
    while len(chosen) < n_tasks:
        cand = int(pool[rng.integers(0, len(pool))])
        want_mr = kinds[len(chosen)] == 'MR'
        ok, at_p0 = False, False
        for g in gantries:
            r, _, hd = sched._gantry_oracles(cp[g], np.array([cand]))
            m = hd[0] if want_mr else r[0].any(axis=1)
            ok |= bool(m.any())
            at_p0 |= bool(m[p0_idx])
        if reject_at_p0 and at_p0:                     # change 2
            ok = False
            rej_p0 += 1
        if ok:
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > max_reject:
                raise RuntimeError('dense generator found no feasible tasks')
    node_idx = np.array(chosen)

    reach, zone, hand = {}, {}, {}
    for g in gantries:
        reach[g], zone[g], hand[g] = sched._gantry_oracles(cp[g], node_idx)
    return sched.Instance(
        kinds, {g: poses for g in gantries}, reach, zone, hand,
        {g: p0_idx for g in gantries}, ref.nodes[node_idx], sched.DWELL, t_fold,
        f'dense(n={n_tasks},mr={n_mr},seed={seed},band={band},p0={p0_mode})',
        dict(nodes=node_idx.tolist(), rejects=rejects, rej_p0=rej_p0,
             n_poses=len(poses), pool=len(pool), band=band, p0_mode=p0_mode,
             dupes=int(n_tasks - len(set(chosen)))))


ARMS = {'DENSE': dict(p0_mode='s1'), 'DENSE-P0XC': dict(p0_mode='xc')}


def build(arm, n, seed, mr, band=BAND):
    """(instance, note). note is None on success, else why NOT BUILDABLE (R3)."""
    kw = ARMS[arm] if arm in ARMS else ARMS['DENSE']
    try:
        inst = gen_real_dense(n, seed, mr, band=band, **kw)
    except (RuntimeError, ValueError) as e:
        return None, f'generator: {e}'
    bad = r3(inst)
    return (None, bad) if bad else (inst, None)


# ===================================================================== gates
def r3(inst):
    """A2.1's four conditions, checked OUTSIDE the generator. p1_g13 B3.

    An instrument that checks itself inherits its own defect, so this re-derives
    reachability from the oracles rather than trusting the acceptance test.
    """
    p0 = int(inst.p0[inst.gantries[0]])
    if not 0 <= p0 < len(inst.poses[inst.gantries[0]]):
        return f'p0 {p0} outside the pose set'
    for t in range(inst.n):
        if inst.kind[t] == 'MR':
            ok = any(inst.hand[g][t].any() for g in inst.gantries)
        else:
            ok = any(inst.reach[g][t].any() for g in inst.gantries)
        if not ok:
            return f'task {t} ({inst.kind[t]}) unreachable at every pose'
    return None


def gates():
    """R4 + R3 + the pool census A2.1 condition 5 demands."""
    rc = 0
    print('R4 -- gen_real_dense(band=2.00, at_p0 OFF) == sched.gen_real ?')
    print('     the gate that makes "gen_real plus exactly two changes" a fact')
    bad = []
    for key, n, seed, mr in keys40():
        a = gen_real_dense(n, seed, mr, band=2.00, reject_at_p0=False)
        b = sched.gen_real(n, seed, mr, (1, 2))
        same = (a.meta['nodes'] == b.meta['nodes']
                and int(a.p0[1]) == int(b.p0[1]))
        if not same:
            bad.append((key, a.meta['nodes'], b.meta['nodes'],
                        int(a.p0[1]), int(b.p0[1])))
    if bad:
        rc = 1
        print(f'  🔴 R4 GAGAL pada {len(bad)} / 40 kunci -- TEMUAN, sesi '
              f'berhenti. Contoh: {bad[:2]}')
    else:
        print('  ✅ R4 LULUS: 40 / 40 kunci, node_idx dan p0 identik.')

    ref = caps()[1]
    print('\nPOOL CENSUS (A2.1 syarat 5) -- x_c = 0.80')
    print(f'{"band":>6s} {"pool":>6s} {"of":>6s}   fraksi peta')
    for b in BANDS:
        pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - X_C) <= b)
        print(f'{b:6.2f} {len(pool):6d} {len(ref.nodes):6d}   '
              f'{100.0*len(pool)/len(ref.nodes):5.1f} %')

    print('\nat_p0 REJECTION RATE over the whole pool (D46)')
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    for arm, kw in ARMS.items():
        anchor = ref.lin[0] if kw['p0_mode'] == 's1' else X_C
        p0 = int(np.argmin(np.hypot(poses[:, 0] - anchor, poses[:, 1])))
        pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - X_C) <= BAND)
        hit_sr = np.zeros(len(pool), bool)
        hit_mr = np.zeros(len(pool), bool)
        live_sr = np.zeros(len(pool), bool)
        live_mr = np.zeros(len(pool), bool)
        for g in (1, 2):
            r, _, hd = sched._gantry_oracles(caps()[g], pool)
            hit_sr |= r[:, p0].any(axis=1)
            hit_mr |= hd[:, p0]
            live_sr |= r.any(axis=(1, 2))
            live_mr |= hd.any(axis=1)
        print(f'  {arm:11s} p0 = pose {p0} (lin {poses[p0,0]:.2f}, '
              f'rot {np.degrees(poses[p0,1]):+.0f} deg)')
        print(f'    SR: {hit_sr.sum():4d} / {len(pool)} pool nodes doable at '
              f'p0 = {100.0*hit_sr.sum()/len(pool):5.1f} %  '
              f'-> {int((live_sr & ~hit_sr).sum())} usable')
        print(f'    MR: {hit_mr.sum():4d} / {len(pool)} pool nodes doable at '
              f'p0 = {100.0*hit_mr.sum()/len(pool):5.1f} %  '
              f'-> {int((live_mr & ~hit_mr).sum())} usable')

    print('\nR3 over the 40 keys x both arms, band = 0.30')
    for arm in ARMS:
        notes = {}
        t0 = time.time()
        for key, n, seed, mr in keys40():
            _, note = build(arm, n, seed, mr)
            if note:
                notes[key] = note
        print(f'  {arm:11s} TIDAK DAPAT DIBANGUN {len(notes)} / 40  '
              f'({time.time()-t0:.1f} s)')
        for k, v in list(notes.items())[:8]:
            print(f'      {k:12s} {v}')
        if len(notes) > 20:
            print('      🔴 > 50 % -- A2.1: TEMUAN, kepadatan tugas dan ruang '
                  'pose tidak dapat dipisahkan di sel ini')
    return rc


# ====================================================================== cell
def _path(arm, c, band=BAND):
    tag = '' if band == BAND else f'_b{band:.2f}'
    return Path(f'/tmp/g15_dense_{arm}_{c:.2f}{tag}.json')


def cell(arm, c, band=BAND, budget=SEARCH_BUDGET, only=None):
    """One (arm, c_arm) cell over the 40 S1 keys. R1/R2/R3 columns recorded."""
    q = _path(arm, c, band)
    db = json.loads(q.read_text()) if q.exists() else {}
    db.setdefault('_load', [os.getloadavg()[0], None])
    db.setdefault('rows', {})
    for key, n, seed, mr in keys40():
        if only and key not in only:
            continue
        if key in db['rows']:
            continue
        t0 = time.time()
        inst, note = build(arm, n, seed, mr, band)
        if inst is None:                                     # R3
            db['rows'][key] = dict(buildable=False, note=note)
            q.write_text(json.dumps(db, indent=1))
            print(f'{key:12s} {arm:10s} c={c:.2f}  NOT BUILDABLE: {note}',
                  flush=True)
            continue
        geom = sa.ArmGeom(inst)
        ss = sk.solve_coupled2(inst, c_clear=0.0, time_budget=budget)
        sf, ctx = af.solve_armfull(inst, c_arm=c, geom=geom, c_clear=0.0,
                                   time_budget=budget)
        errs = (validate_coupled(inst, sf.stops, sf.finish, sf.makespan)
                if sf.stops and np.isfinite(sf.makespan) else ['no schedule'])
        slow = (sa.arm_schedule_conflict(inst, geom, sf.stops, c)
                if sf.stops else None)
        db['rows'][key] = dict(
            buildable=True, nP=len(inst.poses[1]), lb=float(sf.lb),
            struct_ub=float(ss.makespan), struct_route=ss.route,
            struct_proved=bool(ss.proved),
            full_ub=float(sf.makespan), full_route=sf.route,
            full_proved=bool(sf.proved), full_nodes=int(sf.n_nodes),
            wall=time.time() - t0, gate=len(errs),
            err=[str(x) for x in errs[:2]],
            arm_slow=None if slow is None else float(slow),
            arm_calls=ctx.calls, arm_t=ctx.t_arm,
            rejects=inst.meta['rejects'], rej_p0=inst.meta['rej_p0'],
            dupes=inst.meta['dupes'])
        db['_load'][1] = os.getloadavg()[0]
        q.write_text(json.dumps(db, indent=1))
        d = sf.makespan - ss.makespan
        print(f'{key:12s} {arm:10s} c={c:.2f} b={band:.2f} '
              f'unc {sf.lb:8.3f} str {ss.makespan:9.3f}({ss.route:7s}) '
              f'full {sf.makespan:9.3f}({sf.route:7s},pr={int(sf.proved)}) '
              f'D_arm {d:+8.4f}{"  🔴 POSITIF" if d > 1e-9 else ""} '
              f'gate={len(errs)}{"" if slow is None else " ARMVIOL"} '
              f'{time.time()-t0:6.1f}s', flush=True)
    print(f'CELL {arm} c={c:.2f} b={band:.2f} DONE  load {db["_load"][0]:.2f} '
          f'-> {db["_load"][1]}', flush=True)
    return 0


# ===================================================================== probe
def probe(cs=(0.20,), keys=('n6_s0_mr1', 'n6_s4_mr0'), arm='DENSE'):
    """A2.5: the cost of the cell, measured BEFORE the 40 instances.

    Eighth session with the same trap (p1_g8 B4 ... p1_g14 B3). This is the
    EXPENSIVE end of G14's ladder -- |P| = 2376 full -- and G14 B3 measured
    that cost is NOT monotone in |P|, so the sign is not guessable from there
    either.
    """
    print('A2.5 COST PROBE -- decision rule locked in A2.5 before this ran\n')
    for c in cs:
        for key in keys:
            n, seed, mr = (int(key.split('_')[0][1:]),
                           int(key.split('_')[1][1:]),
                           int(key.split('_')[2][2:]))
            inst, note = build(arm, n, seed, mr)
            if inst is None:
                print(f'  {key} {arm}: NOT BUILDABLE ({note})', flush=True)
                continue
            geom = sa.ArmGeom(inst)
            t0 = time.time()
            sf, ctx = af.solve_armfull(inst, c_arm=c, geom=geom, c_clear=0.0,
                                       time_budget=SEARCH_BUDGET)
            w = time.time() - t0
            print(f'  {key:12s} {arm:10s} c={c:.2f} |P|={len(inst.poses[1])} '
                  f'wall {w:8.2f} s  makespan {sf.makespan:9.4f} ({sf.route}, '
                  f'pr={int(sf.proved)}, nodes {sf.n_nodes})  '
                  f'arm gate {ctx.calls} calls {ctx.t_arm:.1f} s '
                  f'= {100*ctx.t_arm/max(w,1e-9):.0f} %', flush=True)
    return 0


# ==================================================================== report
def _s1_base():
    """The paired-KEY control (A2.3): G13's S1 rows, read as artefacts.

    G13's schema nests the per-c_arm columns under row['arm'][f'{c:.2f}'] with
    `lb` on the row itself, so it is flattened here into the same shape this
    file writes. Read from the artefact, never recomputed (A0).
    """
    out = {}
    for c in C_TASK2:
        p = Path(f'/tmp/g13_eval_s1_{c:.2f}.json')
        if not p.exists():
            continue
        rows = json.loads(p.read_text()).get('s1', {})
        ck = f'{c:.2f}'
        out[c] = {k: dict(r['arm'][ck], lb=r['lb'])
                  for k, r in rows.items() if ck in r.get('arm', {})}
    return out


def report(arms=None, bands=(BAND,)):
    base = _s1_base()
    arms = arms or list(ARMS)
    print('TASK 2 -- task crowding. A2.3 criteria, copied verbatim from G14 '
          'A2.2:\n  PRICE APPEARS := >=1 instance with Delta_arm > 0 and BOTH '
          'sides proved.\n  Control is PAIRED-KEY (G13 S1 rows), not '
          'same-instance -- A2.3 says so in advance.\n  A2.2: the 10 % lb bar '
          'does NOT apply here; the tasks differ BY CONSTRUCTION.\n')
    hdr = (f'{"arm":11s} {"c":>5s} {"b":>5s} {"built":>6s} {"pv":>4s} '
           f'{"POS":>4s} {"D_arm mean%":>22s} {"lb vs S1":>9s} '
           f'{"wall mean/max":>15s}')
    print(hdr)
    print('-' * len(hdr))
    for arm in arms:
        for b in bands:
            for c in C_TASK2:
                q = _path(arm, c, b)
                if not q.exists():
                    print(f'{arm:11s} {c:5.2f} {b:5.2f}  TIDAK DIUKUR')
                    continue
                db = json.loads(q.read_text())
                rows = db['rows']
                nb = [k for k, r in rows.items() if not r['buildable']]
                ok = {k: r for k, r in rows.items() if r['buildable']}
                if not ok:
                    print(f'{arm:11s} {c:5.2f} {b:5.2f}  0 buildable of '
                          f'{len(rows)}')
                    continue
                s1 = base.get(c, {})
                dr = [100.0 * (r['lb'] - s1[k]['lb']) / s1[k]['lb']
                      for k, r in ok.items()
                      if k in s1 and s1[k].get('lb')]
                drift = float(np.median(dr)) if dr else float('nan')
                st = [(r['lb'], r['struct_ub'],
                       _exact(r['struct_route'], r['struct_proved'],
                              r['struct_ub'], r['lb'])) for r in ok.values()]
                fu = [(r['lb'], r['full_ub'],
                       _exact(r['full_route'], r['full_proved'], r['full_ub'],
                              r['lb'])) for r in ok.values()
                      if np.isfinite(r['full_ub'])]
                _, slo, shi = verdict(st)
                vf, flo, fhi = ('-', float('nan'), float('nan')) if not fu \
                    else verdict(fu)
                pos = [(k, r['full_ub'] - r['struct_ub'])
                       for k, r in ok.items()
                       if np.isfinite(r['full_ub'])
                       and r['full_ub'] - r['struct_ub'] > 1e-9
                       and _exact(r['full_route'], r['full_proved'],
                                  r['full_ub'], r['lb'])
                       and _exact(r['struct_route'], r['struct_proved'],
                                  r['struct_ub'], r['lb'])]
                pv = sum(1 for _, _, e in fu if e)
                w = [r['wall'] for r in ok.values()]
                print(f'{arm:11s} {c:5.2f} {b:5.2f} {len(ok):6d} {pv:4d} '
                      f'{len(pos):4d} [{flo - slo:+9.4f},{fhi - shi:+9.4f}] '
                      f'{drift:+8.2f}% {np.mean(w):7.1f}/{max(w):6.1f}s')
                none = [k for k, r in ok.items()
                        if not np.isfinite(r['full_ub'])]
                bad = [k for k, r in ok.items()
                       if np.isfinite(r['full_ub'])
                       and (r['gate'] or r['arm_slow'] is not None)]
                if nb:
                    print(f'{"":11s} R3 NOT BUILDABLE {len(nb)}: {nb[:6]}'
                          + (f' +{len(nb)-6}' if len(nb) > 6 else ''))
                if none:
                    ws = [ok[k]['wall'] for k in none]
                    print(f'{"":11s} TIDAK ADA JADWAL DITEMUKAN (pencarian '
                          f'{SEARCH_BUDGET:.0f} s; wall {np.mean(ws):.1f}/'
                          f'{max(ws):.1f} s) {len(none)}: {none[:6]}')
                    # paired-key control: what G13 S1 did on the same keys
                    s1none = [k for k in none
                              if k in s1 and not np.isfinite(
                                  s1[k].get('full_ub', float('inf')))]
                    print(f'{"":11s}   dari kunci itu, S1 juga tanpa jadwal: '
                          f'{len(s1none)}')
                if bad:
                    print(f'{"":11s} 🔴 GATE FAILURES {len(bad)}: {bad[:6]}')
                for k, d in pos:
                    ctrl = ('S1 kunci sama: Delta_arm = '
                            f'{s1[k]["full_ub"] - s1[k]["struct_ub"]:+.4f}'
                            if k in s1 and np.isfinite(
                                s1[k].get('full_ub', float('inf')))
                            else 'S1 kunci sama: TIDAK ADA JADWAL')
                    print(f'{"":11s} 🔴 Delta_arm POSITIF  {k}  {d:+.4f} s '
                          f'({100.0*d/ok[k]["struct_ub"]:+.2f} %) | {ctrl}')
    return 0


# ====================================================== why "no schedule"?
def park(key='n6_s0_mr1', c=0.20, arm='DENSE'):
    """A2.5 demands the probe's NO SCHEDULE be explained before the cells run.

    G14 B4 profiled this category on S1 and measured 19008 rejections / 0
    acceptances, 52 % of them at the STRUCTURAL predicate in the parking phase.
    Whether the dense cell fails the same way is a measurement, not an
    inference -- prior D29 (p1_g14 B8 reading 3): every real finding came from
    a test that was run, zero from re-reading code.

    `_park_profile` is imported from diag_g14 rather than re-written: p1_g13 B3
    measured that a second instrument of the same SHAPE inherits the same defect.
    """
    from diag_g14 import _park_profile, STAGES        # noqa: E402
    n, seed, mr = (int(key.split('_')[0][1:]), int(key.split('_')[1][1:]),
                   int(key.split('_')[2][2:]))
    inst, note = build(arm, n, seed, mr)
    if inst is None:
        print(f'{key}: NOT BUILDABLE ({note})')
        return 1
    geom = sa.ArmGeom(inst)
    ctx = af.ArmCtx(inst, geom, c, c_clear=0.0)
    sol0 = sched.solve_exact(inst)
    print(f'PARK-POSE REJECTION PROFILE -- {arm} {key} c_arm={c} '
          f'|P|={len(inst.poses[1])}', flush=True)
    t0 = time.time()
    agg = dict.fromkeys(STAGES, 0)
    for lead, prk, cnt, first, ntried in _park_profile(inst, ctx, sol0, 0.0):
        for k in STAGES:
            agg[k] += cnt[k]
        print(f'  lead {lead} park {prk}: '
              + '  '.join(f'{k.split()[0]} {cnt[k]:5d}' for k in STAGES)
              + f'   ({ntried} poses, {time.time()-t0:.0f} s)', flush=True)
        if first is not None:
            print(f'      🔴 the frozen loop WOULD have accepted pose '
                  f'{first[0]} -> makespan {first[1]:.4f}')
    tot = sum(agg.values())
    if not tot:
        print('nothing profiled')
        return 0
    parkph = agg['S1 struct/park'] + agg['S2 arm/park']
    tail = agg['S3 struct/tail'] + agg['S4 arm/tail']
    for k in STAGES:
        print(f'  {k:16s} {agg[k]:6d}  {100.0*agg[k]/tot:5.1f} %')
    print(f'  PARKIR {parkph} ({100.0*parkph/tot:.1f} %)   '
          f'EKOR SERIAL {tail} ({100.0*tail/tot:.1f} %)   total {tot}')
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['report']
    if a[0] == 'gates':
        return gates()
    if a[0] == 'park':
        return park(*(a[1:2] or []), **({'c': float(a[2])} if len(a) > 2
                                        else {}))
    if a[0] == 'probe':
        return probe(arm=a[1] if len(a) > 1 else 'DENSE')
    if a[0] == 'cell':
        return cell(a[1], float(a[2]),
                    float(a[3]) if len(a) > 3 else BAND)
    if a[0] == 'report':
        return report(a[1:] or None)
    raise SystemExit(f'unknown: {a[0]}')


if __name__ == '__main__':
    raise SystemExit(main())
