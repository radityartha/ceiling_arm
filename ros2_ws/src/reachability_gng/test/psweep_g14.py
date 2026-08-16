#!/usr/bin/env python3
"""G14 task 2: WHY S2 AND NOT S1. docs/p1_g14_reach4.md A2.2.

p1_g13 B5.4 found the first and only proved positive arm price in P1 --
n6_s0_mr1 on S2, +0.5000 s = +5.40 %, at c_arm 0.15 AND 0.20 -- against 147
proofs of Delta_arm = 0 on S1 across all five c_arm. The prompt's hypothesis is
that the price appears when the ESCAPE SPACE is restricted rather than when the
clearance is raised.

A2.2 names the confound before testing it: S2 differs from S1 in THREE ways,
and the hypothesis mentions one.

  (1) CARDINALITY   |P| = 507 vs 2376
  (2) GEOMETRY      the 507 are a BAND (|sin rot| >= 0.525 and |lin - 0.80| <=
                    0.30) sitting where the arms sweep across the 0.72 m gantry
                    separation -- not a random sample
  (3) TASK CROWDING every S2 task lies in x_c +- 0.30 and NONE is doable at p0,
                    so both gantries are forced into the same region

So there are two arms, each shrinking S1's pose set and changing NOTHING else
(same tasks, same map, same p0, same 40 keys):

  CARD-<m>   |P| shrunk to m by a seeded random subsample of the full grid
  GEO-<tag>  |P| replaced by S2's own mask (GEO-S2) or by its rotation half
             alone (GEO-ROT)

Each GEO arm has a CARD twin of identical |P|, which is the whole design: the
twins differ only in WHICH poses survive, so a difference between them is
geometry and a difference along the CARD ladder is cardinality. (3) stays
uncontrolled -- both arms carry S1's tasks -- and A2.2 requires that to be said
out loud rather than discovered later.

A2.2's four control conditions are enforced per instance per rung, and an
instance that fails any of them is reported NOT BUILDABLE by name (R3), never
dropped:
   p0 retained; every task still reachable; seeded deterministically;
   lb drift reported (a rung whose median lb moves > 10 % is NOT THE SAME
   INSTANCE and its positives may not be quoted).

Run:  python3 test/psweep_g14.py probe
      python3 test/psweep_g14.py cell CARD-507 0.15
      python3 test/psweep_g14.py report
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

SEARCH_BUDGET = 120.0          # A2.1: SEARCH budget. The wall is not capped.
THRESH = 5.0                   # A2.4 p1_g13: not shifted.
LB_DRIFT = 10.0                # A2.2 condition 4, locked.
C_TASK2 = (0.15, 0.20)         # A2.2: only where the S2 counterexample lives.
MAPS = ('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz')

# A2.2 locked ladder, plus the twin of GEO-ROT (see B: PERTENTANGAN 1).
RUNGS = (1519, 1188, 594, 507, 297, 148)

_REF = None


def ref():
    global _REF
    if _REF is None:
        _REF = CapabilityMap.load(MAPS[0])
    return _REF


def _grid():
    r = ref()
    return np.stack(np.meshgrid(r.lin, r.rot, indexing='ij'),
                    -1).reshape(-1, 2)


def s2_mask():
    """S2's own pose mask, read off gen_real_rotcrowded's definition."""
    P = _grid()
    return np.flatnonzero((np.abs(np.sin(P[:, 1])) >= sa.SIN31)
                          & (np.abs(P[:, 0] - 0.80) <= 0.30))


def rot_mask():
    """S2's ROTATION half alone: |sin rot| >= 0.525 at every rail position.

    The half of S2's restriction that the postulated mechanism actually names
    -- near +-90 deg the arms sweep across the gantry separation -- with the
    rail band, which is a TASK-POOL restriction in disguise, left out.
    """
    return np.flatnonzero(np.abs(np.sin(_grid()[:, 1])) >= sa.SIN31)


# ==================================================================== shrink
def keys40():
    return [(f'n{n}_s{s}_mr{m}', n, s, m)
            for n in (4, 6) for s in range(10) for m in (0, 1)]


def _reachable(inst, keep):
    """A2.2 condition 2: every task still doable somewhere in `keep`."""
    for t in range(inst.n):
        if inst.kind[t] == 'MR':
            ok = any(inst.hand[g][t, keep].any() for g in inst.gantries)
        else:
            ok = any(inst.reach[g][t, keep].any() for g in inst.gantries)
        if not ok:
            return False
    return True


def shrink(inst, keep, label):
    """Same instance on a sub-set of poses. Only axis 1 of the oracles moves."""
    keep = np.sort(np.asarray(keep, int))
    p0 = int(inst.p0[inst.gantries[0]])
    if p0 not in keep:                      # A2.2 condition 1
        keep = np.sort(np.append(keep, p0))
    p0i = int(np.searchsorted(keep, p0))
    return sched.Instance(
        inst.kind, {g: inst.poses[g][keep] for g in inst.gantries},
        {g: inst.reach[g][:, keep] for g in inst.gantries},
        {g: inst.zone[g][:, keep] for g in inst.gantries},
        {g: inst.hand[g][:, keep] for g in inst.gantries},
        {g: p0i for g in inst.gantries}, inst.xyz, inst.dwell, inst.t_fold,
        f'{inst.label}|{label}',
        dict(inst.meta, shrink=label, n_poses=len(keep)))


def build(arm, key, n, seed, mr, tries=200):
    """(instance, note). note is None on success, else why it is NOT BUILDABLE."""
    full = sched.gen_real(n, seed, mr, (1, 2))
    if arm.startswith('CARD-'):
        m = int(arm.split('-')[1])
        p0 = int(full.p0[1])
        pool = np.setdiff1d(np.arange(len(full.poses[1])), [p0])
        rng = np.random.default_rng(70000 + 100 * seed + m + 7 * n + 3 * mr)
        for _ in range(tries):                        # A2.2 condition 3
            keep = np.sort(np.append(rng.choice(pool, m - 1, replace=False),
                                     p0))
            if _reachable(full, keep):
                return shrink(full, keep, arm), None
        return None, f'no feasible subsample of size {m} in {tries} tries'
    mask = s2_mask() if arm == 'GEO-S2' else rot_mask()
    keep = np.union1d(mask, [int(full.p0[1])])
    if not _reachable(full, keep):
        return None, 'task unreachable inside the mask'
    return shrink(full, keep, arm), None


# ====================================================================== cell
def _path(arm, c):
    return Path(f'/tmp/g14_psweep_{arm}_{c:.2f}.json')


def cell(arm, c, budget=SEARCH_BUDGET, only=None):
    """One (arm, c_arm) cell over the 40 S1 keys. R1/R2/R3 columns recorded."""
    q = _path(arm, c)
    db = json.loads(q.read_text()) if q.exists() else {}
    db.setdefault('_load', [os.getloadavg()[0], None])
    db.setdefault('rows', {})
    for key, n, seed, mr in keys40():
        if only and key not in only:
            continue
        if key in db['rows']:
            continue
        t0 = time.time()
        inst, note = build(arm, key, n, seed, mr)
        if inst is None:                              # R3
            db['rows'][key] = dict(buildable=False, note=note)
            q.write_text(json.dumps(db, indent=1))
            print(f'{key:12s} {arm:9s} c={c:.2f}  NOT BUILDABLE: {note}',
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
            wall=time.time() - t0, gate=len(errs), err=[str(x) for x in errs[:2]],
            arm_slow=None if slow is None else float(slow),
            arm_calls=ctx.calls, arm_t=ctx.t_arm)
        db['_load'][1] = os.getloadavg()[0]
        q.write_text(json.dumps(db, indent=1))
        d = sf.makespan - ss.makespan
        print(f'{key:12s} {arm:9s} c={c:.2f} |P|={len(inst.poses[1]):4d} '
              f'unc {sf.lb:8.3f} str {ss.makespan:9.3f}({ss.route:7s}) '
              f'full {sf.makespan:9.3f}({sf.route:7s},pr={int(sf.proved)}) '
              f'D_arm {d:+8.4f}{"  🔴 POSITIF" if d > 1e-9 else ""} '
              f'gate={len(errs)}{"" if slow is None else " ARMVIOL"} '
              f'{time.time()-t0:6.1f}s', flush=True)
    print(f'CELL {arm} c={c:.2f} DONE  load {db["_load"][0]:.2f} -> '
          f'{db["_load"][1]}', flush=True)
    return 0


# ===================================================================== probe
def probe(cs=(0.20,), rungs=(148, 1188), keys=('n6_s0_mr1', 'n6_s4_mr0')):
    """A2.2: the cost of the ladder, measured BEFORE the 40 instances.

    Seventh session with the same trap (p1_g8 B4 ... p1_g13 B2), and the two
    mechanisms pull against each other, so the sign is not guessable: a small
    |P| makes arm_serial_ub cheap (few poses to sweep) but makes the escape
    space narrow, so the search has to actually run.
    """
    print('A2.2 COST PROBE -- decision rule locked in A2.2 before this ran\n')
    for c in cs:
        for m in rungs:
            for key in keys:
                n, seed, mr = (int(key.split('_')[0][1:]),
                               int(key.split('_')[1][1:]),
                               int(key.split('_')[2][2:]))
                inst, note = build(f'CARD-{m}', key, n, seed, mr)
                if inst is None:
                    print(f'  {key} CARD-{m}: NOT BUILDABLE ({note})')
                    continue
                geom = sa.ArmGeom(inst)
                t0 = time.time()
                sf, ctx = af.solve_armfull(inst, c_arm=c, geom=geom,
                                           c_clear=0.0,
                                           time_budget=SEARCH_BUDGET)
                w = time.time() - t0
                print(f'  {key:12s} CARD-{m:<5d} c={c:.2f}  wall {w:8.2f} s  '
                      f'makespan {sf.makespan:9.4f} ({sf.route}, '
                      f'pr={int(sf.proved)}, nodes {sf.n_nodes})  '
                      f'arm gate {ctx.calls} calls {ctx.t_arm:.1f} s '
                      f'= {100*ctx.t_arm/max(w,1e-9):.0f} %', flush=True)
    return 0


# ==================================================================== report
def _exact(route, proved, ub, lb):
    return route in ('lemma4', 'single', 'dive-lb') or ub <= lb + 1e-9 or proved


def verdict(pairs):
    lo, hi = [], []
    for lb, ub, ex in pairs:
        d = 100.0 * (ub - lb) / lb
        lo.append(d if ex else 0.0)
        hi.append(d)
    m_lo, m_hi = float(np.mean(lo)), float(np.mean(hi))
    exp = (m_lo >= THRESH, m_hi >= THRESH)
    v = ('TIDAK DAPAT DITENTUKAN' if exp[0] != exp[1] else
         'MAHAL' if exp[0] else 'BUKAN MAHAL (< 5 %)')
    if v.startswith('BUKAN') and m_hi <= 1e-9:
        v = 'GRATIS'
    return v, m_lo, m_hi


def _lb_full():
    """lb of the UNSHRUNK instance, so A2.2 condition 4 can be measured."""
    out = {}
    for key, n, seed, mr in keys40():
        out[key] = float(sched.solve_exact(sched.gen_real(n, seed, mr,
                                                          (1, 2))).makespan)
    return out


def report(arms=None):
    base = _lb_full()
    arms = arms or [f'CARD-{m}' for m in RUNGS] + ['GEO-S2', 'GEO-ROT']
    print('TASK 2 -- the |P| ladder. A2.2 criteria, locked before it was '
          'built:\n  PRICE APPEARS := >=1 instance with Delta_arm > 0 and BOTH '
          'sides proved.\n  MANUSCRIPT SENTENCE := positives rise monotonically '
          'over >=3 rungs.\n  A lone positive gets p1_g13 B5.4\'s '
          'one-instance qualification.\n')
    hdr = (f'{"arm":10s} {"c":>5s} {"|P|":>5s} {"built":>6s} {"pv":>4s} '
           f'{"POS":>4s} {"D_arm mean%":>22s} {"lb drift":>9s} '
           f'{"wall mean/max":>15s}')
    print(hdr)
    print('-' * len(hdr))
    for arm in arms:
        for c in C_TASK2:
            q = _path(arm, c)
            if not q.exists():
                print(f'{arm:10s} {c:5.2f}  TIDAK DIUKUR')
                continue
            db = json.loads(q.read_text())
            rows = db['rows']
            nb = [k for k, r in rows.items() if not r['buildable']]
            ok = {k: r for k, r in rows.items() if r['buildable']}
            if not ok:
                print(f'{arm:10s} {c:5.2f}  0 buildable of {len(rows)}')
                continue
            drift = np.median([100.0 * (r['lb'] - base[k]) / base[k]
                               for k, r in ok.items()])
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
            pos = [(k, r['full_ub'] - r['struct_ub']) for k, r in ok.items()
                   if np.isfinite(r['full_ub'])
                   and r['full_ub'] - r['struct_ub'] > 1e-9
                   and _exact(r['full_route'], r['full_proved'], r['full_ub'],
                              r['lb'])
                   and _exact(r['struct_route'], r['struct_proved'],
                              r['struct_ub'], r['lb'])]
            pv = sum(1 for _, _, e in fu if e)
            w = [r['wall'] for r in ok.values()]
            nP = next(iter(ok.values()))['nP']
            print(f'{arm:10s} {c:5.2f} {nP:5d} {len(ok):6d} {pv:4d} '
                  f'{len(pos):4d} [{flo - slo:+9.4f},{fhi - shi:+9.4f}] '
                  f'{drift:+8.2f}% {np.mean(w):7.1f}/{max(w):6.1f}s')
            none = [k for k, r in ok.items() if not np.isfinite(r['full_ub'])]
            # A NO SCHEDULE FOUND row carries gate == 1 ('no schedule'), which
            # is a budget statement and not a soundness failure -- counting it
            # as one would report R1 breaking where nothing broke.
            bad = [k for k, r in ok.items()
                   if np.isfinite(r['full_ub'])
                   and (r['gate'] or r['arm_slow'] is not None)]
            if nb:
                print(f'{"":10s} R3 NOT BUILDABLE {len(nb)}: {nb[:6]}'
                      + (f' +{len(nb)-6}' if len(nb) > 6 else ''))
            if none:
                print(f'{"":10s} NO SCHEDULE FOUND (pencarian '
                      f'{SEARCH_BUDGET:.0f} s) {len(none)}: {none[:6]}')
            if bad:
                print(f'{"":10s} 🔴 GATE FAILURES {len(bad)}: {bad[:6]}')
            if abs(drift) > LB_DRIFT:
                print(f'{"":10s} 🔺 lb drift {drift:+.2f} % > {LB_DRIFT} % '
                      f'-- A2.2 cond. 4: NOT THE SAME INSTANCE, positives '
                      f'here may NOT be quoted as the answer')
            for k, d in pos:
                print(f'{"":10s} 🔴 Delta_arm POSITIF  {k}  {d:+.4f} s '
                      f'({100.0*d/ok[k]["struct_ub"]:+.2f} %)')
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['report']
    if a[0] == 'probe':
        return probe()
    if a[0] == 'cell':
        return cell(a[1], float(a[2]))
    if a[0] == 'report':
        return report(a[1:] or None)
    raise SystemExit(f'unknown: {a[0]}')


if __name__ == '__main__':
    raise SystemExit(main())
