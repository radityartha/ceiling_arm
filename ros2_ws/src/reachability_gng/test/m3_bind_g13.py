#!/usr/bin/env python3
"""G13 A2.3: the M3 oracle debt. Can an instance be built where ARM_BLOCK
actually RAISES the brute-force optimum?

G12 answered "no" on 18 instances whose (pose, task) blocking fraction was
0.12-0.75 (p1_g12 B8), and reported the failure as the result. This session
does not lower the bar; it turns the two levers G12 never turned, both named in
A2.3 BEFORE anything was built:

  lever 3  tasks FORCED per gantry -- each task reachable by exactly one
           gantry, so work cannot migrate to the other side
  lever 4  BALANCED load -- equal task counts and comparable dwell, so the
           makespan is set by CONCURRENCY, not by one gantry's serial chain

The hypothesis those levers test is stated in A2.3 and is the reason they are
here: on the small instances G12 used, there is a SECOND escape route it never
closed -- WAITING. If one gantry's serial chain sets the makespan, the other
may be serialised entirely for free, and ARM_BLOCK is then priced at zero by
the instance's shape rather than by the cell's geometry.

WHAT IS NOT NEGOTIABLE (A2.3, so that "cannot be built" stays distinguishable
from "the criterion moved"):

    BINDING(inst, c) <=> brute_arm(inst, c) > brute_arm(inst, +inf) + 1e-9

`brute_arm`, `gen_small_armreal` and `arm_binding_fraction` are IMPORTED from
the frozen test/verify_sched_armfull.py, not copied, so "binding" is decided by
the identical enumerator that reported 0 of 18.

Run:  python3 test/m3_bind_g13.py scan | classes | m3
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
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng.capability import CapabilityMap       # noqa: E402
from verify_sched_armfull import (brute_arm, arm_binding_fraction,  # noqa: E402
                                  gen_small_armreal, MAPS)

C_SWEEP = (0.05, 0.10, 0.15, 0.20)
_CAPS = {}


def caps():
    if not _CAPS:
        for g in (1, 2):
            _CAPS[g] = CapabilityMap.load(MAPS[g - 1])
    return _CAPS


# ===================================================================== levers
def gen_forced(n, n_pose, seed, x_c=0.80, band=0.06, rot_tol=12.0,
               dwell_mul=1.0, node_band=0.30):
    """LEVERS 3 + 4: half the tasks reachable ONLY by gantry 1, half ONLY by 2.

    Same pose selection as gen_small_armreal (the arm-adversarial band: near
    |rot| = 90 deg, where the arms sweep across the 0.72 m gantry separation),
    but the TASKS are partitioned so that:

      * no task can be moved to the other gantry (lever 3), which removes the
        "reassign it" escape, and
      * both gantries carry the same count (lever 4), which removes the
        "serialise the idle one for free" escape.

    Returns None if the map cannot supply a balanced forced split, which is
    itself reportable -- it would mean the levers are not available on this
    cell, not that they were not tried.
    """
    C = caps()
    ref = C[1]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    p0 = int(np.argmin(np.hypot(poses[:, 0] - x_c, poses[:, 1])))
    hot = np.flatnonzero((np.abs(poses[:, 0] - x_c) <= band)
                         & (np.abs(np.abs(poses[:, 1]) - np.pi / 2)
                            <= np.deg2rad(rot_tol)))
    if len(hot) < n_pose - 1:
        return None
    rng = np.random.default_rng(9000 + seed)
    keep = np.sort(np.append(rng.choice(hot, n_pose - 1, replace=False), p0))
    p0_idx = int(np.searchsorted(keep, p0))

    pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= node_band)
    r = {g: sched._gantry_oracles(C[g], pool)[0][:, keep] for g in (1, 2)}
    ok = {g: r[g].any(axis=(1, 2)) for g in (1, 2)}          # reachable at all
    at_p0 = {g: r[g][:, p0_idx].any(axis=1) for g in (1, 2)}
    only = {1: np.flatnonzero(ok[1] & ~ok[2] & ~at_p0[1]),
            2: np.flatnonzero(ok[2] & ~ok[1] & ~at_p0[2])}
    half = n // 2
    if len(only[1]) < half or len(only[2]) < n - half:
        return None
    pick = np.concatenate([rng.choice(only[1], half, replace=False),
                           rng.choice(only[2], n - half, replace=False)])
    node_idx = pool[pick]
    reach, zone, hand = {}, {}, {}
    for g in (1, 2):
        a, b, c = sched._gantry_oracles(C[g], node_idx)
        reach[g], zone[g], hand[g] = a[:, keep], b[:, keep], c[:, keep]
    return sched.Instance(
        np.array(['SR'] * n, dtype='<U2'), {g: poses[keep] for g in (1, 2)},
        reach, zone, hand, {g: p0_idx for g in (1, 2)}, ref.nodes[node_idx],
        sched.DWELL * dwell_mul, 0.0,
        f'forced(n={n},P={n_pose},seed={seed},dw={dwell_mul})',
        dict(nodes=node_idx.tolist(), pose_keep=keep.tolist(),
             forced=True, split=[half, n - half]))


# ================================================== the diagnostic enumerator
def enum_opt(inst, geom, c_arm, c_clear=0.0, step=0.25, max_states=400_000):
    """(best makespan, HOW MANY complete schedules attain it).

    A SECOND instrument, written separately from `brute_arm` on purpose and
    labelled as such. `brute_arm` returns only the optimum, which is enough to
    decide BINDING but cannot separate the two ways a constraint can be priced
    at zero -- and A2.3 requires that separation to be reported whatever the
    binding column says:

      vacuous   the constraint removes NO optimal schedule; it never touches
                the optimum at all
      rerouted  it removes SOME optimal schedules but others of the same
                makespan survive -- it BINDS, and its price is zero
      binding   it removes ALL of them, so the optimum rises

    Its makespan must equal `brute_arm`'s on every instance; that agreement is
    reported as a cross-check the frozen enumerator never had.
    """
    gs = inst.gantries
    full = (1 << inst.n) - 1
    P = {g: len(inst.poses[g]) for g in gs}
    best, cnt, seen = np.inf, 0, 0

    def rec(R, stops, tmin):
        nonlocal best, cnt, seen
        seen += 1
        if seen > max_states:
            return
        if R == 0:
            st = {g: list(stops[g]) for g in gs}
            if any(st[g] for g in gs):
                if sc.schedule_conflict(inst, st, c_clear) is not None:
                    return
                if (np.isfinite(c_arm) and sa.arm_schedule_conflict(
                        inst, geom, st, c_arm) is not None):
                    return
            m = max((s['start'] + s['dur'] for g in gs for s in st[g]),
                    default=0.0)
            if m < best - 1e-9:
                best, cnt = m, 1
            elif m <= best + 1e-9:
                cnt += 1
            return
        for g in gs:
            U = R
            while U:
                for p in range(P[g]):
                    dur, assign = sched.stop_duration(inst, g, U, p)
                    if not np.isfinite(dur):
                        continue          # G13 B3 -- see verify_sched_armfull
                    cur = (stops[g][-1]['pose'] if stops[g]
                           else int(inst.p0[g]))
                    T = sc.leg_duration(tuple(inst.poses[g][cur]),
                                        tuple(inst.poses[g][p]))
                    t_ready = (stops[g][-1]['start'] + stops[g][-1]['dur']
                               if stops[g] else 0.0)
                    for k in range(6):
                        dep = t_ready + k * step
                        if dep > tmin + 6 * step:
                            break
                        stops[g].append(dict(pose=p, depart=dep, start=dep + T,
                                             dur=dur, tasks=U, assign=assign))
                        rec(R ^ U, stops, max(tmin, dep + T + dur))
                        stops[g].pop()
                U = (U - 1) & R
        return

    rec(full, {g: [] for g in gs}, 0.0)
    return best, cnt


def classify(inst, geom, c_arm):
    """The three-way split A2.3 locked, plus the BINDING decision made by the
    FROZEN enumerator. Returns a dict, or None if nothing is feasible."""
    m_free = brute_arm(inst, geom, np.inf, 0.0)
    m_arm = brute_arm(inst, geom, c_arm, 0.0)
    if not np.isfinite(m_free):
        return None
    e_free, k_free = enum_opt(inst, geom, np.inf)
    e_arm, k_arm = enum_opt(inst, geom, c_arm)
    binding = np.isfinite(m_arm) and m_arm > m_free + 1e-9
    if binding or not np.isfinite(m_arm):
        kind = 'binding' if np.isfinite(m_arm) else 'infeasible'
    elif k_arm >= k_free:
        kind = 'vacuous'
    else:
        kind = 'rerouted'
    return dict(m_free=float(m_free), m_arm=float(m_arm), kind=kind,
                k_free=int(k_free), k_arm=int(k_arm), binding=bool(binding),
                agree=bool(abs(e_free - m_free) <= 1e-9
                           and (not np.isfinite(m_arm)
                                or abs(e_arm - m_arm) <= 1e-9)))


# ============================================================ lever 3 is real?
def scan(node_band=0.30, x_c=0.80, band=0.06, rot_tol=12.0, n_pose=4,
         seeds=6):
    """Is lever 3 even AVAILABLE on this map? Measured before it is used.

    p1_g11 B3 measured that reachability on this cell is very nearly
    ROTATION-invariant, which is what killed the S2 probe's original design. The
    analogous question for lever 3 is whether it is nearly GANTRY-invariant --
    if every task is reachable from both gantries, tasks cannot be forced and
    the lever does not exist. That is measured, not assumed.
    """
    C = caps()
    ref = C[1]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    p0 = int(np.argmin(np.hypot(poses[:, 0] - x_c, poses[:, 1])))
    hot = np.flatnonzero((np.abs(poses[:, 0] - x_c) <= band)
                         & (np.abs(np.abs(poses[:, 1]) - np.pi / 2)
                            <= np.deg2rad(rot_tol)))
    print(f'hot poses in the arm-adversarial band: {len(hot)} of {len(poses)}')
    rng = np.random.default_rng(9000)
    for s in range(seeds):
        keep = np.sort(np.append(rng.choice(hot, n_pose - 1, replace=False),
                                 p0))
        p0_idx = int(np.searchsorted(keep, p0))
        pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= node_band)
        r = {g: sched._gantry_oracles(C[g], pool)[0][:, keep] for g in (1, 2)}
        ok = {g: r[g].any(axis=(1, 2)) for g in (1, 2)}
        ap = {g: r[g][:, p0_idx].any(axis=1) for g in (1, 2)}
        o1 = int((ok[1] & ~ok[2] & ~ap[1]).sum())
        o2 = int((ok[2] & ~ok[1] & ~ap[2]).sum())
        both = int((ok[1] & ok[2]).sum())
        print(f'  seed {s}: pool {len(pool):4d}  both {both:4d}  '
              f'ONLY g1 {o1:4d}  ONLY g2 {o2:4d}  neither '
              f'{int((~ok[1] & ~ok[2]).sum()):4d}')
    return 0


# ==================================================================== the run
CLASSES = [
    # (label, generator, kwargs) -- A2.3 lantai usaha: >= 5 distinct classes
    ('G12-baseline', 'armreal', dict(n=2, n_pose=3, n_mr=0)),
    ('G12-baseline-P4', 'armreal', dict(n=3, n_pose=4, n_mr=0)),
    ('forced-n2P3', 'forced', dict(n=2, n_pose=3)),
    ('forced-n2P4', 'forced', dict(n=2, n_pose=4)),
    ('forced-n4P3', 'forced', dict(n=4, n_pose=3)),
    ('forced-n4P4', 'forced', dict(n=4, n_pose=4)),
    ('forced-n2P3-dw2', 'forced', dict(n=2, n_pose=3, dwell_mul=2.0)),
    ('forced-n4P3-dw2', 'forced', dict(n=4, n_pose=3, dwell_mul=2.0)),
    ('forced-n2P2', 'forced', dict(n=2, n_pose=2)),
]


def build(kind, seed, kw):
    if kind == 'armreal':
        return gen_small_armreal(kw['n'], kw['n_pose'], seed,
                                 n_mr=kw.get('n_mr', 0))
    return gen_forced(kw['n'], kw['n_pose'], seed,
                      dwell_mul=kw.get('dwell_mul', 1.0))


def m3(seeds=14, cs=C_SWEEP, need=0.02, wall_cap=1800.0):
    t_start = time.time()
    cand = tried = 0
    tab = {}
    rows = []
    for label, kind, kw in CLASSES:
        tab[label] = dict(cand=0, built=0, fmax=0.0, brute=0, bind=0,
                          vac=0, rer=0, inf=0, disagree=0)
        for seed in range(seeds):
            cand += 1
            tab[label]['cand'] += 1
            inst = build(kind, seed, kw)
            if inst is None:
                continue
            try:
                geom = sa.ArmGeom(inst)
            except Exception:
                continue
            tab[label]['built'] += 1
            for c in cs:
                f, tot = arm_binding_fraction(inst, geom, c)
                tab[label]['fmax'] = max(tab[label]['fmax'], f)
                if not tot or f < need:
                    continue
                r = classify(inst, geom, c)
                if r is None:
                    continue
                tried += 1
                tab[label]['brute'] += 1
                tab[label]['disagree'] += (not r['agree'])
                tab[label][{'binding': 'bind', 'vacuous': 'vac',
                            'rerouted': 'rer', 'infeasible': 'inf'}[r['kind']]] += 1
                rows.append((label, seed, c, f, r))
                if r['binding'] or not r['agree']:
                    print(f"  {'🔴 BINDING' if r['binding'] else '⚠ DISAGREE'} "
                          f"{label} seed {seed} c={c:.2f} f={f:.2f} "
                          f"free {r['m_free']:.4f} arm {r['m_arm']:.4f} "
                          f"k {r['k_free']}->{r['k_arm']}", flush=True)
                if time.time() - t_start > wall_cap:
                    break
            if time.time() - t_start > wall_cap:
                break
        print(f'  {label:18s} cand {tab[label]["cand"]:3d} built '
              f'{tab[label]["built"]:3d} fmax {tab[label]["fmax"]:.2f} '
              f'brute {tab[label]["brute"]:3d}  BIND {tab[label]["bind"]:3d} '
              f'vac {tab[label]["vac"]:3d} rer {tab[label]["rer"]:3d} '
              f'inf {tab[label]["inf"]:3d} disagree '
              f'{tab[label]["disagree"]:2d}', flush=True)
        if time.time() - t_start > wall_cap:
            print('   WALL CAP -- remaining classes NOT MEASURED')
            break

    nb = sum(t['bind'] for t in tab.values())
    nv = sum(t['vac'] for t in tab.values())
    nr = sum(t['rer'] for t in tab.values())
    nd = sum(t['disagree'] for t in tab.values())
    print(f'\nA2.3 effort floor: classes {len(tab)}/>=5, candidates '
          f'{cand}/>=300, brute-forced both ways {tried}/>=40')
    print(f'M3 G13: BINDING {nb}   vacuous {nv}   rerouted {nr}   '
          f'enumerator disagreements {nd} (must be 0)')
    print(f'wall {time.time() - t_start:.0f} s')
    return 0


def gate(pairs, budget=45.0):
    """THE gate G12 could not run: solver vs W3 where the arms actually PRICE.

    p1_g12 B8 is explicit about why its M3 could not discriminate -- "W3
    confirms the solver is never worse than brute force, but it CANNOT detect a
    solver that is too loose, because there is no small instance where loose and
    tight differ." On a BINDING instance they differ by construction: a solver
    that ignores or under-applies ARM_BLOCK returns the arm-INFEASIBLE optimum,
    which is strictly BELOW W3. So both directions are checked here, and the
    `solver < W3` direction is the one that is new.
    """
    from reachability_gng import sched_armfull as af
    bad_hi, bad_lo, rows = [], [], []
    for label, kind, kw, seed, c in pairs:
        inst = build(kind, seed, kw)
        geom = sa.ArmGeom(inst)
        w3 = brute_arm(inst, geom, c, 0.0)
        w3f = brute_arm(inst, geom, np.inf, 0.0)
        s, _ = af.solve_armfull(inst, c_arm=c, geom=geom, c_clear=0.0,
                                time_budget=budget)
        hi = s.makespan > w3 + 1e-6
        lo = s.makespan < w3 - 1e-6
        rows.append((label, seed, c, w3f, w3, float(s.makespan), s.proved))
        if hi:
            bad_hi.append((label, seed, c))
        elif lo:
            bad_lo.append((label, seed, c))
        print(f'  {label:14s} s{seed:<3d} c={c:.2f}  W3(no arm) {w3f:8.4f}  '
              f'W3(arm) {w3:8.4f}  solver {s.makespan:8.4f} '
              f'(proved={int(s.proved)})  '
              f'{"🔴 SOLVER > W3" if hi else "🔴 SOLVER < W3 (TOO LOOSE)" if lo else "match"}',
              flush=True)
    print(f'\nM3 gate on {len(rows)} BINDING instances: '
          f'{len(bad_hi)} times solver > W3 (unsound), '
          f'{len(bad_lo)} times solver < W3 (too loose -- the direction G12 '
          f'could not test)')
    return 0


BINDERS = [('G12-baseline', 'armreal', dict(n=2, n_pose=3, n_mr=0), s, c)
           for s, cs in ((0, (0.10, 0.15, 0.20)), (2, (0.15, 0.20)),
                         (5, (0.15, 0.20)), (9, (0.15, 0.20)),
                         (10, (0.15, 0.20)))
           for c in cs]


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['m3']
    if a[0] == 'scan':
        return scan()
    if a[0] == 'gate':
        return gate(BINDERS)
    return m3(seeds=int(a[1]) if len(a) > 1 else 14)


if __name__ == '__main__':
    raise SystemExit(main())
