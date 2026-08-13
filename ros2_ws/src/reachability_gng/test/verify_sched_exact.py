#!/usr/bin/env python3
"""Is sched.solve_exact actually EXACT? Five independent checks, all mandatory.

docs/p1_g7_sched.md A2-K4: "a solver with no independent comparison is not
ground truth, it is just a second solver". So nothing here reuses the machinery
it is judging.

  V0  traverse_matrix        vs a scalar re-implementation of p1_state 5.6
  V1  sched.stop_duration    vs EXHAUSTIVE slot enumeration -- never touches
                                the max(a, b, z) closed form
  V2  sched.solve_exact      vs a brute-force enumerator over ordered set
                                partitions x poses x arm assignments, with no
                                memoisation, no min-plus, and its own cost model
  V3  sched.solve_exact      vs the pathological instances of A2-K5, whose
                                answers were written down before any solver ran
  V4  the REPORTED SCHEDULE  replayed against the rules from scratch -- the
                                optimal VALUE being right does not prove the
                                schedule handed back with it is executable

Every check compares the OPTIMAL VALUE, not the schedule: ties are common and a
different optimal schedule is not an error. Tolerance is 1e-9 and is not
negotiable -- A2-K4 forbids loosening it to make things agree.

Run:  python3 test/verify_sched_exact.py
Exit: 0 all pass, 1 any failure.
"""

from __future__ import annotations

import itertools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reachability_gng.sched import (DWELL, GANTRY_ARMS,  # noqa: E402
                                    gen_random_small, gen_real, gen_synthetic,
                                    solve_exact, stop_duration,
                                    traverse_matrix)

TOL = 1e-9


# ---------------------------------------------------------------------------
# independent cost model -- typed from p1_state 5.6, not imported
# ---------------------------------------------------------------------------
def ref_traverse(lin_a, rot_a, lin_b, rot_b):
    """T_traverse for one pose pair. Scalar, no broadcasting, no numpy tricks."""
    d_mm = abs(lin_a - lin_b) * 1000.0
    d_deg = abs(np.degrees(rot_a) - np.degrees(rot_b))
    if d_deg > 180.0:
        d_deg = 360.0 - d_deg
    t_lin = 0.29 + d_mm / (3000.0 / 95.4930) if d_mm > 1e-6 else 0.0
    t_rot = 0.26 + d_deg / (1000.0 / 100.0) if d_deg > 1e-6 else 0.0
    return max(t_lin, t_rot)


# ---------------------------------------------------------------------------
# independent stop scheduler -- exhaustive over slots, no closed form anywhere
# ---------------------------------------------------------------------------
def ref_stop_slots(inst, g, tasks, p):
    """Fewest 2.0 s slots to run `tasks` at pose `p`, by brute-force search.

    Places every task on an explicit slot (and, if SR, on an explicit arm) and
    checks the three rules directly:
      - an arm does at most one task per slot                (ST)
      - an MR task occupies BOTH arms of the gantry in its slot
      - at most one zone-occupying task per slot, gantry-wide (mutex r=0.20)
    Returns None if the task set cannot be done at this pose at all.
    """
    reach, zone, hand = inst.reach[g], inst.zone[g], inst.hand[g]
    mr = [i for i in tasks if inst.kind[i] == 'MR']
    sr = [i for i in tasks if inst.kind[i] != 'MR']
    if any(not hand[i, p] for i in mr):
        return None
    arm_opts = [[a for a in (0, 1) if reach[i, p, a]] for i in sr]
    if any(not o for o in arm_opts):
        return None

    for m_slots in range(1, len(tasks) + 1):
        slots = range(m_slots)
        for arms in itertools.product(*arm_opts):
            for sr_slot in itertools.product(slots, repeat=len(sr)):
                for mr_slot in itertools.product(slots, repeat=len(mr)):
                    busy, zone_used = set(), set()
                    ok = True
                    for i, s in zip(mr, mr_slot):
                        if (s, 0) in busy or (s, 1) in busy or s in zone_used:
                            ok = False
                            break
                        busy.add((s, 0))
                        busy.add((s, 1))
                        zone_used.add(s)
                    if not ok:
                        continue
                    for i, a, s in zip(sr, arms, sr_slot):
                        if (s, a) in busy:
                            ok = False
                            break
                        if zone[i, p, a]:
                            if s in zone_used:
                                ok = False
                                break
                            zone_used.add(s)
                        busy.add((s, a))
                    if ok:
                        return m_slots
    return None


# ---------------------------------------------------------------------------
# independent whole-schedule enumerator
# ---------------------------------------------------------------------------
def ref_gantry_cost(inst, g, tasks):
    """Exact completion time for one gantry, by exhaustive enumeration.

    Enumerates ordered set partitions of `tasks` into stops, each stop paired
    with any candidate pose (so revisiting a pose is included), scores every
    stop with ref_stop_slots and every move with ref_traverse. Plain recursion:
    no dynamic programming, no shared state with sched.py.
    """
    poses = inst.poses[g]
    best = [float('inf')]
    memo = {}     # a cache, not a shortcut: same exhaustive search behind it

    def slots_of(block, p):
        key = (block, p)
        if key not in memo:
            memo[key] = ref_stop_slots(inst, g, list(block), p)
        return memo[key]

    def rec(remaining, cur_pose, acc):
        if acc >= best[0] - 1e-12:
            return
        if not remaining:
            best[0] = acc
            return
        rem = sorted(remaining)
        for k in range(1, len(rem) + 1):
            for block in itertools.combinations(rem, k):
                for p in range(len(poses)):
                    slots = slots_of(block, p)
                    if slots is None:
                        continue
                    move = 0.0 if p == cur_pose else ref_traverse(
                        poses[cur_pose][0], poses[cur_pose][1],
                        poses[p][0], poses[p][1])
                    rec(remaining - set(block), p,
                        acc + move + slots * inst.dwell)

    rec(set(tasks), inst.p0[g], 0.0)
    return best[0]


def ref_makespan(inst):
    """Exact makespan by brute force: every task->gantry split, both gantries."""
    gs = inst.gantries
    best = float('inf')
    for combo in itertools.product(gs, repeat=inst.n):
        per = {g: [i for i in range(inst.n) if combo[i] == g] for g in gs}
        m = max(ref_gantry_cost(inst, g, per[g]) for g in gs)
        if m < best:
            best = m
    return best


# ---------------------------------------------------------------------------
# V3 fixtures -- values fixed in docs/p1_g7_sched.md A2-K5 BEFORE any solver ran
# ---------------------------------------------------------------------------
def _synth(n, poses, spec, p0=0, gantries=(1,), kinds=None):
    """spec[g] = (reach, zone, hand) as nested lists/arrays."""
    kinds = kinds if kinds is not None else ['SR'] * n
    return gen_synthetic(kinds, {g: poses for g in gantries},
                         {g: spec[g][0] for g in gantries},
                         {g: spec[g][1] for g in gantries},
                         {g: spec[g][2] for g in gantries},
                         {g: p0 for g in gantries}, gantries=gantries)


def _blank(n, P):
    return (np.zeros((n, P, 2), bool), np.zeros((n, P, 2), bool),
            np.zeros((n, P), bool))


def pathological():
    """(name, instance, expected_makespan, what_it_isolates)."""
    out = []
    one = np.array([[0.0, 0.0]])

    # P1 -- forced serialisation on one arm
    n = 4
    r, z, h = _blank(n, 1)
    r[:, 0, 0] = True
    out.append(('P1 serial', _synth(n, one, {1: (r, z, h)}), n * DWELL,
                'ST: one arm, one task at a time'))

    # P2 -- perfect parallelism, no zone
    r, z, h = _blank(2, 1)
    r[0, 0, 0] = r[1, 0, 1] = True
    out.append(('P2 parallel', _synth(2, one, {1: (r, z, h)}), DWELL,
                'two arms of one gantry run concurrently'))

    # P3 -- pure traverse
    two = np.array([[0.0, 0.0], [0.8, np.deg2rad(90.0)]])
    r, z, h = _blank(1, 2)
    r[0, 1, 0] = True
    exp = ref_traverse(0.0, 0.0, 0.8, np.deg2rad(90.0)) + DWELL
    out.append(('P3 traverse', _synth(1, two, {1: (r, z, h)}), exp,
                'sequence-dependent setup cost'))

    # P4 -- the mutex bites: same as P2 but both tasks occupy the zone
    r, z, h = _blank(2, 1)
    r[0, 0, 0] = r[1, 0, 1] = True
    z[0, 0, 0] = z[1, 0, 1] = True
    out.append(('P4 mutex', _synth(2, one, {1: (r, z, h)}), 2 * DWELL,
                'r=0.20 mutex, THE core quantitative claim'))

    # P5 -- one handover
    r, z, h = _blank(1, 1)
    r[0, 0, :] = True
    h[0, 0] = True
    out.append(('P5 handover', _synth(1, one, {1: (r, z, h)}, kinds=['MR']),
                DWELL, 'MR = one shared two-arm window'))

    # P5b -- handover plus a zone task: MR consumes the mutex too
    r, z, h = _blank(2, 1)
    r[0, 0, :] = True
    h[0, 0] = True
    r[1, 0, 0] = True
    z[1, 0, 0] = True
    out.append(('P5b handover+zone',
                _synth(2, one, {1: (r, z, h)}, kinds=['MR', 'SR']), 2 * DWELL,
                'MR occupies the mutex as well'))

    # P6 -- pose tour: each task lives at exactly one distinct pose
    pl = np.array([[0.0, 0.0],
                   [0.40, np.deg2rad(30.0)],
                   [1.60, np.deg2rad(-120.0)],
                   [0.90, np.deg2rad(175.0)]])
    k = 3
    r, z, h = _blank(k, 4)
    for i in range(k):
        r[i, i + 1, 0] = True
    tour = min(sum(ref_traverse(*pl[a], *pl[b])
                   for a, b in zip((0,) + perm, perm))
               for perm in itertools.permutations(range(1, 4)))
    out.append(('P6 tour', _synth(k, pl, {1: (r, z, h)}), tour + k * DWELL,
                'sequence-dependent ordering (small TSP)'))

    # P7 -- two gantries: makespan is a MAX, not a sum
    r1, z1, h1 = _blank(2, 1)
    r2, z2, h2 = _blank(2, 1)
    r1[0, 0, 0] = True
    r2[1, 0, 0] = True
    inst = gen_synthetic(['SR', 'SR'], {1: one, 2: one}, {1: r1, 2: r2},
                         {1: z1, 2: z2}, {1: h1, 2: h2}, {1: 0, 2: 0},
                         gantries=(1, 2))
    out.append(('P7 two gantries', inst, DWELL, 'makespan = max over gantries'))
    return out


# ---------------------------------------------------------------------------
# checks
# ---------------------------------------------------------------------------
def v0_traverse():
    rng = np.random.default_rng(7)
    lin = rng.choice(np.arange(33) * 0.05, 40)
    rot = np.deg2rad(rng.choice(np.arange(-180, 180, 5), 40))
    poses = np.stack([lin, rot], axis=1)
    T = traverse_matrix(poses)
    worst = 0.0
    for i in range(len(poses)):
        for j in range(len(poses)):
            worst = max(worst, abs(T[i, j] - ref_traverse(*poses[i],
                                                          *poses[j])))
    print(f'V0 traverse matrix vs scalar reference: max |diff| {worst:.3e} '
          f'over {len(poses)**2} pairs')
    return worst < TOL


def v1_stop_duration():
    bad, checked = [], 0
    for seed in range(60):
        inst = gen_random_small(4, 3, seed, n_mr=(1 if seed % 3 == 0 else 0))
        for U in range(1, 1 << inst.n):
            tasks = [i for i in range(inst.n) if U >> i & 1]
            for p in range(len(inst.poses[1])):
                got, _ = stop_duration(inst, 1, U, p)
                ref = ref_stop_slots(inst, 1, tasks, p)
                ref = np.inf if ref is None else ref * inst.dwell
                checked += 1
                if not (np.isinf(got) and np.isinf(ref)) and \
                        abs(got - ref) > TOL:
                    bad.append((seed, U, p, got, ref))
    print(f'V1 closed form vs exhaustive slot search: '
          f'{checked} (subset, pose) cases, {len(bad)} mismatches')
    for b in bad[:5]:
        print(f'   seed {b[0]} U {b[1]:#06b} pose {b[2]}: '
              f'closed {b[3]} vs brute {b[4]}')
    return not bad


def v2_solver():
    bad, checked = [], 0
    cases = [(2, 4, (1,), 0), (3, 4, (1,), 0), (3, 3, (1,), 1),
             (4, 3, (1,), 0), (4, 3, (1,), 1), (4, 3, (1,), 2),
             (3, 3, (1, 2), 0), (3, 4, (1, 2), 1), (4, 2, (1, 2), 1),
             (4, 3, (1, 2), 0), (2, 6, (1,), 0), (3, 5, (1,), 1)]
    for n, P, gs, mr in cases:
        for seed in range(8):
            inst = gen_random_small(n, P, 100 + seed, n_mr=mr, gantries=gs)
            got = solve_exact(inst).makespan
            ref = ref_makespan(inst)
            checked += 1
            if abs(got - ref) > TOL:
                bad.append((n, P, gs, mr, seed, got, ref))
    print(f'V2 DP vs brute-force enumerator: {checked} instances, '
          f'{len(bad)} mismatches')
    for b in bad[:5]:
        print(f'   n={b[0]} |P|={b[1]} gantries={b[2]} mr={b[3]} seed={b[4]}: '
              f'DP {b[5]:.6f} vs brute {b[6]:.6f}')
    return not bad


def v3_pathological():
    bad = []
    print('V3 pathological instances (expected values fixed in g7 A2-K5):')
    for name, inst, exp, why in pathological():
        got = solve_exact(inst).makespan
        ok = abs(got - exp) <= TOL
        print(f'   {"PASS" if ok else "FAIL"}  {name:<20} '
              f'expected {exp:9.4f} s  got {got:9.4f} s   {why}')
        if not ok:
            bad.append(name)
    return not bad


def validate_schedule(inst, sol):
    """Replay the reported schedule against the rules and list every violation.

    Rebuilt from the model text, not from sched.py: traverse from
    ref_traverse, stop length from ref_stop_slots, feasibility straight off the
    oracle arrays. An optimal number attached to an unexecutable schedule is
    still a wrong answer, and nothing else in this file would notice.
    """
    errs, done = [], []
    for g, stops in sol.stops.items():
        t, cur = 0.0, inst.p0[g]
        for st in stops:
            p = st['pose']
            if p != cur:
                t += ref_traverse(inst.poses[g][cur][0], inst.poses[g][cur][1],
                                  inst.poses[g][p][0], inst.poses[g][p][1])
            if abs(t - st['start']) > TOL:
                errs.append(f'g{g} stop start {st["start"]} != replay {t}')
            tasks = [i for i in range(inst.n) if st['tasks'] >> i & 1]
            slots = ref_stop_slots(inst, g, tasks, p)
            if slots is None:
                errs.append(f'g{g} stop at pose {p} is INFEASIBLE')
            elif abs(slots * inst.dwell - st['dur']) > TOL:
                errs.append(f'g{g} stop dur {st["dur"]} != replay '
                            f'{slots * inst.dwell}')
            for i in tasks:
                a = st['assign'][i]
                if inst.kind[i] == 'MR':
                    if a != 'both' or not inst.hand[g][i, p]:
                        errs.append(f'g{g} MR task {i} not a valid handover')
                elif a not in GANTRY_ARMS[g]:
                    errs.append(f'g{g} task {i} assigned to {a}')
                elif not inst.reach[g][i, p, GANTRY_ARMS[g].index(a)]:
                    errs.append(f'g{g} task {i} unreachable by {a} at pose {p}')
            done += tasks
            t += st['dur']
            cur = p
        if abs(t - sol.finish[g]) > TOL:
            errs.append(f'g{g} finish {sol.finish[g]} != replay {t}')
    if sorted(done) != list(range(inst.n)):
        errs.append(f'tasks done {sorted(done)} != every task exactly once')
    if abs(max(sol.finish.values()) - sol.makespan) > TOL:
        errs.append('makespan is not the max over gantry finish times')
    return errs


def v4_schedule_replay():
    bad, checked = [], 0
    insts = [gen_random_small(n, P, 200 + s, n_mr=mr, gantries=gs)
             for n, P, gs, mr in [(4, 3, (1,), 0), (4, 3, (1,), 1),
                                  (4, 3, (1, 2), 1), (3, 5, (1,), 0)]
             for s in range(6)]
    insts += [gen_real(n, s, mr, gs) for n, mr, gs in
              [(4, 0, (1,)), (5, 1, (1, 2)), (6, 1, (1,))] for s in range(3)]
    for inst in insts:
        errs = validate_schedule(inst, solve_exact(inst))
        checked += 1
        if errs:
            bad.append((inst.label, errs))
    print(f'V4 reported schedule replayed from the rules: {checked} schedules, '
          f'{len(bad)} with violations')
    for label, errs in bad[:5]:
        print(f'   {label}: {errs[:3]}')
    return not bad


def main():
    checks = [('V0', v0_traverse), ('V1', v1_stop_duration),
              ('V2', v2_solver), ('V3', v3_pathological),
              ('V4', v4_schedule_replay)]
    results = []
    for tag, fn in checks:
        ok = fn()
        results.append((tag, ok))
        print()
    print('=' * 60)
    for tag, ok in results:
        print(f'{tag}: {"PASS" if ok else "FAIL"}')
    allok = all(ok for _, ok in results)
    print('=' * 60)
    print('EXACTNESS ESTABLISHED' if allok else
          'EXACTNESS NOT ESTABLISHED -- solver may NOT be called ground truth')
    return 0 if allok else 1


if __name__ == '__main__':
    raise SystemExit(main())
