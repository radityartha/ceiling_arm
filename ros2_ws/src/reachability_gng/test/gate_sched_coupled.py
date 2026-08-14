#!/usr/bin/env python3
"""The wait-aware validity gate. docs/p1_g10_sched4.md A2.4.

p1_g9 B4 measured why this file has to exist. `validate_schedule()` in
test/verify_sched_exact.py replays a schedule as

    t += traverse ; t += dwell

with no place for WAITING, which is correct for the G7/G8 model -- there waiting
is never useful, so it never appears -- and structurally unable to validate a
schedule from the coupled model, where A2.5 makes waiting free and sometimes
optimal. G9 therefore reported 3 of 12 coupled schedules as violations that were
not violations at all. That was an error in G9's A, not a finding about its
solver.

This is the successor, written NEW. test/verify_sched_exact.py stays byte
identical (its `git diff` has been empty since G7 and A0 keeps it that way);
what is REUSED from it, unmodified and by import, is the pair of independent
reference functions it already owns:

    ref_traverse    typed from p1_state 5.6, not from sched.py
    ref_stop_slots  exhaustive slot placement, never touches the closed form

so the intra-gantry half of this gate is exactly as independent as G7 V4 was.

WHAT CHANGED IN THE SCHEDULE REPRESENTATION, and why (A2.4):

  A stop (pose, start, dur) does NOT say when the traverse happened. A2.5 lets a
  gantry hold at its current pose before departing AND hold at the target after
  arriving, and those two readings put the moving body in different places at
  different times -- different collision footprint, same stop list. So a coupled
  stop carries BOTH times:

      depart  when the traverse to this pose begins
      start   when the dwell begins

      depart >= previous stop's (start + dur)          hold at the source
      start  >= depart + T(previous pose, this pose)   hold at the target

  A G7/G8 stop with no `depart` is read as depart = start - T, i.e. leave as
  late as possible, which is what those schedules meant.

Run:  python3 test/gate_sched_coupled.py      # M1-M5 mutation self-test
Exit: 0 all pass, 1 any failure.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_coll as sc               # noqa: E402
from verify_sched_exact import ref_stop_slots, ref_traverse  # noqa: E402

TOL = 1e-9
GATE_EPS = sc.EPS_CERT / 10.0      # A3-K4: ten times tighter than the solver's


def stop_depart(inst, g, st, cur_pose_idx):
    """`depart` of a stop, defaulting to the G7/G8 reading (leave as late as
    possible). Kept in one place so the default can never drift."""
    if 'depart' in st:
        return float(st['depart'])
    p0 = inst.poses[g][cur_pose_idx]
    p1 = inst.poses[g][st['pose']]
    return float(st['start']) - ref_traverse(p0[0], p0[1], p1[0], p1[1])


def traj_from_stops(inst, g, stops):
    """Rebuild the gantry's trajectory from (depart, pose). Rules, not solver.

    Returns (Traj, errors). A leg that starts before the previous one ends is
    reported rather than raised: a corrupted schedule must produce a finding,
    not a traceback.
    """
    errs = []
    tr = sc.Traj(g, tuple(inst.poses[g][inst.p0[g]]))
    cur = inst.p0[g]
    for k, st in enumerate(stops):
        d = stop_depart(inst, g, st, cur)
        q = tuple(inst.poses[g][st['pose']])
        try:
            tr.append(d, q)
        except ValueError:
            errs.append(f'g{g} stop {k}: leg departs at {d:.4f} while the '
                        f'previous leg is still running')
            return tr, errs
        cur = st['pose']
    return tr, errs


def validate_coupled(inst, stops, finish=None, makespan=None,
                     c_clear=sc.C_CLEAR, eps=GATE_EPS, check_collision=True):
    """Replay a COUPLED schedule against every rule and list all violations.

    Intra-gantry rules (A2.5 + p1_g7 A1), replayed from the reference
    functions:
      * depart >= previous stop end            (waiting at the source, >= 0)
      * start  >= depart + ref_traverse        (waiting at the target, >= 0)
      * dur    == ref_stop_slots * dwell       (slots, mutex, MR, arm reach)
      * every task exactly once, arm assignments legal at that pose
      * finish[g] == last stop end, makespan == max finish

    Coupling rule (A2.4), replayed separately at eps = EPS_CERT/10 from
    trajectories rebuilt out of `depart`.
    """
    errs, done = [], []
    ends = {}
    for g, sg in sorted(stops.items()):
        t, cur, last_work = 0.0, inst.p0[g], 0.0
        for k, st in enumerate(sg):
            p = st['pose']
            pa, pb = inst.poses[g][cur], inst.poses[g][p]
            T = ref_traverse(pa[0], pa[1], pb[0], pb[1])
            d = stop_depart(inst, g, st, cur)
            if d < t - TOL:
                errs.append(f'g{g} stop {k} departs at {d:.6f} before the '
                            f'previous stop ends at {t:.6f}')
            if st['start'] < d + T - TOL:
                errs.append(f'g{g} stop {k} starts at {st["start"]:.6f} but '
                            f'cannot arrive before {d + T:.6f}')
            tasks = [i for i in range(inst.n) if st['tasks'] >> i & 1]
            # An EMPTY stop is an evasive move: p1_g9 contradiction 2 put pure
            # get-out-of-the-way traverses into the model, and A2.4 of this
            # session did not say how to replay one. Rule, fixed here: no tasks
            # means no dwell, and ref_stop_slots is not consulted at all (it
            # enumerates slot placements and has no answer for zero tasks).
            if not tasks:
                if abs(st['dur']) > TOL:
                    errs.append(f'g{g} stop {k} carries no task but dwells '
                                f'{st["dur"]}')
            elif (slots := ref_stop_slots(inst, g, tasks, p)) is None:
                errs.append(f'g{g} stop {k} at pose {p} is INFEASIBLE')
            elif abs(slots * inst.dwell - st['dur']) > TOL:
                errs.append(f'g{g} stop {k} dur {st["dur"]} != replay '
                            f'{slots * inst.dwell}')
            for i in tasks:
                a = st['assign'][i] if st.get('assign') else None
                if inst.kind[i] == 'MR':
                    if a != 'both' or not inst.hand[g][i, p]:
                        errs.append(f'g{g} MR task {i} is not a valid handover')
                elif a not in sched.GANTRY_ARMS[g]:
                    errs.append(f'g{g} task {i} assigned to {a}')
                elif not inst.reach[g][i, p, sched.GANTRY_ARMS[g].index(a)]:
                    errs.append(f'g{g} task {i} unreachable by {a} at pose {p}')
            done += tasks
            t = st['start'] + st['dur']
            if tasks:
                last_work = t
            cur = p
        # completion = end of the last DWELL. A gantry that steps aside after
        # finishing its work has not extended the makespan (p1_g7 A2-K2 defines
        # it as the end of the last task's dwell window), and counting the
        # evasive leg would inflate it by a whole traverse on exactly the
        # instances where collision binds.
        ends[g] = last_work
        if finish is not None and abs(last_work - finish.get(g, last_work)) > TOL:
            errs.append(f'g{g} finish {finish[g]} != replay {last_work}')
    if sorted(done) != list(range(inst.n)):
        errs.append(f'tasks done {sorted(done)} != every task exactly once')
    if makespan is not None and abs(max(ends.values()) - makespan) > TOL:
        errs.append(f'makespan {makespan} != max over gantry finishes '
                    f'{max(ends.values())}')

    if check_collision and len(stops) >= 2:
        gs = sorted(stops)
        trs = {}
        for g in gs:
            tr, e = traj_from_stops(inst, g, stops[g])
            errs += e
            trs[g] = tr
        A, B = trs[gs[0]], trs[gs[1]]
        hit = sc.first_block(A, B, 0.0, max(A.end_time(), B.end_time(),
                                            max(ends.values())),
                             c_clear, eps)
        if hit is not None:
            errs.append(f'A2.4 violated at t = {hit:.4f} s')
    return errs


def total_wait(inst, stops):
    """Seconds spent holding (source + target), the quantity G7/G8 cannot express.

    Reported rather than checked: it is how we know the gate is being asked a
    question the old one could not answer.
    """
    w = 0.0
    for g, sg in stops.items():
        t, cur = 0.0, inst.p0[g]
        for st in sg:
            pa, pb = inst.poses[g][cur], inst.poses[g][st['pose']]
            T = ref_traverse(pa[0], pa[1], pb[0], pb[1])
            d = stop_depart(inst, g, st, cur)
            w += max(0.0, d - t) + max(0.0, st['start'] - d - T)
            t, cur = st['start'] + st['dur'], st['pose']
    return w


# ===========================================================================
# M1-M5 -- the mutation self-test. A2.4, in red: a gate that never rejects
# anything is not evidence (p1_g8 B1, where the K4 gate never fired once).
# ===========================================================================
def _blocking_pair_instance():
    """Two gantries, one task each, at poses that block each other.

    Poses: (0.8, 0), (0.8, -90 deg), (0.8, +90 deg). W0c measured the last two
    as BLOCK at dlin = 0, so a schedule that dwells at both simultaneously is
    physically impossible and the gate must say so.
    """
    P = np.array([[0.8, 0.0], [0.8, -np.pi / 2], [0.8, np.pi / 2]])
    n, K = 2, 3
    r1 = np.zeros((n, K, 2), bool)
    r2 = np.zeros((n, K, 2), bool)
    r1[0, 1, 0] = True
    r2[1, 2, 0] = True
    z = np.zeros((n, K, 2), bool)
    h = np.zeros((n, K), bool)
    return sched.gen_synthetic(['SR', 'SR'], {1: P, 2: P}, {1: r1, 2: r2},
                               {1: z, 2: z}, {1: h, 2: h}, {1: 0, 2: 0},
                               'gate-blocking', gantries=(1, 2))


def _hand_schedules():
    """(instance, GOOD schedule, BAD schedule) built by hand from A2.3/A2.5.

    GOOD serialises: gantry 1 works, returns to the safe rot = 0 pose, and only
    then does gantry 2 depart. BAD is the same two stops run concurrently. The
    numbers come from A2.3 -- 90 deg at 10 deg/s plus the 0.26 s dead time.
    """
    inst = _blocking_pair_instance()
    T = 0.26 + 90.0 / 10.0                       # 9.26 s, pose 0 <-> 1 or 2
    good = {
        1: [dict(pose=1, depart=0.0, start=T, dur=2.0, tasks=1,
                 assign={0: 'arm1'})],
        2: [dict(pose=2, depart=T + 2.0 + T, start=T + 2.0 + T + T, dur=2.0,
                 tasks=2, assign={1: 'arm3'})],
    }
    # gantry 1 has to clear out before gantry 2 arrives; it goes back to pose 0
    good[1].append(dict(pose=0, depart=T + 2.0, start=T + 2.0 + T, dur=0.0,
                        tasks=0, assign={}))
    bad = {
        1: [dict(pose=1, depart=0.0, start=T, dur=2.0, tasks=1,
                 assign={0: 'arm1'})],
        2: [dict(pose=2, depart=0.0, start=T, dur=2.0, tasks=2,
                 assign={1: 'arm3'})],
    }
    return inst, good, bad


def _valid_real_schedules(k=3):
    """Real coupled-valid schedules to mutate: uncoupled optima that happen to
    be collision-free (Lemma 4), so they are optimal coupled schedules too.

    Using these rather than the new solver's output is deliberate -- the gate
    has to be trustworthy BEFORE the solver it will judge exists (A7 order).
    """
    out = []
    for n, mr, seed in [(4, 0, 0), (4, 1, 0), (6, 0, 1), (4, 0, 3), (6, 1, 0)]:
        inst = sched.gen_real(n, seed, mr, (1, 2))
        sol = sched.solve_exact(inst)
        if sc.schedule_conflict(inst, sol.stops) is None:
            out.append((f'n{n}_mr{mr}_s{seed}', inst, sol))
        if len(out) >= k:
            break
    return out


def _binding_real_schedules():
    """Real instances whose UNCOUPLED optimum provably collides.

    p1_g9 B6 measured 13 of these in S1; two of them are taken by name. Their
    uncoupled optimum is a genuine, physically impossible schedule produced by
    a solver rather than by a mutation function -- the strongest negative the
    A2.4 half of this gate can be given, and it costs nothing to run.
    """
    out = []
    for n, mr, seed in [(4, 0, 4), (4, 0, 7)]:
        inst = sched.gen_real(n, seed, mr, (1, 2))
        sol = sched.solve_exact(inst)
        if sc.schedule_conflict(inst, sol.stops) is not None:
            out.append((f'n{n}_mr{mr}_s{seed}', inst, sol))
    return out


def _mutations(inst, stops):
    """M1-M5 of A2.4. (name, mutated stops); every one must be REJECTED."""
    out = []
    gs = sorted(stops)

    def _clone():
        return {g: [dict(s) for s in stops[g]] for g in gs}

    g0 = next((g for g in gs if stops[g]), None)
    if g0 is None:
        return out

    m = _clone()                                    # M1: start pulled earlier
    m[g0][0]['start'] -= 0.5
    out.append(('M1 start pulled 0.5 s earlier', m))

    # M2: depart pulled before the previous stop has finished -- the gantry
    # drives off mid-dwell. This replaces the first draft of M2 ("shift the
    # second gantry onto the first"), which ESCAPED twice: on a Lemma-4
    # instance the two gantries do not interfere, so shifting one of them is a
    # legal schedule and the gate was right to accept it. A mutation that is
    # not actually a violation tests nothing.
    m = _clone()
    g2 = next((g for g in gs if len(m[g]) >= 2), None)
    if g2 is not None:
        prev = m[g2][0]
        m[g2][1] = dict(m[g2][1],
                        depart=prev['start'] + prev['dur'] - 1.0)
        out.append(('M2 departs 1.0 s before the previous dwell ends', m))
    else:
        m[g0][0] = dict(m[g0][0], depart=-1.0)
        out.append(('M2 departs before t = 0', m))

    m = _clone()                                    # M3: a task disappears
    for g in gs:
        for s in m[g]:
            if bin(s['tasks']).count('1') >= 1:
                i = (s['tasks'] & -s['tasks']).bit_length() - 1
                s['tasks'] &= ~(1 << i)
                s['assign'] = {k: v for k, v in s['assign'].items() if k != i}
                break
        else:
            continue
        break
    out.append(('M3 one task dropped from a stop', m))

    m = _clone()                                    # M4: illegal arm
    done = False
    for g in gs:
        for s in m[g]:
            for i, a in list(s['assign'].items()):
                if a == 'both':
                    continue
                other = [x for x in sched.GANTRY_ARMS[g] if x != a][0]
                k = sched.GANTRY_ARMS[g].index(other)
                if not inst.reach[g][i, s['pose'], k]:
                    s['assign'] = dict(s['assign'])
                    s['assign'][i] = other
                    done = True
                    break
            if done:
                break
        if done:
            break
    if done:
        out.append(('M4 task moved to an arm that cannot reach it', m))

    m = _clone()                                    # M5: a slot shaved off
    for g in gs:
        if m[g] and m[g][0]['dur'] > inst.dwell:
            m[g][0]['dur'] -= inst.dwell
            out.append(('M5 one dwell slot removed', m))
            break
    else:
        m = _clone()
        m[gs[0]][0]['dur'] += inst.dwell
        out.append(('M5 one dwell slot added', m))
    return out


def main():
    ok = True

    # positive control first: the gate must ACCEPT a hand-built legal schedule.
    # A gate that rejects everything is as useless as one that accepts
    # everything, and only this direction catches that.
    inst, good, bad = _hand_schedules()
    e_good = validate_coupled(inst, good)
    e_bad = validate_coupled(inst, bad)
    p_ok = not e_good and any('A2.4' in e for e in e_bad)
    ok &= p_ok
    print(f'P  hand-built serialised schedule at mutually blocking poses: '
          f'{len(e_good)} violations (must be 0); the concurrent version: '
          f'{len(e_bad)} violations, A2.4 among them = '
          f'{any("A2.4" in e for e in e_bad)} -> {"PASS" if p_ok else "FAIL"}')
    if e_good:
        print(f'     unexpected: {e_good[:3]}')
    print(f'     wait time the OLD gate could not express: '
          f'{total_wait(inst, good):.3f} s in the legal schedule')

    # the old gate, on the same legal schedule, for the record
    from verify_sched_exact import validate_schedule

    class _S:
        pass
    s_ = _S()
    s_.stops = good
    s_.finish = {g: (good[g][-1]['start'] + good[g][-1]['dur'])
                 for g in good}
    s_.makespan = max(s_.finish.values())
    old = validate_schedule(inst, s_)
    print(f'     for the record, the G7/G8 gate on that same LEGAL schedule: '
          f'{len(old)} "violations" -- p1_g9 B4, contradiction 4')

    # mutation battery on real instances
    cases = _valid_real_schedules(3)
    print(f'\nM  mutation battery on {len(cases)} real Lemma-4 schedules')
    n_mut = n_escaped = 0
    for label, i2, sol in cases:
        errs = validate_coupled(i2, sol.stops, sol.finish, sol.makespan)
        if errs:
            ok = False
            print(f'   {label}: BASELINE SCHEDULE REJECTED {errs[:2]} -> FAIL')
            continue
        for name, mut in _mutations(i2, sol.stops):
            n_mut += 1
            me = validate_coupled(i2, mut)
            if not me:
                n_escaped += 1
                print(f'   {label}: {name} -> ESCAPED (gate said OK)')

    # M6: the A2.4 half, on schedules no mutation function wrote -- real
    # uncoupled optima that p1_g9 B6 measured as physically impossible.
    n6 = 0
    for label, i2, sol in _binding_real_schedules():
        n_mut += 1
        n6 += 1
        me = validate_coupled(i2, sol.stops)
        hit = [e for e in me if 'A2.4' in e]
        if not hit:
            n_escaped += 1
            print(f'   {label}: M6 colliding uncoupled optimum -> ESCAPED')
        else:
            print(f'   {label}: M6 colliding uncoupled optimum rejected, '
                  f'{hit[0]}')
    m_ok = n_mut >= 15 and n_escaped == 0 and n6 >= 2
    ok &= m_ok
    print(f'   {n_mut} mutations (A2.4 demands >= 15), {n_escaped} escaped '
          f'(demands 0) -> {"PASS" if m_ok else "FAIL"}')

    print('\n' + '=' * 62)
    print('WAIT-AWARE GATE READY' if ok else 'GATE NOT READY -- do not use it')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
