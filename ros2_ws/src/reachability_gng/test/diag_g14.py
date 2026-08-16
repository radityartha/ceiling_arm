#!/usr/bin/env python3
"""G14 diagnostics. docs/p1_g14_reach4.md A2.1, A2.3, A2.5.

Three things that share nothing but the fact that each is a MEASUREMENT the
previous session could name but not make. None of them is allowed to move a
Delta (A2.7), and none of them writes to a frozen file.

  relabel  TASK 1 (A2.1 K2/K3). Option (b) was chosen and locked BEFORE any
           measurement: the 120 s figure is a SEARCH budget, never a wall
           budget, and the wall is reported separately. This is the part of
           that decision that is work rather than wording -- it re-reads G13's
           artifacts and prints every cell with BOTH numbers, plus the
           NO SCHEDULE FOUND rows carrying the wall they actually consumed.
           R0: every column it does not relabel must reproduce p1_g13 B5.

  park     TASK 3 (A2.3). The 4/6/1/11 instances with NO SCHEDULE FOUND are a
           BUDGET, not an infeasibility -- Lemma C passes at both c_arm. So
           where does arm_serial_ub actually lose? Profiled per park pose over
           four rejection stages, split into the PARKING phase (which Lemma C
           certifies) and the SERIAL TAIL (which it says nothing about,
           p1_g13 B5.5 contradiction 2). The decision rule between (a) and (b)
           is locked in A2.3 and is read off the S3+S4 share.

           This re-implements arm_serial_ub's loop ONLY as an instrument: the
           predicates it calls (sc.schedule_conflict, af.schedule_arm_conflict)
           are the frozen ones, and the acceptance test is byte-identical to
           the frozen loop's. It counts where each pose dies; it does not
           decide anything the solver decides.

  grid     TASK 5 (A2.5). The last standing candidate for `solver < W2`
           (p1_g13 B8): Brute._starts samples departures on a UNIFORM grid
           anchored at the gantry's own clock, while feasible_starts computes
           them CONTINUOUSLY. Replay the solver's schedule and ask, at each
           stop, whether its departure was a time W2 could ever have tried.

Run:  python3 test/diag_g14.py relabel | park [set c n] | grid
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reachability_gng import sched                          # noqa: E402
from reachability_gng import sched_arm as sa                # noqa: E402
from reachability_gng import sched_armfull as af            # noqa: E402
from reachability_gng import sched_coll as sc               # noqa: E402
from reachability_gng import sched_coupled as sk            # noqa: E402
from reachability_gng.irm_sweep import polyline_min_dist    # noqa: E402

SEARCH_BUDGET = 120.0          # A2.1: this is a SEARCH budget. Not a wall cap.
C_SWEEP = (0.00, 0.05, 0.10, 0.15, 0.20)


# =========================================================== TASK 1: relabel
def _g13(tag, c):
    q = Path(f'/tmp/g13_eval_{tag}_{c:.2f}.json')
    return json.loads(q.read_text()) if q.exists() else None


def relabel():
    """A2.1 K2/K3 over G13's artifacts. The Deltas are NOT recomputed."""
    print('TASK 1 -- A2.1 option (b): 120 s is the SEARCH budget; wall is')
    print('reported separately. No Delta changes; the LABELS do.\n')
    print(f'{"cell":14s} {"n":>3s} {"wall mean":>10s} {"wall max":>9s} '
          f'{"> search":>9s} {"overshoot max":>14s}  NO SCHEDULE FOUND')
    tot = over = 0
    for tag in ('s1', 's2'):
        for c in C_SWEEP:
            raw = _g13(tag, c)
            if raw is None:
                print(f'{tag} c={c:.2f}   TIDAK DIUKUR')
                continue
            db = raw[tag]
            ck = f'{c:.2f}'
            rows = [(k, r, r['arm'][ck]) for k, r in db.items()
                    if ck in r.get('arm', {})]
            w = np.array([a['wall'] for _, _, a in rows])
            hot = w[w > SEARCH_BUDGET]
            none = [(k, a['wall']) for k, _, a in rows
                    if not np.isfinite(a['full_ub'])]
            tot += len(rows)
            over += len(hot)
            lab = ('-' if not none else
                   ', '.join(f'{k} (pencarian {SEARCH_BUDGET:.0f} s; '
                             f'wall {v:.1f} s)' for k, v in none[:3])
                   + (f' +{len(none) - 3}' if len(none) > 3 else ''))
            print(f'{tag} c={c:.2f}     {len(rows):3d} {w.mean():9.1f}s '
                  f'{w.max():8.1f}s {len(hot):9d} '
                  f'{(hot.max() - SEARCH_BUDGET if len(hot) else 0.0):13.1f}s'
                  f'  {lab}')
    print(f'\nTOTAL {over} of {tot} instance-runs exceeded the SEARCH budget '
          f'as WALL.')
    print('K4: the wall is NOT capped anywhere in the stack. Every makespan is '
          'either\n    a proof of optimality or a valid upper bound, so no '
          'Delta depends on it;\n    the two-sided verdict depends on '
          'exact/proved, i.e. on the SEARCH budget,\n    which is uniform at '
          f'{SEARCH_BUDGET:.0f} s for every instance in every cell.')
    return 0


# ============================================================== TASK 3: park
STAGES = ('S1 struct/park', 'S2 arm/park', 'S3 struct/tail', 'S4 arm/tail',
          'ACCEPT')


def _park_profile(inst, ctx, sol0, c_clear, cap=None):
    """Where every park pose dies in arm_serial_ub's loop. A2.3.

    Mirrors sched_armfull.arm_serial_ub exactly -- same order (`argsort` of
    traverse time), same two trial schedules, same four predicate calls -- but
    it does NOT break on acceptance, because the question is the distribution
    and not the first survivor. `first` records what the frozen loop WOULD have
    returned, so the instrument can be checked against the thing it profiles.
    """
    gs = inst.gantries
    out = []
    for lead, park in (gs, gs[::-1]):
        cnt = dict.fromkeys(STAGES, 0)
        first = None
        lead_stops = sk._stops_with_depart(inst, lead, sol0.stops[lead])
        t_lead = max((s['start'] + s['dur'] for s in lead_stops), default=0.0)
        P = inst.poses[park]
        cur = P[inst.p0[park]]
        order = np.argsort(sched.traverse_time(P[:, 0] - cur[0],
                                               P[:, 1] - cur[1]), kind='stable')
        if cap:
            order = order[:cap]
        for p in order:
            p = int(p)
            T0 = sc.leg_duration(tuple(cur), tuple(P[p]))
            st = ([] if p == int(inst.p0[park]) else
                  [dict(pose=p, depart=0.0, start=T0, dur=0.0, tasks=0,
                        assign={})])
            trial = {lead: lead_stops, park: st}
            if sc.schedule_conflict(inst, trial, c_clear) is not None:
                cnt['S1 struct/park'] += 1
                continue
            if (not ctx.off and af.schedule_arm_conflict(
                    inst, ctx.geom, trial, ctx.c_arm, ctx.eps,
                    ctx.lemma_b) is not None):
                cnt['S2 arm/park'] += 1
                continue
            rest = sk._stops_with_depart(inst, park, sol0.stops[park])
            t, cur2, moved = max(t_lead, T0), p, []
            for s in rest:
                T = sc.leg_duration(tuple(P[cur2]), tuple(P[s['pose']]))
                moved.append(dict(s, depart=t, start=t + T))
                t += T + s['dur']
                cur2 = s['pose']
            fullsch = {lead: lead_stops, park: st + moved}
            if sc.schedule_conflict(inst, fullsch, c_clear) is not None:
                cnt['S3 struct/tail'] += 1
                continue
            if (not ctx.off and af.schedule_arm_conflict(
                    inst, ctx.geom, fullsch, ctx.c_arm, ctx.eps,
                    ctx.lemma_b) is not None):
                cnt['S4 arm/tail'] += 1
                continue
            cnt['ACCEPT'] += 1
            if first is None:
                first = (p, max(t_lead, t))
        out.append((lead, park, cnt, first, len(order)))
    return out


def park(setname='s1', c=0.15, keys=None, cap=None, wall_cap=3600.0):
    """A2.3: profile the NO SCHEDULE FOUND instances, then read the rule."""
    gen = sched.gen_real if setname == 's1' else sa.gen_real_rotcrowded
    raw = _g13(setname, c)
    if keys is None:
        ck = f'{c:.2f}'
        keys = [k for k, r in raw[setname].items()
                if ck in r.get('arm', {})
                and not np.isfinite(r['arm'][ck]['full_ub'])]
    print(f'TASK 3 -- park-pose rejection profile, {setname} c_arm={c}')
    print(f'  NO SCHEDULE FOUND on {len(keys)}: {keys}\n', flush=True)

    agg = dict.fromkeys(STAGES, 0)
    t_start = time.time()
    done, lemma_gap = [], 0
    for key in keys:
        n, seed, mr = (int(key.split('_')[0][1:]), int(key.split('_')[1][1:]),
                       int(key.split('_')[2][2:]))
        inst = gen(n, seed, mr, (1, 2))
        geom = sa.ArmGeom(inst)
        ctx = af.ArmCtx(inst, geom, c, c_clear=0.0)
        sol0 = sched.solve_exact(inst)
        t0 = time.time()
        prof = _park_profile(inst, ctx, sol0, 0.0, cap=cap)
        # A2.3 palang: is lemma_c NECESSARY? Compare its free set against the
        # poses that actually survived all four stages. A pose accepted by the
        # frozen loop but rejected by lemma_c refutes necessity (D41).
        st0 = {g: sk._stops_with_depart(inst, g, sol0.stops[g])
               for g in inst.gantries}
        lc = af.lemma_c(inst, ctx, st0)
        for lead, prk, cnt, first, ntried in prof:
            for k in STAGES:
                agg[k] += cnt[k]
            free = [f'{k}/{P}' for g, _, k, P in lc if g == lead]
            print(f'  {key:12s} lead {lead} park {prk}: '
                  + '  '.join(f'{k.split()[0]} {cnt[k]:5d}' for k in STAGES)
                  + f'   ({ntried} poses, {time.time()-t0:.0f}s)'
                  + f'  lemma_c free {",".join(free) or "-"}', flush=True)
            if first is not None:
                print(f'      🔴 the frozen loop WOULD have accepted pose '
                      f'{first[0]} -> makespan {first[1]:.4f}')
        done.append(key)
        if time.time() - t_start > wall_cap:
            print(f'   WALL CAP {wall_cap:.0f}s -- remaining NOT MEASURED: '
                  f'{[k for k in keys if k not in done]}')
            break

    tot = sum(agg.values())
    if not tot:
        print('nothing profiled')
        return 0
    tail = agg['S3 struct/tail'] + agg['S4 arm/tail']
    parkph = agg['S1 struct/park'] + agg['S2 arm/park']
    print(f'\n  measured on {len(done)} of {len(keys)} instances '
          f'({", ".join(done)})')
    for k in STAGES:
        print(f'    {k:16s} {agg[k]:7d}   {100.0*agg[k]/tot:5.1f} %')
    print(f'\n  PARKING phase (Lemma C certifies this)     {parkph:7d}   '
          f'{100.0*parkph/tot:5.1f} %')
    print(f'  SERIAL TAIL   (Lemma C says NOTHING here)  {tail:7d}   '
          f'{100.0*tail/tot:5.1f} %')
    print(f'  ACCEPTED                                   {agg["ACCEPT"]:7d}')
    r_tail, r_s2 = 100.0 * tail / tot, 100.0 * agg['S2 arm/park'] / tot
    v = ('(b) -- Lemma C certifies the wrong phase' if r_tail >= 80.0 else
         '(a) MAY be considered -- but A2.3 palang (necessity) must pass first'
         if r_s2 >= 80.0 else '(b) -- mixed, split reported above')
    print(f'\n  A2.3 DECISION RULE -> {v}')
    if agg['ACCEPT']:
        print('  🔴 A pose passed ALL FOUR stages while the solver reported NO '
              'SCHEDULE:\n     that is a BUG in a frozen file, p1_g10 A0.')
    return 0


# ============================================================== TASK 5: grid
def _starts(t_min, other_end, ds):
    """Brute._starts, retyped -- same shape, so the comparison is against the
    times W2 could actually try, not against a description of them."""
    hi = max(t_min, other_end)
    out = [t_min]
    s = t_min + ds
    while s < hi - 1e-9:
        out.append(s)
        s += ds
    if hi > t_min + 1e-9:
        out.append(hi)
    return out


def grid(budget=120.0, ds=0.25):
    """A2.5: are the solver's departures times W2 could ever have sampled?"""
    from verify_sched_coupled import Brute, DS, s5_instances, ref_end_time
    ds = DS
    print(f'TASK 5 -- U4: is `solver < W2` the START GRID? (ds = {ds})\n')
    rows = []
    for label, inst in s5_instances():
        s = sk.solve_coupled2(inst, time_budget=budget)
        b1 = Brute(inst, ds=ds, max_evade=1, budget=240.0)
        v1, _ = b1.solve()
        if not (s.makespan < v1 - 1e-9):
            continue
        gs = inst.gantries
        # replay: each gantry's clock, and the opponent's COMMITTED motion, in
        # the order the departures actually happen -- the same state W2's
        # recursion would have been in when it chose this action.
        ev = sorted(((st['depart'], g, st) for g in gs for st in s.stops[g]),
                    key=lambda x: (x[0], x[1]))
        clk = {g: 0.0 for g in gs}
        trj = {g: (tuple(inst.poses[g][inst.p0[g]]), []) for g in gs}
        off, tot = [], 0
        for dep, g, st in ev:
            h = [k for k in gs if k != g][0]
            cand = _starts(clk[g], ref_end_time(trj[h]), ds)
            tot += 1
            if min(abs(dep - x) for x in cand) > 1e-9:
                off.append((g, round(dep, 6), round(min(cand, key=lambda x:
                                                        abs(dep - x)), 6)))
            q = tuple(inst.poses[g][st['pose']])
            cur = trj[g][0] if not trj[g][1] else trj[g][1][-1][2]
            T = sc.leg_duration(cur, q)
            trj[g] = (trj[g][0], trj[g][1] + ([(dep, cur, q)] if T > 0 else []))
            clk[g] = st['start'] + st['dur']
        rows.append((label, float(s.makespan), float(v1), tot, off))
        print(f'  {label:22s} solver {s.makespan:9.4f}  W2 {v1:9.4f}  '
              f'stops {tot:2d}  OFF-GRID {len(off):2d}'
              + (f'  e.g. {off[0]}' if off else '  (all on grid)'), flush=True)
    if not rows:
        print('U4: 0 instances with solver < W2 -- nothing to test')
        return 0
    k = sum(1 for _, _, _, _, o in rows if o)
    print(f'\nU4 GRID TEST: {k} of {len(rows)} instances have >= 1 departure '
          f'OFF the W2 grid')
    v = ('CONFIRMED -- the gap is W2 discretisation, not the solver'
         if k >= 5 else
         'REFUTED -- no named candidate remains for `solver < W2`'
         if k == 0 else f'PARTIAL -- {k} of {len(rows)}, not promoted to closed')
    print(f'  A2.5 VERDICT: {v}')
    return 0


def necess(setname='s1', c=0.15, limit=12):
    """A2.3 PALANG: is lemma_c a NECESSARY condition for arm_serial_ub?

    The profile in `park` can only look at instances where NOTHING is accepted,
    so it cannot answer this. Here the question is asked where it can be: on
    instances whose arm_serial_ub DOES return a pose. If any accepted pose is
    one lemma_c calls blocked, then lemma_c is not necessary and using it as a
    hard prefilter would manufacture the very "not found" this task exists to
    remove -- so it may only ever be an ORDER, and an order changes which
    upper bound comes back (the frozen loop breaks on the first acceptance).

    The mechanism predicted in A2.3/D41 is the ARRIVAL TIME: lemma_c demands the
    park pose be clear of EVERY task-carrying stop of the lead, but the parking
    gantry does not arrive until T0, so a lead stop that closes before T0 never
    meets it.
    """
    gen = sched.gen_real if setname == 's1' else sa.gen_real_rotcrowded
    print(f'A2.3 PALANG / D41 -- is lemma_c NECESSARY? {setname} c_arm={c}\n')
    n_acc = n_viol = n_inst = 0
    for key, n, seed, mr in [(f'n{n}_s{s}_mr{m}', n, s, m)
                             for n in (4, 6) for s in range(10)
                             for m in (0, 1)][:limit]:
        inst = gen(n, seed, mr, (1, 2))
        geom = sa.ArmGeom(inst)
        ctx = af.ArmCtx(inst, geom, c, c_clear=0.0)
        sol0 = sched.solve_exact(inst)
        m_ub, stops = af.arm_serial_ub(inst, ctx, sol0, 0.0)
        if not np.isfinite(m_ub):
            print(f'  {key:12s} arm_serial_ub -> inf (nothing accepted)',
                  flush=True)
            continue
        n_inst += 1
        st0 = {g: sk._stops_with_depart(inst, g, sol0.stops[g])
               for g in inst.gantries}
        lc = af.lemma_c(inst, ctx, st0)
        # which gantry parked, and at which pose
        park = [g for g in inst.gantries
                if stops[g] and stops[g][0]['tasks'] == 0]
        lead = [g for g in inst.gantries if g not in park]
        if not park:
            print(f'  {key:12s} parked at p0 (no move) -- lemma_c vacuous here')
            continue
        prk, ld = park[0], lead[0]
        p = int(stops[prk][0]['pose'])
        # lemma_c's free set for the LEAD's stops, recomputed per pose
        P = inst.poses[prk]
        H = np.stack(ctx.geom.hang[prk])
        blocked_by = []
        for j, st in enumerate(st0[ld]):
            if not st['tasks']:
                continue
            cfg = ctx.geom.configs(ld, st['pose'], st['tasks'],
                                   st.get('assign'))
            d = np.inf
            for X in cfg:
                for k in (0, 1):
                    dd = polyline_min_dist(X[:, None], H[k][None, :][:, p:p+1])
                    dd = np.where(np.isnan(dd), np.inf, dd)
                    d = min(d, float(dd.min()))
            if d <= ctx.c_arm:
                blocked_by.append((j, st['start'], st['start'] + st['dur'],
                                   round(d, 6)))
        n_acc += 1
        T0 = float(stops[prk][0]['start'])
        if blocked_by:
            n_viol += 1
            after = [b for b in blocked_by if b[2] > T0 + 1e-9]
            print(f'  {key:12s} 🔴 pose {p} ACCEPTED by arm_serial_ub '
                  f'(makespan {m_ub:.4f}) but lemma_c calls it BLOCKED by '
                  f'{len(blocked_by)} lead stop(s); T0 = {T0:.4f}; '
                  f'{len(blocked_by) - len(after)} of them CLOSE BEFORE T0 '
                  f'-> {blocked_by[:2]}', flush=True)
        else:
            print(f'  {key:12s} pose {p} accepted, lemma_c agrees free '
                  f'(makespan {m_ub:.4f}, T0 {T0:.4f})', flush=True)
    print(f'\n  instances with an accepted park pose: {n_acc}')
    print(f'  of those, pose REJECTED by lemma_c:     {n_viol}')
    print('  -> lemma_c is ' + ('NOT NECESSARY: a hard prefilter would DROP '
                                'valid park poses (D41 CONFIRMED)' if n_viol
                                else 'necessary on every case measured here '
                                     '(D41 REFUTED on this sample)'))
    return 0


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['relabel']
    if a[0] == 'relabel':
        return relabel()
    if a[0] == 'necess':
        return necess(*(a[1:2] or ['s1']), *(float(x) for x in a[2:3]))
    if a[0] == 'park':
        return park(*(a[1:2] or ['s1']),
                    *(float(x) for x in a[2:3]),
                    **({'cap': int(a[3])} if len(a) > 3 else {}))
    if a[0] == 'grid':
        return grid()
    raise SystemExit(f'unknown: {a[0]}')


if __name__ == '__main__':
    raise SystemExit(main())
