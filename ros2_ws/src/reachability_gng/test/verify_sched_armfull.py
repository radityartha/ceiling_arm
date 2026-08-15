#!/usr/bin/env python3
"""G12 gate: M0-M5. docs/p1_g12_armsolve.md A3.

No Delta_full number may be spoken before these pass.

  M0  TRANSPARENCY  solve_armfull with the arm test OFF (c_arm = -inf) must
                    reproduce solve_coupled2 EXACTLY -- makespan, route, proved,
                    node count. This is what carries G10's bar (W2, W2b(i),
                    P0-P4, the 17-mutation gate) across the fork A2.1 did not
                    foresee: equivalence CHECKED, not asserted.
  M1  SOUNDNESS     every schedule the arm solver returns passes BOTH
                    validate_coupled and sched_arm.arm_schedule_conflict -- the
                    slow independent walk, which shares no line with the fast
                    path in sched_armfull.
  M2  L6            (a) Lemma B's delta re-measured on all 2376^2 at every
                    sweep c_arm;  (b) fast path vs slow walk on random
                    schedules, 0 verdict differences.
  M3  W3            arm-aware brute force on small instances; 0 solver > W3.
  M4  LEMMA 3       ub >= lb, 0 violations.
  M5  REGRESSION    Delta_struct at c_clear = 0 reproduces p1_g11 B6.2.

Run:  python3 test/verify_sched_armfull.py all | m0 | m1 | m2 | m3 | m4 | m5
"""

from __future__ import annotations

import itertools
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
from gate_sched_coupled import validate_coupled             # noqa: E402
from verify_sched_coupled import s5_instances               # noqa: E402
from reachability_gng.capability import CapabilityMap       # noqa: E402

C_SWEEP = (0.00, 0.05, 0.10, 0.15, 0.20)
C_MAIN = 0.05
MAPS = ('/tmp/cap_g1_rail160.npz', '/tmp/cap_g2_rail160.npz')


def _s1(n_list=(4, 6), seeds=range(10), mrs=(0, 1)):
    for n in n_list:
        for seed in seeds:
            for mr in mrs:
                yield f'n{n}_s{seed}_mr{mr}', sched.gen_real(n, seed, mr, (1, 2))


def gen_small_armreal(n, n_pose, seed, n_mr=0, x_c=0.80, band=0.06,
                      maps=MAPS, max_reject=200_000):
    """W3 fuel: small enough to enumerate, REAL enough to have arm geometry.

    The W2 fuel (`verify_sched_coupled.gen_small_crowded`) cannot be used here.
    It fabricates poses and reach oracles, so it has no `meta['nodes']` and
    `ArmGeom` has nothing to read canonical arm configurations FROM -- the arm
    predicate is defined against the capability maps' `canon` index, not against
    an arbitrary Instance. Building W3 on fabricated geometry would be measuring
    a predicate that does not exist.

    Same SHAPE as gen_small_crowded, on the real map instead:
      pose 0      nearest (x_c, rot = 0) -- structurally safe (Lemma 1), p0 for
                  both gantries, so the start state is collision-free
      poses 1..   real grid poses at |lin - x_c| <= band with |rot| near 90 deg,
                  i.e. inside the structural blocking band AND where the arms
                  sweep across the 0.72 m gantry separation
      tasks       real nodes feasible in that set and NOT at p0, so both
                  gantries are forced off the safe pose (the rejection clause
                  gen_real_rotcrowded measured the need for)
    """
    caps = {g: CapabilityMap.load(maps[g - 1]) for g in (1, 2)}
    ref = caps[1]
    poses = np.stack(np.meshgrid(ref.lin, ref.rot, indexing='ij'),
                     -1).reshape(-1, 2)
    p0 = int(np.argmin(np.hypot(poses[:, 0] - x_c, poses[:, 1])))
    hot = np.flatnonzero((np.abs(poses[:, 0] - x_c) <= band)
                         & (np.abs(np.abs(poses[:, 1]) - np.pi / 2)
                            <= np.deg2rad(12.0)))
    rng = np.random.default_rng(7000 + seed)
    keep = np.sort(np.append(rng.choice(hot, n_pose - 1, replace=False), p0))
    p0_idx = int(np.searchsorted(keep, p0))

    pool = np.flatnonzero(np.abs(ref.nodes[:, 0] - x_c) <= 0.30)
    kinds = np.array(['MR'] * n_mr + ['SR'] * (n - n_mr), dtype='<U2')
    chosen, rejects = [], 0
    while len(chosen) < n:
        cand = int(pool[rng.integers(0, len(pool))])
        want_mr = kinds[len(chosen)] == 'MR'
        ok = at_p0 = False
        for g in (1, 2):
            r, _, hd = sched._gantry_oracles(caps[g], np.array([cand]))
            m = hd[0][keep] if want_mr else r[0][keep].any(axis=1)
            ok |= bool(m.any())
            at_p0 |= bool(m[p0_idx])
        if ok and not at_p0 and cand not in chosen:
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > max_reject:
                return None
    node_idx = np.array(chosen)
    reach, zone, hand = {}, {}, {}
    for g in (1, 2):
        r, z, h = sched._gantry_oracles(caps[g], node_idx)
        reach[g], zone[g], hand[g] = r[:, keep], z[:, keep], h[:, keep]
    return sched.Instance(
        kinds, {g: poses[keep] for g in (1, 2)}, reach, zone, hand,
        {g: p0_idx for g in (1, 2)}, ref.nodes[node_idx], sched.DWELL, 0.0,
        f'armreal(n={n},P={n_pose},seed={seed})',
        dict(nodes=node_idx.tolist(), pose_keep=keep.tolist()))


def w3_fuel(limit=24):
    out = []
    for n, P, mr in [(2, 3, 0), (2, 4, 0), (3, 3, 0), (2, 3, 1), (3, 4, 0)]:
        for seed in range(6):
            if len(out) >= limit:
                return out
            inst = gen_small_armreal(n, P, seed, n_mr=mr)
            if inst is not None:
                out.append((f'armreal_n{n}P{P}mr{mr}s{seed}', inst))
    return out


# ---------------------------------------------------------------- M0
def m0(n_inst=None, budget=120.0):
    """The fork is equivalent to the frozen solver when the arms are off."""
    bad, rows = [], 0
    cases = list(_s1())
    if n_inst:
        cases = cases[:n_inst]
    for key, inst in cases:
        ref = sk.solve_coupled2(inst, c_clear=0.0, time_budget=budget)
        got, ctx = af.solve_armfull(inst, c_arm=-np.inf, c_clear=0.0,
                                    time_budget=budget)
        rows += 1
        same = (abs(ref.makespan - got.makespan) <= 1e-9
                and ref.route == got.route and ref.proved == got.proved
                and ref.n_nodes == got.n_nodes)
        if not same:
            bad.append(f'{key}: ref({ref.makespan:.6f},{ref.route},'
                       f'{ref.proved},{ref.n_nodes}) != '
                       f'got({got.makespan:.6f},{got.route},{got.proved},'
                       f'{got.n_nodes})')
        if ctx.calls:
            bad.append(f'{key}: arm predicate evaluated {ctx.calls}x with '
                       'c_arm = -inf')
    for e in bad[:8]:
        print('   MISMATCH', e)
    print(f'M0 transparency on {rows} S1 instances: '
          f'{rows - len({e.split(":")[0] for e in bad})} identical')
    return not bad


def m0_w2(budget=45.0):
    """Same equivalence on the W2 fuel -- the small crowded instances where the
    search machinery actually runs (p1_g10 K4 S5)."""
    bad = n = 0
    for i, (key, inst) in enumerate(s5_instances()):
        ref = sk.solve_coupled2(inst, c_clear=0.0, time_budget=budget)
        got, _ = af.solve_armfull(inst, c_arm=-np.inf, c_clear=0.0,
                                  time_budget=budget)
        n += 1
        if not (abs(ref.makespan - got.makespan) <= 1e-9
                and ref.route == got.route and ref.proved == got.proved
                and ref.n_nodes == got.n_nodes):
            bad += 1
            if bad <= 5:
                print(f'   MISMATCH W2[{key}]: {ref.makespan:.6f}/{ref.route}/'
                      f'{ref.n_nodes} != {got.makespan:.6f}/{got.route}/'
                      f'{got.n_nodes}')
    print(f'M0 transparency on {n} W2 fuel instances: {n - bad} identical')
    return bad == 0


# ---------------------------------------------------------------- M2
def m2a(cs=C_SWEEP):
    """Lemma B: HANG_BLOCK(c) subset BLOCK(c + delta) on all 2376^2.

    A2.2 forbids using Lemma B at any c_arm whose delta was not re-measured, so
    this is a gate and not a repeat of p1_g11 L2 for its own sake.
    """
    inst = sched.gen_real(4, 0, 0, (1, 2))
    geom = sa.ArmGeom(inst)
    P = inst.poses[1]
    ok = True
    print(f'{"c_arm":>7} | {"HANG_BLOCK":>12} | {"BLOCK(c+delta)":>15} | delta')
    for c in cs:
        hb = sa.hang_block_poses(geom, 1, 2, P, c)
        nh = int(hb.sum())
        found = None
        for delta in (0.0, 0.002, 0.004, 0.006, 0.010, 0.020):
            blk = _block_grid(P, c + delta)
            if not (hb & ~blk).any():
                found = delta
                break
        good = found is not None and found <= af.DELTA_LEMMA_B
        ok &= good
        print(f'{c:7.2f} | {nh:12d} | {int(blk.sum()):15d} | '
              f'{"none" if found is None else f"{found:.3f}"} '
              f'{"OK" if good else "EXCEEDS DELTA_LEMMA_B"}')
    print(f'M2a Lemma B: {"PASS" if ok else "FAIL"}')
    return ok


def _block_grid(P, c, chunk=256):
    out = np.zeros((len(P), len(P)), bool)
    for s in range(0, len(P), chunk):
        a = P[s:s + chunk]
        m = sc.may_block(a[:, 0][:, None], a[:, 1][:, None], P[None, :, 0],
                         P[None, :, 1], c)
        if not m.any():
            continue
        ii, jj = np.nonzero(m)
        d = sc.pair_distance(a[ii, 0], a[ii, 1], P[jj, 0], P[jj, 1])
        blk = np.zeros(m.shape, bool)
        blk[ii, jj] = d <= c
        out[s:s + chunk] = blk
    return out


def m2b(n_sched=2000, seed=0, c=C_MAIN):
    """Fast decomposed check vs the slow independent walk, on real schedules.

    The schedules are MUTATED optima, not random noise: an optimum that already
    satisfies both predicates tests nothing, so each is perturbed until the two
    checkers have something to disagree about.
    """
    rng = np.random.default_rng(seed)
    diff, n, hit = [], 0, 0
    for key, inst in itertools.islice(_s1(), 8):
        geom = sa.ArmGeom(inst)
        s = sk.solve_coupled2(inst, c_clear=0.0, time_budget=30.0)
        if not s.stops:
            continue
        base = {g: sk._stops_with_depart(inst, g, s.stops[g])
                for g in inst.gantries}
        for _ in range(n_sched // 8):
            st = {g: [dict(x) for x in base[g]] for g in base}
            g = int(rng.choice(inst.gantries))
            if not st[g]:
                continue
            # Repose one stop, then rebuild that gantry's whole timeline
            # sequentially with a random extra wait. Perturbing depart/start
            # directly produced trajectories with OVERLAPPING LEGS, which
            # sched_coll.Traj rejects outright -- a mutation that cannot exist
            # tests nothing, so the mutation is made physical instead.
            j = int(rng.integers(0, len(st[g])))
            st[g][j]['pose'] = int(rng.integers(0, len(inst.poses[g])))
            t, cur = 0.0, int(inst.p0[g])
            for x in st[g]:
                T = sc.leg_duration(tuple(inst.poses[g][cur]),
                                    tuple(inst.poses[g][x['pose']]))
                x['depart'] = t + float(rng.uniform(0.0, 1.5))
                x['start'] = x['depart'] + T
                t = x['start'] + x['dur']
                cur = x['pose']
            slow = sa.arm_schedule_conflict(inst, geom, st, c)
            # lemma_b=False is the mode the solver actually runs in
            # (contradiction 2). With Lemma B ON the fast path deliberately
            # skips hanging x hanging and would differ BY DESIGN, so comparing
            # that mode here would be a gate that cannot fire.
            fast = af.schedule_arm_conflict(inst, geom, st, c, lemma_b=False)
            n += 1
            hit += slow is not None
            if (slow is None) != (fast is None):
                diff.append((key, j, slow, fast))
    for d in diff[:8]:
        print('   DISAGREE', d)
    print(f'M2b fast vs slow on {n} mutated schedules ({hit} with a real '
          f'violation): {len(diff)} verdict differences')
    return not diff


# ---------------------------------------------------------------- M3
def brute_arm(inst, geom, c_arm, c_clear, horizon=None, step=0.25,
              max_states=400_000):
    """W3: arm-aware brute force. Shares NO line with the search.

    Enumerates every interleaving of (gantry, task subset, pose) assignments on
    a discretised start-time grid and checks each complete schedule with the
    SLOW gate (sched_arm.arm_schedule_conflict) plus the structural one. Only
    usable on gen_random_small (n <= 3, |P| <= 4), which is the point: W2 is
    built the same way (p1_g10 K1).
    """
    gs = inst.gantries
    n, full = inst.n, (1 << inst.n) - 1
    P = {g: len(inst.poses[g]) for g in gs}
    best = np.inf
    seen = 0

    def rec(R, stops, tmin):
        nonlocal best, seen
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
            best = min(best, m)
            return
        for g in gs:
            U = R
            while U:
                for p in range(P[g]):
                    dur, assign = sched.stop_duration(inst, g, U, p)
                    if not np.isfinite(dur):
                        U = (U - 1) & R
                        break
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
                        stops[g].append(dict(pose=p, depart=dep,
                                             start=dep + T, dur=dur, tasks=U,
                                             assign=assign))
                        rec(R ^ U, stops, max(tmin, dep + T + dur))
                        stops[g].pop()
                else:
                    U = (U - 1) & R
                    continue
        return

    rec(full, {g: [] for g in gs}, 0.0)
    return best


def m3(n_inst=24, c=C_MAIN, budget=45.0):
    worse, binds, n, rows = [], 0, 0, []
    for i, (key, inst) in enumerate(w3_fuel(n_inst)):
        if n >= n_inst:
            break
        geom = sa.ArmGeom(inst)
        cc = 0.0                       # contradiction 2: Lemma B not used
        t0 = time.time()
        w3 = brute_arm(inst, geom, c, cc)
        if not np.isfinite(w3):
            continue
        got, _ = af.solve_armfull(inst, c_arm=c, geom=geom, time_budget=budget)
        n += 1
        w3_struct = brute_arm(inst, geom, np.inf, cc)
        b = w3 > w3_struct + 1e-9
        binds += b
        rows.append((i, w3_struct, w3, got.makespan, b))
        if got.makespan > w3 + 1e-6:
            worse.append((i, got.makespan, w3))
        if time.time() - t0 > 90:
            break
    for w in worse[:8]:
        print(f'   SOLVER > W3: inst {w[0]} solver {w[1]:.4f} > brute '
              f'{w[2]:.4f}')
    print(f'M3 W3: {n} instances, {binds} where the ARM constraint binds, '
          f'{len(worse)} times solver > W3')
    ok = not worse and binds >= 10
    if binds < 10:
        print(f'   🔴 only {binds} binding instances, A3-M3 asks for >= 10 '
              '-- REPORTED, not smoothed')
    return ok


# ---------------------------------------------------------------- M1/M4/M5
def m145(c=C_MAIN, budget=120.0):
    gate_fail, lemma3, rows = [], [], []
    for key, inst in _s1():
        geom = sa.ArmGeom(inst)
        s, ctx = af.solve_armfull(inst, c_arm=c, geom=geom, time_budget=budget)
        rows.append((key, s))
        if s.stops:
            errs = validate_coupled(inst, s.stops, s.finish, s.makespan)
            slow = sa.arm_schedule_conflict(inst, geom, s.stops, c)
            if errs:
                gate_fail.append((key, 'wait-aware', errs[:2]))
            if slow is not None:
                gate_fail.append((key, 'arm-slow', slow))
        if np.isfinite(s.makespan) and s.makespan < s.lb - 1e-9:
            lemma3.append((key, s.makespan, s.lb))
    for e in gate_fail[:8]:
        print('   GATE FAIL', e)
    for e in lemma3[:8]:
        print('   LEMMA 3 VIOLATION', e)
    print(f'M1 soundness: {len(rows) - len({e[0] for e in gate_fail})}/'
          f'{len(rows)} schedules pass BOTH gates')
    print(f'M4 Lemma 3: {len(lemma3)} violations')
    return not gate_fail and not lemma3


def m5(budget=120.0):
    """Delta_struct at c_clear = 0 must reproduce p1_g11 B6.2."""
    routes, exact, deltas = {}, 0, []
    for key, inst in _s1():
        s = sk.solve_coupled2(inst, c_clear=0.0, time_budget=budget)
        routes[s.route] = routes.get(s.route, 0) + 1
        ex = (s.route in ('lemma4', 'single', 'dive-lb')
              or s.makespan <= s.lb + 1e-9 or s.proved)
        exact += ex
        deltas.append(100.0 * (s.makespan - s.lb) / s.lb)
    want = {'lemma4': 25, 'dive-lb': 5, 'bnb': 10}
    ok = routes == want and exact == 39
    print(f'M5 regression vs p1_g11 B6.2: routes {routes} (want {want}), '
          f'exact {exact}/40 (want 39), mean Delta% hi {np.mean(deltas):.4f} '
          f'(want <= 0.4679)  -> {"PASS" if ok else "MISMATCH -- A FINDING"}')
    return ok


def main(argv=None):
    a = (argv or sys.argv[1:]) or ['all']
    tests = {'m0': lambda: m0() and m0_w2(), 'm2': lambda: m2a() and m2b(),
             'm3': m3, 'm1': m145, 'm5': m5}
    names = list(tests) if a[0] == 'all' else a
    res = {}
    for k in names:
        print(f'\n--- {k} ---')
        res[k] = bool(tests[k]())
    print('\n' + '=' * 66)
    for k, v in res.items():
        print(f'  {k.upper():4s} {"PASS" if v else "FAIL"}')
    print('=' * 66)
    print('A3 GATE: ' + ('M0-M5 PASS -- Delta_full may be spoken'
                         if all(res.values()) else
                         'FAILED -- Delta_full stays a bracket'))
    return 0 if all(res.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
