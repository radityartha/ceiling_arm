"""G25 A3 gate G0' -- the NEW cost paths of sched.py against an INDEPENDENT brute force.

Reference (no sched.py code on the cost path; sched is used only to BUILD random
instances, and verify_sched_exact.ref_stop_slots -- itself independent of sched --
for the old arm-parallel stop):
  move(a -> b)  0 if a == b, else max(T_lin', T_rot) + (t_fold_first if this is the
                gantry's FIRST move and no stop has happened yet, else t_fold)
                T_lin' = 0.29 + d/v            (rail_cmd = 0)
                       = rail_cmd * max(3, pi d / (2 * 0.9 * v))   (rail_cmd > 0)
  stop(U, p)    serial: (#tasks) * dwell, iff every SR task has an arm reaching it at p
                and every MR task is a handover at p;  parallel: ref_stop_slots * dwell
  gantry cost   exhaustive ordered partitions x any pose per stop (revisits included)
  makespan      max over gantries, every task -> gantry split

Also exposes eval_schedule(inst, stops) -- an independent replay of a reported
schedule under the same rules, used on the 45 G24 instances.

    python3 g0_gate.py
"""
import itertools
import math
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '../../../ros2_ws/src/reachability_gng')
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'test'))
from reachability_gng import sched  # noqa: E402  (instance building + solver under test)
from verify_sched_exact import ref_stop_slots  # noqa: E402

V = 3000.0 / 95.4930


def ref_move(inst, g, a, b, first):
    if a == b:
        return 0.0
    pa, pb = inst.poses[g][a], inst.poses[g][b]
    d = abs(pa[0] - pb[0]) * 1000.0
    r = abs(math.degrees(pa[1]) - math.degrees(pb[1]))
    if r > 180.0:
        r = 360.0 - r
    if d > 1e-6:
        tl = inst.rail_cmd * max(3.0, math.pi * d / (2 * 0.9 * V)) if inst.rail_cmd \
            else 0.29 + d / V
    else:
        tl = 0.0
    tr = 0.26 + r / 10.0 if r > 1e-6 else 0.0
    fold = inst.t_fold_first if (first and inst.t_fold_first is not None) else inst.t_fold
    return max(tl, tr) + fold


def ref_stop(inst, g, tasks, p):
    if not inst.serial_arms:
        s = ref_stop_slots(inst, g, list(tasks), p)
        return None if s is None else s * inst.dwell
    for i in tasks:
        if inst.kind[i] == 'MR':
            if not inst.hand[g][i, p]:
                return None
        elif not (inst.reach[g][i, p, 0] or inst.reach[g][i, p, 1]):
            return None
    return len(tasks) * inst.dwell


def ref_gantry(inst, g, tasks):
    best = [math.inf]
    P = len(inst.poses[g])

    def rec(rem, cur, acc, first):
        if acc >= best[0] - 1e-9:
            return
        if not rem:
            best[0] = acc
            return
        rs = sorted(rem)
        for k in range(1, len(rs) + 1):
            for blk in itertools.combinations(rs, k):
                for p in range(P):
                    d = ref_stop(inst, g, blk, p)
                    if d is None:
                        continue
                    rec(rem - set(blk), p, acc + ref_move(inst, g, cur, p, first) + d, False)

    rec(set(tasks), inst.p0[g], 0.0, True)
    return best[0]


def ref_makespan(inst):
    gs = inst.gantries
    best = math.inf
    for combo in itertools.product(gs, repeat=inst.n):
        best = min(best, max(ref_gantry(inst, g, [i for i in range(inst.n) if combo[i] == g])
                             for g in gs))
    return best


def eval_schedule(inst, stops):
    """Independent replay of solver stops {g: [dict(pose, tasks bitmask)]} -> {g: finish}."""
    out = {}
    for g, st in stops.items():
        cur, t, first = inst.p0[g], 0.0, True
        for s in st:
            tasks = [i for i in range(inst.n) if s['tasks'] >> i & 1]
            t += ref_move(inst, g, cur, s['pose'], first)
            d = ref_stop(inst, g, tasks, s['pose'])
            assert d is not None, (g, s)
            t += d
            cur, first = s['pose'], False
        out[g] = t
    return out


def main():
    combos = list(itertools.product((False, True), (None, 4.9), (0.0, 0.95)))
    n_ok = n_all = 0
    fails = []
    for serial, tff, rc in combos:
        for seed in range(12):
            rng = np.random.default_rng(1000 + seed)
            n = int(rng.integers(3, 6))
            P = int(rng.integers(3, 6))
            gs = (1,) if seed % 2 == 0 else (1, 2)
            mr = int(seed % 3 == 2)
            base = sched.gen_random_small(n, P, seed, n_mr=mr, gantries=gs)
            for dwell, tf in ((2.0, 0.0), (42.3, 55.8)):
                inst = replace(base, dwell=dwell, t_fold=tf, serial_arms=serial,
                               t_fold_first=tff, rail_cmd=rc)
                n_all += 1
                sol = sched.solve_exact(inst)
                ref = ref_makespan(inst)
                rep = eval_schedule(inst, sol.stops)
                ok = abs(sol.makespan - ref) < 1e-9 and \
                    all(abs(rep[g] - sol.finish[g]) < 1e-9 for g in gs) and \
                    abs(max(rep.values()) - sol.makespan) < 1e-9
                n_ok += ok
                if not ok:
                    fails.append((serial, tff, rc, seed, dwell, tf, sol.makespan, ref, rep, sol.finish))
    for f in fails:
        print('FAIL', f)
    print(f"G0': {n_ok}/{n_all} solve_exact == brute force AND replay == solver finish "
          f"({len(combos)} cost combos x 12 instances x 2 (dwell, t_fold))")
    return not fails


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
