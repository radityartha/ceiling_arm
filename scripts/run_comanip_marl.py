#!/usr/bin/env python3
"""Greedy-energy vs auction-coalition on the surrogate workcell (poster Fig. 2).

The claim under test is NOT "an allocator without a coalition operator fails to
form coalitions" -- that would be true by construction and worth nothing. It is
the sharper one:

    energy-greedy single-arm allocation is ATTRACTED to the heavy block (a block
    handle is frequently the lowest-J action available to some arm), commits one
    arm to it, and cannot lift it, because 1 x 0.5 kg payload < 1.8 kg. It then
    burns that energy again on the next arm. The task is never completed no
    matter the ordering, and the energy spent failing is measured.

So both policies are given the SAME action space and the SAME cost function; the
only difference is whether an allocation may bind four arms to one item at once.

Policies
    greedy-energy      : the allocator in the codebase today
                         (gantry_reach_executor: one arm per object, min J,
                         falling through to the next candidate on failure).
    auction-coalition  : same per-item bidding, but a >1-arm item bids as a
                         JOINT plan over all four arms with per-gantry
                         consistent poses. Cheapest bid wins each round, so the
                         expensive coalition is naturally served last -- it is
                         not given priority to make it look good.

    python3 scripts/run_comanip_marl.py --seeds 30 --out data/marl
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from comanip_env import ARMS, WorkcellEnv          # noqa: E402


def greedy_energy(env, log=None):
    """One arm per item, lowest J first -- the allocator that exists today.

    Mirrors gantry_reach_executor._do_pick: score candidates by J, take the
    best, and on failure fall through to the next candidate. An item is
    abandoned once every arm has failed on it (the executor's max_attempts
    exhaustion), which is what stops this from looping forever.
    """
    tried = {}                                   # item name -> arms that failed
    while env.pending() and not env.truncated():
        bids = []
        for it in env.pending():
            for a in ARMS:
                if a in tried.get(it.name, set()):
                    continue
                pl = env.single_plan(it, a, env.state)
                if pl is not None:
                    bids.append(pl)
        if not bids:
            break
        pl = min(bids, key=lambda b: b['J'])
        it = pl['item']
        if it.n_arms == 1:
            env.apply(pl)
            if log is not None:
                log.append(('grasp', it.name, pl['arm'], pl['J'], True))
        else:
            # Reached the handle, but one arm cannot carry the load. The travel
            # was spent; the lift fails. This is the poster's failure mode.
            env.charge_failed(pl, arms_committed=1)
            tried.setdefault(it.name, set()).add(pl['arm'])
            if log is not None:
                log.append(('payload-fail', it.name, pl['arm'], pl['J'], False))
    return env.state


def auction_coalition(env, log=None):
    """Per-item bids, where a multi-arm item bids as one joint 4-arm plan.

    Cheapest bid wins each round. No preference is given to coalitions: because
    the coalition bid is the most expensive single action, it is served LAST.
    """
    stuck = set()
    while env.pending() and not env.truncated():
        bids = []
        for it in env.pending():
            if it.name in stuck:
                continue
            if it.n_arms == 1:
                cand = [env.single_plan(it, a, env.state) for a in ARMS]
                cand = [c for c in cand if c is not None]
                if cand:
                    bids.append(min(cand, key=lambda c: c['J']))
            else:
                pl = env.coalition_plan(it, env.state)
                if pl is not None:
                    bids.append(pl)
                else:
                    stuck.add(it.name)
        if not bids:
            break
        pl = min(bids, key=lambda b: b['J'])
        env.apply(pl)
        if log is not None:
            who = ('coalition' if pl.get('kind') == 'coalition' else pl['arm'])
            log.append(('grasp', pl['item'].name, who, pl['J'], True))
    return env.state


POLICIES = {'greedy-energy': greedy_energy, 'auction-coalition': auction_coalition}


def block_attraction(env):
    """How often a BLOCK handle is an arm's own lowest-J action, at the home state.

    This is what stops the greedy result from being a tautology: greedy does not
    skip the block for lack of an operator, it is drawn to it and then cannot
    lift it. Returns (n_arms_preferring_block, 4).
    """
    st = env.zero_state()
    block = [it for it in env.items if it.n_arms > 1][0]
    lights = [it for it in env.items if it.n_arms == 1]
    n = 0
    for a in ARMS:
        pb = env.single_plan(block, a, st)            # best over all 4 corners
        pl = [p for p in (env.single_plan(it, a, st) for it in lights) if p]
        bj = pb['J'] if pb else None
        lj = min((p['J'] for p in pl), default=None)
        if bj is not None and (lj is None or bj < lj):
            n += 1
    return n, len(ARMS)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--maps', default='data/maps')
    ap.add_argument('--out', default='data/marl')
    ap.add_argument('--seeds', type=int, default=30)
    ap.add_argument('--n-light', type=int, default=5)
    ap.add_argument('--light-mass', type=float, default=0.3)
    ap.add_argument('--block-mass', type=float, default=1.8)
    ap.add_argument('--block-size', type=float, default=0.25)
    ap.add_argument('--z', type=float, default=1.30)
    ap.add_argument('--max-rounds', type=int, default=12)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows, traces = [], []
    for seed in range(args.seeds):
        attraction = None
        for pname, fn in POLICIES.items():
            env = WorkcellEnv(maps_dir=args.maps, max_rounds=args.max_rounds)
            env.reset(seed=seed, n_light=args.n_light, light_mass=args.light_mass,
                      block_mass=args.block_mass, block_size=args.block_size,
                      z=args.z)
            if attraction is None:
                attraction = block_attraction(env)
            log = []
            st = fn(env, log)
            block_i = [i for i, it in enumerate(env.items) if it.n_arms > 1][0]
            rows.append(dict(
                seed=seed, policy=pname,
                items_done=int(st['done'].sum()), n_items=len(env.items),
                block_done=int(st['done'][block_i]),
                energy=round(st['energy'], 4),
                wasted=round(st['wasted'], 4),
                failed=st['failed'],
                coalition_peak=st['coalition_peak'],
                rounds=st['round'],
                # The headline honest metric. Greedy's TOTAL J is lower simply
                # because it delivers fewer items, so total energy alone would
                # flatter it; energy per DELIVERED item is the comparison that
                # is not confounded by the unfinished task.
                j_per_item=round(st['energy'] / max(int(st['done'].sum()), 1), 4),
                arms_preferring_block=attraction[0]))
            for k, (kind, item, who, j, ok) in enumerate(log):
                traces.append(dict(seed=seed, policy=pname, step=k, kind=kind,
                                   item=item, who=who, J=round(j, 4), ok=int(ok)))
        print(f'seed {seed:2d}: ' + '  |  '.join(
            f"{r['policy']}: {r['items_done']}/{r['n_items']} items, "
            f"block={'YES' if r['block_done'] else 'no '}, J={r['energy']:7.1f} "
            f"(wasted {r['wasted']:6.1f}), peak={r['coalition_peak']}"
            for r in rows[-len(POLICIES):]))

    for name, data in (('summary.csv', rows), ('traces.csv', traces)):
        path = os.path.join(args.out, name)
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
            w.writeheader()
            w.writerows(data)
        print('wrote', path)

    print('\n=== aggregate over', args.seeds, 'seeds ===')
    for pname in POLICIES:
        r = [x for x in rows if x['policy'] == pname]
        f = lambda k: np.array([x[k] for x in r], float)   # noqa: E731
        print(f'{pname:20s} block completed {100 * f("block_done").mean():5.1f}% | '
              f'items {f("items_done").mean():.2f}/{r[0]["n_items"]} | '
              f'J {f("energy").mean():7.1f}+-{f("energy").std():5.1f} | '
              f'J/item {f("j_per_item").mean():6.2f} | '
              f'wasted {f("wasted").mean():6.1f} '
              f'({100 * f("wasted").sum() / f("energy").sum():4.1f}%) | '
              f'peak arms on block {f("coalition_peak").mean():.2f} | '
              f'failed lifts {f("failed").mean():.2f}')
    pref = np.array([x['arms_preferring_block'] for x in rows], float).mean()
    print(f'\nnon-tautology check: on average {pref:.2f}/4 arms rank a BLOCK handle '
          f'as their own lowest-J action,\nso greedy is drawn to the block rather '
          f'than ignoring it -- and still never lifts it.')


if __name__ == '__main__':
    main()
