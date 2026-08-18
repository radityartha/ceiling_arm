#!/usr/bin/env python3
"""Pick the LOCKED target pairs for p1_state.md 8c step 3: arm_1 + arm_2 together.

Offline: reads the torque-safe sets, no ROS, no hardware.

Step 2 picked one target from the REACHABLE set and G16 B5.1 measured what that
cost -- three of eight successes pulled more than joint_2's 14 N.m rating. So
step 3 draws from the TORQUE-SAFE sets instead (B6), one per arm:

    /tmp/torque_safe_arm_1.npy    628 nodes
    /tmp/torque_safe_arm_2.npy    445 nodes

and adds the constraint step 2 never had, because step 2 only ever moved one
arm: the two GOAL configurations must not put the arms inside each other. That
check cannot be delegated to MoveIt here -- 112 of the 121 geometry-bearing
t1_a1_* <-> t1_a2_* pairs are disabled in the SRDF with reason="Never" (see
scripts/interarm_collision.py), so MoveIt scores the two arms as unable to
touch. They are 0.800 m apart and reach 1.005 m each.

Spread uses the same farthest-point method as reachable_targets.py, so A2's
"different poses, not one pose N times" holds per arm, and the two selectors
agree by construction rather than by coincidence.

A pair that fails the collision screen is REPORTED and the next candidate is
tried. It is not quietly replaced with something easier: the count of rejected
pairs is itself a measurement of how contested the shared workspace is.

--plan-screen ADDS THE TRAJECTORY LEVEL, and docs/p1_g17_hw.md B1.8 is why it
had to exist. The torque-safe map is STATIC gravity, which g16 B6 already
called a NECESSARY but not sufficient condition. Measured: of ten pairs that
cleared the static screen, only THREE produced a plan for both arms that passed
the dynamic RNEA screen -- the static map over-predicts trajectory-level
feasibility by about 3x. Selecting on it alone locks a list that cannot reach
A2's 8/10 bar no matter what the hardware does.

So --plan-screen asks MoveIt for a real plan from the REST pose for every
candidate and runs the same RNEA screen the probe runs before executing. It
needs a live move_group (fake hardware is fine and is the right place to do
it). This changes the SELECTION CRITERION, not the bar: A2 stays 10 trials at
>= 8/10, and the list is still locked before any hardware data exists.

    python3 scripts/dual_arm_targets.py --pairs 10                 # statis
    python3 scripts/dual_arm_targets.py --pairs 10 --plan-screen   # + lintasan
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from interarm_collision import InterArmChecker, _ik


def spread(nodes, n):
    """Farthest-point sampling over the interior half -- reachable_targets.py's
    method, kept identical on purpose so the two lists are comparable."""
    depth = np.linalg.norm(nodes - nodes.mean(0), axis=1)
    interior = nodes[depth <= np.median(depth)]
    pick = [int(np.argmin(np.linalg.norm(interior - nodes.mean(0), axis=1)))]
    dmin = np.linalg.norm(interior - interior[pick[0]], axis=1)
    while len(pick) < min(n, len(interior)):
        j = int(np.argmax(dmin))
        pick.append(j)
        dmin = np.minimum(dmin, np.linalg.norm(interior - interior[j], axis=1))
    return interior[pick]


def plan_screen(cands, arm, other_arm, tau_max, repeats=3):
    """Keep candidates that plan within joint ratings `repeats` times out of
    `repeats` -- deliberately STRICTER than what the probe will do at run time.

    Planned from the arm's CURRENT state, which the caller must have put at
    REST -- that is the state every trial departs from (g16 B4), so screening
    from anywhere else would screen a motion nobody will command.

    WHY k-of-k, measured in docs/p1_g17_hw.md B1.9: a single pass is not
    evidence a target is feasible. The same pose from the same rest state came
    back PLANNED 8 times and TORQUE-UNSAFE 2 times out of 10, while other poses
    were stable 10/10 either way. Locking a list on one sample smuggles in the
    marginal poses, which then fail on the day at their marginal rate and get
    counted as method failures.

    Note the asymmetry, and it is intentional. Selection demands k-of-k (a
    target must be reliably feasible to earn a place on a locked list), while
    the probe at run time retries up to k times and takes the first plan that
    passes (a good target should not be lost to one bad sample). Both push the
    same way: separate the pose's feasibility from the planner's luck.

    plan_only throughout: this asks for plans and executes none of them.
    """
    import json
    import os

    import rclpy
    from geometry_msgs.msg import PoseStamped

    from reach_dwell_probe import Probe, move_to

    # Cache, because k-of-k costs k plans per candidate and the pool has to be
    # widened when too few survive. Farthest-point sampling is greedy and
    # therefore PREFIX-STABLE -- the first N picks of a larger pool are exactly
    # the picks of the smaller one -- so a wider re-run only has to test the
    # candidates it has not seen. Keyed by the parameters that change a verdict.
    cache_path = f'/tmp/g17_screen_{arm}.json'
    cache = {}
    if os.path.exists(cache_path):
        try:
            cache = json.load(open(cache_path))
        except ValueError:
            cache = {}

    def key(xyz):
        return f'{xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f}|{repeats}|{tau_max}'

    rclpy.init()
    node = Probe(arm, 0.0, tau_max, False, arms=[arm])
    kept, why = [], []
    try:
        for i, xyz in enumerate(cands, 1):
            if key(xyz) in cache:
                vs = cache[key(xyz)]
                ok = len(vs) == repeats and all(v == 'PLANNED' for v in vs)
                print(f'  {arm} kandidat {i:2d}/{len(cands)} '
                      f'({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}) -> '
                      f'{"/".join(vs)}  {"LOLOS" if ok else "ditolak"} (cache)')
                (kept if ok else why).append((xyz, vs[-1]))
                continue
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = \
                (float(v) for v in xyz)
            p.pose.orientation.x, p.pose.orientation.w = 1.0, 0.0
            # attempts=1 per repeat: each repeat must be an INDEPENDENT sample
            # of the planner, otherwise the retry logic would hide exactly the
            # variance this is here to measure.
            vs = []
            for _ in range(repeats):
                vs.append(move_to(arm, p, node, plan_only=True,
                                  tau_max=tau_max, other_arm=other_arm,
                                  attempts=1))
                if vs[-1] != 'PLANNED':
                    break                       # k-of-k: one miss is enough
            ok = len(vs) == repeats and all(v == 'PLANNED' for v in vs)
            print(f'  {arm} kandidat {i:2d}/{len(cands)} '
                  f'({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}) -> '
                  f'{"/".join(vs)}  {"LOLOS" if ok else "ditolak"}')
            (kept if ok else why).append((xyz, vs[-1]))
            cache[key(xyz)] = vs
            json.dump(cache, open(cache_path, 'w'))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return [x for x, _ in kept], why


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pairs', type=int, default=10,
                    help='A2 mengunci 10 percobaan; step 3 mewarisinya')
    ap.add_argument('--lin', type=float, default=0.550,
                    help='posisi rel TERUKUR (m)')
    ap.add_argument('--margin', type=float, default=0.05,
                    help='jarak minimum antar-lengan di pose tujuan (m)')
    ap.add_argument('--pool', type=int, default=40,
                    help='kandidat per lengan sebelum penyaringan tabrakan')
    ap.add_argument('--plan-screen', action='store_true',
                    help='B1.8: saring tiap kandidat dengan rencana MoveIt + '
                         'RNEA, bukan gravitasi statis. BUTUH move_group hidup '
                         '(perangkat keras palsu tempat yang benar). Lengan '
                         'HARUS di rest dulu: scripts/return_rest.py --move')
    ap.add_argument('--repeats', type=int, default=3,
                    help='B1.9: kandidat harus lolos k dari k sampel perencana '
                         'independen. Sekali lolos BUKAN bukti kelayakan.')
    ap.add_argument('--tau-max', type=float, default=12.0,
                    help='= nominal KA-75+. JANGAN dinaikkan agar lolos.')
    a = ap.parse_args()

    s1 = np.load('/tmp/torque_safe_arm_1.npy')
    s2 = np.load('/tmp/torque_safe_arm_2.npy')
    print(f'aman-torsi: arm_1 {len(s1)} node, arm_2 {len(s2)} node')

    c1 = spread(s1, a.pool)
    c2 = spread(s2, a.pool)

    if a.plan_screen:
        # arm_1 is planned with arm_2 at REST, and that is exactly the state it
        # departs from in a real trial. arm_2's plan runs with arm_1 already at
        # its paired target, which is pair-specific and therefore cannot be
        # screened here -- the probe screens it for real before executing.
        print(f'\n--- saringan LINTASAN {a.repeats}/{a.repeats}, arm_1 '
              f'({len(c1)} kandidat) ---')
        k1, r1 = plan_screen(c1, 'arm_1', 'arm_2', a.tau_max, a.repeats)
        print(f'\n--- saringan LINTASAN {a.repeats}/{a.repeats}, arm_2 '
              f'({len(c2)} kandidat) ---')
        k2, r2 = plan_screen(c2, 'arm_2', 'arm_1', a.tau_max, a.repeats)
        from collections import Counter
        print(f'\nlolos lintasan: arm_1 {len(k1)}/{len(c1)}, '
              f'arm_2 {len(k2)}/{len(c2)}')
        print(f'  arm_1 ditolak: {dict(Counter(m for _, m in r1))}')
        print(f'  arm_2 ditolak: {dict(Counter(m for _, m in r2))}')
        if len(k1) < a.pairs or len(k2) < a.pairs:
            print(f'🔴 kandidat lolos kurang dari {a.pairs}. Naikkan --pool.')
        c1 = np.array(k1) if k1 else c1[:0]
        c2 = np.array(k2) if k2 else c2[:0]

    chk = InterArmChecker(gantry='gantry_1', margin=a.margin)
    rest = {f't1_a1_joint_{i}': v for i, v in enumerate(
        [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885], 1)}
    rest.update({f't1_a2_joint_{i}': v for i, v in enumerate(
        [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885], 1)})
    rest['t1_linear_joint'] = a.lin
    q_rest = chk.q_from(rest)

    chosen, rejected, j = [], [], 0
    for p1 in c1:
        if len(chosen) >= a.pairs:
            break
        while j < len(c2):
            p2 = c2[j]
            j += 1
            q, ok1 = _ik(chk.model, chk.data, 't1_a1_', np.asarray(p1, float),
                         q_rest)
            q, ok2 = _ik(chk.model, chk.data, 't1_a2_', np.asarray(p2, float), q)
            if not (ok1 and ok2):
                rejected.append((p1, p2, 'IK-GAGAL', float('nan')))
                continue
            d, pair = chk.check(q)
            if d < a.margin:
                rejected.append((p1, p2, 'TABRAKAN' if d <= 0 else 'MARGIN', d))
                continue
            chosen.append((p1, p2, d, np.linalg.norm(p1 - p2)))
            break

    print(f'\npasangan DITOLAK penyaring tabrakan-tujuan: {len(rejected)}')
    for p1, p2, why, d in rejected:
        print(f'   {why:9s} d={d * 1000:7.1f} mm  '
              f'a1({p1[0]:.3f},{p1[1]:.3f},{p1[2]:.3f}) '
              f'a2({p2[0]:.3f},{p2[1]:.3f},{p2[2]:.3f})')

    if len(chosen) < a.pairs:
        print(f'\n🔴 hanya {len(chosen)} dari {a.pairs} pasangan lolos. '
              'Naikkan --pool, JANGAN turunkan --margin.')

    print(f'\n{len(chosen)} PASANGAN TERKUNCI '
          f'(jarak antar-lengan di tujuan, margin {a.margin * 100:.0f} cm)\n')
    print('# SALIN APA ADANYA. Urutan DIKUNCI SEBELUM data dilihat.')
    print('#  n | arm_1 target        | arm_2 target        | d_lengan | d_titik')
    for i, (p1, p2, d, dp) in enumerate(chosen, 1):
        print(f'# {i:2d} | {p1[0]:.3f},{p1[1]:.3f},{p1[2]:.3f} | '
              f'{p2[0]:.3f},{p2[1]:.3f},{p2[2]:.3f} | '
              f'{d * 1000:6.1f} mm | {dp * 1000:6.1f} mm')
    print()
    for i, (p1, p2, _, _) in enumerate(chosen, 1):
        print('python3 scripts/reach_dwell_probe.py --dual --approach 0 '
              '--trials 1 --target %.3f,%.3f,%.3f --target2 %.3f,%.3f,%.3f'
              '   # pasangan %d' % (*p1, *p2, i))

    if chosen:
        np.save('/tmp/g17_pairs.npy',
                np.array([np.concatenate([p1, p2]) for p1, p2, _, _ in chosen]))
        print('\n  -> /tmp/g17_pairs.npy')
    return 0 if len(chosen) == a.pairs else 1


if __name__ == '__main__':
    sys.exit(main())
