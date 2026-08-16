#!/usr/bin/env python3
"""Pick a target for docs/p1_g16_hw.md stage 4 that is KNOWN reachable.

Offline: reads the capability map, no ROS, no hardware. Exists so trial 1 does
not come back NO-PLAN because the target was made up -- which is what the
`--target 0.9,0.3,1.2` placeholder in the runbook would have done.

Reachability depends on where the gantry actually IS, so pass the measured rail
position. Read it from the live system rather than assuming:

    ros2 topic echo /joint_states --once | grep -A30 t1_linear

The gantry does not move during stage 4, so one rail position is all that
matters. Poses are scored at the TIGHTEST tolerance the map carries (5 cm, L1)
-- that is the layer that says "a solution exists near here", NOT an accuracy
claim; execution accuracy (L2, < 5 mm) is what the session measures.

    python3 scripts/reachable_targets.py --lin 0.550                # inspeksi
    python3 scripts/reachable_targets.py --lin 0.550 --spread 10    # 10 titik uji

`--spread N` is the one to use for stage 4: it returns N targets SPREAD across
the reachable set, one per trial. Plain depth ranking does not -- its top hits
are grid neighbours ~7 cm apart, which is one pose measured N times, and that
would report a tracking property as if it were a workspace property.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'ros2_ws' / 'src' / 'reachability_gng'))

from reachability_gng.capability import (           # noqa: E402
    CapabilityMap, GANTRY_ARM, PARTNER)

# arm_1/arm_2 sit on gantry 1, arm_3/arm_4 on gantry 2. The map stores ONE
# gantry; the partner arm is the same map rolled half a turn in rotation. The
# in-map names are NOT 'arm1'/'arm2' on both gantries -- gantry 2's pair is
# 'arm3'/'arm4' -- so they are read from the map's own tables rather than
# hardcoded, which is how the first version of this got it wrong.
ARM_MAP = {'arm_1': (1, GANTRY_ARM[1]), 'arm_2': (1, PARTNER[1]),
           'arm_3': (2, GANTRY_ARM[2]), 'arm_4': (2, PARTNER[2])}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', default='arm_1', choices=sorted(ARM_MAP))
    ap.add_argument('--lin', type=float, required=True,
                    help='rail position of that arm\'s gantry, in METRES, as '
                         'MEASURED from /joint_states')
    ap.add_argument('--rot', type=float, default=0.0, help='gantry rotation, deg')
    ap.add_argument('--n', type=int, default=8)
    ap.add_argument('--spread', type=int, metavar='N',
                    help='emit N targets SPREAD across the reachable set, one '
                         'per trial (A2 wants different poses, not one pose '
                         'ten times)')
    a = ap.parse_args()

    gantry, arm_key = ARM_MAP[a.arm]
    cm = CapabilityMap.load(f'/tmp/cap_g{gantry}_rail160.npz')
    li = int(np.argmin(abs(cm.lin - a.lin)))
    ri = int(np.argmin(abs(cm.rot - np.radians(a.rot))))
    print(f'{a.arm} on gantry {gantry} ({arm_key} in the map)')
    print(f'gantry pose used: lin={cm.lin[li]:.3f} m (asked {a.lin:.3f}), '
          f'rot={np.degrees(cm.rot[ri]):+.0f} deg')

    ok = np.array([xyz for xyz in cm.nodes
                   if cm.reach(arm_key, xyz, tol_i=0)[li, ri]])
    if not len(ok):
        print('\nNOTHING reachable at that pose -- check the rail position; a '
              'wrong `--lin` is the likely cause, not a broken map.')
        return 1
    print(f'\n{len(ok)} of {len(cm.nodes)} map nodes reachable '
          f'(L1 tol {cm.tols[0] * 100:.0f} cm)')
    print(f'  x {ok[:,0].min():.2f}..{ok[:,0].max():.2f}   '
          f'y {ok[:,1].min():.2f}..{ok[:,1].max():.2f}   '
          f'z {ok[:,2].min():.2f}..{ok[:,2].max():.2f}')

    # Depth from the reachable set's centroid: interior points are least likely
    # to sit on a reachability edge, where a few mm of mounting shift (A4) flips
    # them infeasible.
    depth = np.linalg.norm(ok - ok.mean(0), axis=1)

    if not a.spread:
        print('\ndeepest interior targets (furthest from any reachability '
              'edge).\nUse --approach 0: there is no object to clear, and the '
              'offset would\ncommand a pose that was never checked.\n')
        for j in np.argsort(depth)[:a.n]:
            print('  python3 scripts/reach_dwell_probe.py --arm %s '
                  '--approach 0 --target %.3f,%.3f,%.3f' % (a.arm, *ok[j]))
        return 0

    # A2 demands the trials use DIFFERENT poses. Ranking by depth alone does not
    # give that: the deepest points are grid neighbours ~7 cm apart, i.e. the
    # same pose measured ten times. Ten near-identical poses would report a
    # tracking property as if it were a workspace property.
    #
    # Farthest-point sampling instead, seeded at the deepest point and confined
    # to the interior half so nothing lands on a reachability edge: each new
    # target is the candidate furthest from every target already chosen.
    interior = ok[depth <= np.median(depth)]
    pick = [int(np.argmin(np.linalg.norm(interior - ok.mean(0), axis=1)))]
    dmin = np.linalg.norm(interior - interior[pick[0]], axis=1)
    while len(pick) < min(a.spread, len(interior)):
        j = int(np.argmax(dmin))
        pick.append(j)
        dmin = np.minimum(dmin, np.linalg.norm(interior - interior[j], axis=1))
    sel = interior[pick]

    sep = [np.linalg.norm(sel[i] - sel[j])
           for i in range(len(sel)) for j in range(i + 1, len(sel))]
    print(f'\n{len(sel)} target TERSEBAR (farthest-point, dibatasi ke separuh '
          f'bagian dalam).\n  pemisahan min {min(sep)*100:.1f} cm, '
          f'median {np.median(sep)*100:.1f} cm  <- A2 menuntut pose BERBEDA\n')
    print('# SALIN APA ADANYA. Urutan ini DIKUNCI SEBELUM data dilihat --')
    print('# memilih ulang titik setelah melihat hasil = memilih hasilnya.')
    for i, p in enumerate(sel, 1):
        print('python3 scripts/reach_dwell_probe.py --arm %s --approach 0 '
              '--trials 1 --target %.3f,%.3f,%.3f   # percobaan %d'
              % (a.arm, *p, i))
    return 0


if __name__ == '__main__':
    sys.exit(main())
