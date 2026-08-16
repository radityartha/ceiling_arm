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

    python3 scripts/reachable_targets.py --lin 0.550
    python3 scripts/reachable_targets.py --lin 0.550 --arm arm2 --n 10
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

    # Rank by distance from the reachable set's own centroid: the deepest
    # interior points are the ones least likely to sit on a reachability edge
    # where a few mm of mounting shift flips them infeasible.
    d = np.linalg.norm(ok - ok.mean(0), axis=1)
    print(f'\ndeepest interior targets (furthest from any reachability edge).\n'
          f'Use --approach 0: there is no object to clear, and the offset would\n'
          f'command a pose that was never checked.\n')
    for j in np.argsort(d)[:a.n]:
        print('  python3 scripts/reach_dwell_probe.py --arm %s '
              '--approach 0 --target %.3f,%.3f,%.3f'
              % (a.arm, *ok[j]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
