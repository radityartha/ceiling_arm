#!/usr/bin/env python3
"""G63 K0: scripts/k0_check.py unchanged, except that the arm_2 / arm_3 finger is expected CLOSED at its measured value
(A8.11: grippers closed under the boards, never commanded during G63), same tolerance 0.02 rad. The values come from the
first plain k0_check of the session (which fails ONLY on those two fingers) -> later K0s also catch a finger that slipped.

    python3 g63_k0.py --finger-arm2 <rad> --finger-arm3 <rad> --rail1 1.40 --rail2 0 [--json out.json]
"""
import argparse
import sys

sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
import k0_check  # noqa: E402

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument('--finger-arm2', type=float, required=True)
ap.add_argument('--finger-arm3', type=float, required=True)
a, rest = ap.parse_known_args()
orig = k0_check.expected


def expected(args):
    e = orig(args)
    for arm, v in (('arm_2', a.finger_arm2), ('arm_3', a.finger_arm3)):
        j = f'{k0_check.PREFIX[arm]}right_finger_bottom_joint'
        kind, _, tol = e[j]
        e[j] = (kind, v, tol)
    return e


k0_check.expected = expected
sys.argv = [sys.argv[0]] + rest
print(f'G63 K0: jari tertutup diharapkan arm_2 {a.finger_arm2:+.4f} / arm_3 {a.finger_arm3:+.4f} rad (tol '
      f'{k0_check.TOL_FINGER_RAD}); lainnya = k0_check', flush=True)
sys.exit(k0_check.main())
