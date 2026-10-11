#!/usr/bin/env python3
"""K0 = the MEASURED start state of a run (docs/p1_g45_k0.md). Observer only -- commands nothing.

G36b: the verbal K0 ("same as before") was wrong -- three arms off REST, gantry 2 at -34.9 mm / +2.91 deg. This merges
/joint_states BY NAME for --seconds (two publishers send separate messages, G16) and refuses unless every arm is at
REST, every rail / rotation is where the plan starts, and every gripper finger is closed.

    python3 k0_check.py --rail1 1.50 --rail2 0.00 [--rot1 0 --rot2 0] [--seconds 1.0] [--json out.json]

rc 0 = K0 LULUS, 1 = TOLAK, 2 = bad arguments.
"""
import argparse
import json
import math
import sys
import time

REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
TOL_ARM_DEG = 0.5            # = run_g32 A3
TOL_RAIL_M = 0.002           # = run_g32 s26; bridge stops <= 1 mm short
TOL_ROT_DEG = 0.5            # G30: rotation error <= 0.29 deg (s26 allows 1.0 per event)
TOL_FINGER_RAD = 0.02        # G36 B1: +0.05 rad already puts a REST-adjacent finger into the MoveIt map box
MIN_SAMPLES = 3


def expected(a):
    e = {}
    for arm, p in PREFIX.items():
        for i, v in enumerate(REST, 1):
            e[f'{p}joint_{i}'] = ('arm', v, math.radians(TOL_ARM_DEG))
        e[f'{p}right_finger_bottom_joint'] = ('finger', 0.0, TOL_FINGER_RAD)
    for g in (1, 2):
        e[f't{g}_linear_joint'] = ('rail', getattr(a, f'rail{g}'), TOL_RAIL_M)
        e[f't{g}_rotation_joint'] = ('rot', math.radians(getattr(a, f'rot{g}')), math.radians(TOL_ROT_DEG))
    return e


def judge(samples, exp):
    """samples {joint: [values]} -> (ok, reasons, rows {joint: dict})."""
    why, rows = [], {}
    for j, (kind, want, tol) in exp.items():
        v = samples.get(j, [])
        if len(v) < MIN_SAMPLES:
            why.append(f'{j}: {len(v)} sampel (< {MIN_SAMPLES})')
            rows[j] = dict(kind=kind, n=len(v))
            continue
        med = sorted(v)[len(v) // 2]
        rng = max(v) - min(v)
        rows[j] = dict(kind=kind, n=len(v), median=med, err=med - want, range=rng, tol=tol)
        if abs(med - want) > tol:
            why.append(f'{j}: galat {fmt(kind, med - want)} > {fmt(kind, tol)}')
        if rng > tol:
            why.append(f'{j}: bergerak, rentang {fmt(kind, rng)} > {fmt(kind, tol)}')
    return not why, why, rows


def fmt(kind, x):
    return f'{x * 1000:+.2f} mm' if kind == 'rail' else (f'{x:+.4f} rad' if kind == 'finger'
                                                        else f'{math.degrees(x):+.3f} deg')


def main():
    ap = argparse.ArgumentParser()
    for g in (1, 2):
        ap.add_argument(f'--rail{g}', type=float, required=True, help=f'rel g{g} yang diharapkan (m)')
        ap.add_argument(f'--rot{g}', type=float, default=0.0, help=f'rotasi g{g} yang diharapkan (deg)')
    ap.add_argument('--seconds', type=float, default=1.0)
    ap.add_argument('--json', default=None)
    a = ap.parse_args()
    if a.seconds < 0.5:
        ap.error('--seconds minimal 0.5')
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    samples = {}
    rclpy.init()
    n = Node('k0_check')

    def cb(m):
        for k, x in zip(m.name, m.position):
            samples.setdefault(k, []).append(float(x))
    n.create_subscription(JointState, '/joint_states', cb, 200)
    exp = expected(a)
    t0 = time.time()                         # G45 B1: the window opens at the FIRST message, not at
    # subscribe -- DDS discovery alone can eat a 1 s window. G51 B10: and only once EVERY checked joint was seen:
    # the gantry bridge (2nd /joint_states publisher) can be discovered first, leaving 0 arm samples in the window.
    while not all(j in samples for j in exp) and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()
    ok, why, rows = judge(samples, exp)
    for arm, p in PREFIX.items():
        r = [(abs(rows[f'{p}joint_{i}'].get('err', math.inf)), i) for i in range(1, 7)]
        worst = max(r)
        print(f'{arm}: maks |q - REST| {math.degrees(worst[0]):.3f} deg (joint_{worst[1]}); jari '
              f'{rows[f"{p}right_finger_bottom_joint"].get("median", "HILANG")}')
    for g in (1, 2):
        rl, rt = rows[f't{g}_linear_joint'], rows[f't{g}_rotation_joint']
        print(f'g{g}: rel {rl.get("median", "HILANG")} m (harap {getattr(a, f"rail{g}"):.4f}), rotasi '
              f'{math.degrees(rt["median"]) if "median" in rt else "HILANG"} deg (harap {getattr(a, f"rot{g}"):+.2f})')
    print(f'sampel per sendi: min {min(r["n"] for r in rows.values())}, {a.seconds:.1f} s')
    print(f'K0 {"LULUS" if ok else "TOLAK"}' + ('' if ok else ': ' + '; '.join(why)))
    if a.json:
        json.dump(dict(ok=ok, why=why, rows=rows, args=vars(a), t=time.time()), open(a.json, 'w'), indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
