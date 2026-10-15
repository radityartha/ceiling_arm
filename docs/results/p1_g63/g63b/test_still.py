#!/usr/bin/env python3
"""Uji unit aturan diam (g63_capture.still_check) -- G63 (kosong) dan G63b (j4 berbasis pp). Tanpa ROS."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import g63_capture as C  # noqa: E402

names = [f't1_a2_joint_{k}' for k in range(1, 7)]
rng = np.random.default_rng(0)


def win(j4_pp=0.0, other_std=0.0, n=20):
    t = np.linspace(0, 1, n)
    out = []
    for k in range(n):
        j = {nm: 0.1 + rng.normal(scale=other_std) if other_std else 0.1 for nm in names}
        j['t1_a2_joint_4'] = 0.1 + 0.5 * j4_pp * np.sign(np.sin(2 * np.pi * 3 * t[k] + 0.1))
        out.append(j)
    return out


fails = 0


def check(label, got, want):
    global fails
    ok = got == want
    fails += not ok
    print(f'{"OK " if ok else "GAGAL"} {label}: {got} (harus {want})')


C.STILL_EXEMPT_PP = {}
check('G63 diam total', C.still_check(win(), names)[0], True)
check('G63 j4 berburu 3.2e-3 -> tolak (perilaku lama)', C.still_check(win(3.2e-3), names)[0], False)
LIM = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0e-3
C.STILL_EXEMPT_PP = {'joint_4': LIM}
check(f'G63b j4 3.2e-3 <= {LIM:.2e} -> terima', C.still_check(win(3.2e-3), names)[0], True)
check(f'G63b j4 {LIM * 1.5:.2e} > batas -> tolak', C.still_check(win(LIM * 1.5), names)[0], False)
check('G63b sendi lain std 5e-4 -> tolak (tidak dilonggarkan)', C.still_check(win(0.0, 5e-4), names)[0], False)
print('HASIL', 'LULUS' if fails == 0 else f'GAGAL {fails}')
sys.exit(1 if fails else 0)
