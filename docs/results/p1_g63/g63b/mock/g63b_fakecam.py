#!/usr/bin/env python3
"""G63b kamera palsu MOCK (domain 77 saja): g63_fakecam.py (G63, TIDAK diubah) + rakitan dua muka di satu lengan +
render lembar A4 papan catur POLOS (g63b_e2e.render: tepi 21 / 43.5 mm, blur 1.3, derau 2.3).

Kebenaran tersembunyi (truth-out) = incumbent digeser (--rot-deg/--pos-m) + mount per muka digeser; prior yang terbit =
incumbent. Muka belakang dibuang oleh g63_fakecam (Tcb[2,2] < 0.2).

    ROS_DOMAIN_ID=77 g63b_fakecam.py --arm arm_2 --mount ../mount_g63b.json --truth-out truth.json
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
G63B = os.path.dirname(HERE)
G63 = os.path.dirname(G63B)
for p in (G63B, G63, os.path.join(G63, 'mock')):
    sys.path.insert(0, p)
import g63_detect  # noqa: E402
import g63_fakecam  # noqa: E402
import g63b_capture  # noqa: E402
import g63b_e2e  # noqa: E402

i = sys.argv.index('--arm')
arm = sys.argv[i + 1]
del sys.argv[i:i + 2]
j = sys.argv.index('--mount')
tmp, mt = g63b_capture.two_face_mount(sys.argv[j + 1])
sys.argv[j + 1] = tmp
g63_fakecam.BOARD_ARM = {'A': arm, 'B': arm}
SQ = float(mt['A']['square_m'])
TEX = g63b_e2e.sheet_tex(SQ)


def render(name, Tcb, K, blur_px=1.0, noise=3.0, rng=None, bg=None, margin_m=0.005, bg_level=90):
    import numpy as np
    base = bg if bg is not None else np.full((720, 1280), bg_level, np.uint8)
    return g63b_e2e.render(TEX, SQ, Tcb, K, base, rng or np.random.default_rng(0))


g63_detect.render = render
if __name__ == '__main__':
    sys.exit(g63_fakecam.main())
