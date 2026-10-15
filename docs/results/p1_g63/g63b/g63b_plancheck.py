#!/usr/bin/env python3
"""G63b plan-check berantai: g63_plancheck.py (G63, TIDAK diubah) + '--g63-boards <papan fase>' ke tiap probe_g63.

Fase arm_2: rakitan = entri 'A' (BOARD_ARM A->arm_2), payload URDF dari mount fase berisi A saja; fase arm_3: entri 'B'.
Tanpa sisipan ini probe_g63 menuntut papan A DAN B terpasang (G63: dua lengan sekaligus).

    g63b_plancheck.py --board A --poses poses_g63b_v3.json --payload P.urdf --arm arm_2 --out pc.json [--stages P0] [--extra-env S.npz]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g63_plancheck  # noqa: E402

if __name__ == '__main__':
    i = sys.argv.index('--board')
    board = sys.argv[i + 1]
    del sys.argv[i:i + 2]
    _run = subprocess.run

    def run(cmd, *a, **k):
        j = next(n for n, c in enumerate(cmd) if str(c).endswith('probe_g63.py'))
        return _run(cmd[:j + 1] + ['--g63-boards', board] + cmd[j + 1:], *a, **k)
    g63_plancheck.subprocess.run = run
    sys.exit(g63_plancheck.main())
