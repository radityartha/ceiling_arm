#!/usr/bin/env python3
"""G63b HW: SATU leg = g63_move.py (G63, TIDAK diubah; argumen identik plan-check, dari keadaan TERUKUR, --move kecuali
--dry) + '--g63-boards <papan fase>' ke probe_g63 (fase arm_2: A, fase arm_3: B). Satu izin operator per pemanggilan.

    g63b_move.py --board A --session S --poses poses_g63b_v4.json --payload S/payload_arm_2.urdf \
        --extra-env ../s0c_extra_env.npz --arm arm_2 --pose A-R0 [--dry]      (--pose REST = leg pulang)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g63_move  # noqa: E402

if __name__ == '__main__':
    i = sys.argv.index('--board')
    board = sys.argv[i + 1]
    del sys.argv[i:i + 2]
    _run = subprocess.run

    def run(cmd, *a, **k):
        j = next(n for n, c in enumerate(cmd) if str(c).endswith('probe_g63.py'))
        return _run(cmd[:j + 1] + ['--g63-boards', board] + cmd[j + 1:], *a, **k)
    g63_move.subprocess.run = run
    sys.exit(g63_move.main())
