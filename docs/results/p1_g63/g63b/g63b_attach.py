#!/usr/bin/env python3
"""G63b attach: g63_attach.py (G63, TIDAK diubah) + upper_wrist_link sebagai touch link.

Rakitan G63b dimodelkan SATU kotak (dua lembar + ruang di antaranya, mount_g63b.json); tepi atas lembar 8 mm di atas flens
-> kotak beririsan housing pergelangan secara MODEL (lembar nyata di luar, menempel badan gripper). Tanpa touch link ini
MoveIt menilai setiap keadaan = tabrakan (START-JUMP). Hanya pasangan papan<->link lengan pembawa sendiri yang dibebaskan;
papan vs lingkungan / lengan lain tetap diperiksa penuh. Fase arm_2: --boards A saja (rakitan di arm_2).

    python3 g63b_attach.py --mount mount_g63b.json --boards A [--remove | --verify-only]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g63_attach  # noqa: E402

if 'upper_wrist_link' not in g63_attach.TOUCH:
    g63_attach.TOUCH = tuple(g63_attach.TOUCH) + ('upper_wrist_link',)

if __name__ == '__main__':
    sys.exit(g63_attach.main())
