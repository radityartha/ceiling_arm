"""G61 A5.1 regression: every task argv of a G61 smoke == the argv of the G58/G60 mock smoke of the same seed + variant
(paths of the monitor csv, reservation dir and out-json normalised). Fake HW logs.

    python3 g61_argv_cmp.py <g61 prefix> <old prefix>      e.g. smoke/pos_s7B ../p1_g54/smoke/g58_s7B
"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def argv(fn):
    ln = open(fn).readline()
    a = ln.split(' CMD ', 1)[1].strip()
    a = re.sub(r'--monitor-csv \S+', '--monitor-csv MON', a)
    a = re.sub(r'--reserve-dir \S+', '--reserve-dir RES', a)
    return re.sub(r'--out-json \S+', '--out-json OUT', a)


new, old = (os.path.join(HERE, x) for x in sys.argv[1:3])
fs = sorted(glob.glob(new + '_ev*_task.log'))
bad = 0
for f in fs:
    o = old + f[len(new):]
    same = os.path.exists(o) and argv(f) == argv(o)
    bad += not same
    print(f'{os.path.basename(f)}: {"IDENTIK" if same else "BEDA"}')
print(f'{len(fs) - bad}/{len(fs)} argv tugas identik')
sys.exit(1 if bad or not fs else 0)
