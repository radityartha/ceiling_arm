"""G23 A3: ORACLE' vs the REAL planner on the 49 G22 B1 plans, + NC1-NC5.

V = every '[k] task tN arm -> [xyz] @rail R: VERDICT' line of p1_g22/g22_screen.json.
Margin = g23_calib2.json (A2', B0.1). Computed ONCE.

    python3 validate.py -> g23_validate.json + tables
"""
import json
import os
import re
import sys

import numpy as np

import oracle as O

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../../ros2_ws/src/reachability_gng'))
from reachability_gng.capability import CapabilityMap, GANTRY_ARM, PARTNER  # noqa: E402

LINE = re.compile(r'task t(\d+) (arm_\d) -> \[([-\d.]+), ([-\d.]+), ([-\d.]+)\] @rail ([\d.]+): (\S+)')
KEY = {'arm_1': GANTRY_ARM[1], 'arm_2': PARTNER[1], 'arm_3': GANTRY_ARM[2], 'arm_4': PARTNER[2]}
CAP = {g: CapabilityMap.load(os.path.join(HERE, f'../../../ros2_ws/src/reachability_gng/data/cap_g{g}_rail160.npz'))
       for g in (1, 2)}


def vset():
    out = []
    for s in json.load(open(os.path.join(HERE, '../p1_g22/g22_screen.json'))):
        for ln in s['log']:
            m = LINE.search(ln)
            if m:
                out.append(dict(seed=s['seed'], task=int(m[1]), arm=m[2],
                                xyz=[float(m[3]), float(m[4]), float(m[5])],
                                rail=float(m[6]), verdict=m[7]))
    return out


def l1(v):
    c = CAP[O.GANTRY[v['arm']]]
    li = int(np.argmin(abs(c.lin - v['rail'])))
    ri = int(np.argmin(abs(c.rot)))
    return bool(c.reach(KEY[v['arm']], np.array(v['xyz']), tol_i=0)[li, ri])


def main():
    m2 = np.array(json.load(open(os.path.join(HERE, 'g23_calib2.json')))['margin'])
    m1 = np.array(json.load(open(os.path.join(HERE, 'g23_calib.json')))['margin'])
    V = vset()
    print(f'V = {len(V)}: ' + ', '.join(f'{k} {sum(v["verdict"] == k for v in V)}'
                                        for k in ('PLANNED', 'TORQUE-UNSAFE', 'NO-PLAN')))
    print(f"margin A2' = {np.round(m2, 4).tolist()}")
    for v in V:
        t, _ = O.solve(v['xyz'], v['rail'], v['arm'])
        t2, _ = O.solve(v['xyz'], v['rail'], v['arm'])
        tp, _ = O.solve(v['xyz'], v['rail'], v['arm'], pos_only=True)
        tz, _ = O.solve([v['xyz'][0], v['xyz'][1], 0.40], v['rail'], v['arm'])
        conv = ~np.isnan(t[:, 0])
        v.update(L1=l1(v), n_conv=int(conv.sum()),
                 j2_branches=np.round(np.sort(t[conv, 1]), 3).tolist(),
                 ik=bool(conv.any()),
                 torq=bool(O.torq_all(t, m2)),
                 oracle=bool(l1(v) and conv.any() and O.torq_all(t, m2)),
                 locked_A2=bool(l1(v) and O.torq_ok(t, m1)),
                 nc1=bool((~np.isnan(tz[:, 0])).any()),          # IK_V alone
                 nc2=bool(O.torq_all(t, m2, lim=np.array([10, 5, 10, 7, 7, 7.0]))),
                 nc3=bool(l1(v) and O.torq_all(tp, m2)),
                 nc4=bool(l1(v) and conv.any()),
                 nc5=bool(np.array_equal(t, t2, equal_nan=True)))
        v['why'] = ('-' if v['oracle'] else 'L1' if not v['L1'] else
                    'IK_V' if not v['ik'] else 'TORQ')
        print(f"  s{v['seed']:2d} t{v['task']} {v['arm']} {v['xyz']} @{v['rail']:.2f} "
              f"{v['verdict']:13s} oracle {'TERIMA' if v['oracle'] else 'tolak:' + v['why']:11s} "
              f"j2 cabang {v['j2_branches']}")
    print('\nmatriks ORACLE\' x perencana:')
    for acc in (True, False):
        print(f"  {'terima' if acc else 'tolak '}: " + '  '.join(
            f"{k} {sum(v['oracle'] == acc and v['verdict'] == k for v in V)}"
            for k in ('PLANNED', 'TORQUE-UNSAFE', 'NO-PLAN')))
    rej = [v for v in V if not v['oracle']]
    for k in ('TORQUE-UNSAFE', 'NO-PLAN', 'PLANNED'):
        print(f"  sebab tolak pada {k}: " + ', '.join(
            f"{w} {sum(v['why'] == w and v['verdict'] == k for v in rej)}" for w in ('L1', 'IK_V', 'TORQ')))
    acc = [v for v in V if v['oracle']]
    npl = sum(v['verdict'] == 'PLANNED' for v in V)
    print(f"  presisi {sum(v['verdict'] == 'PLANNED' for v in acc)}/{len(acc)}, "
          f"recall {sum(v['verdict'] == 'PLANNED' for v in acc)}/{npl}")
    print(f"  oracle TERKUNCI A2 (m {np.round(m1, 2).tolist()}, exists): terima {sum(v['locked_A2'] for v in V)}/{len(V)}")
    print('kontrol:')
    print(f"  NC1 z=0.40         terima {sum(v['nc1'] for v in V)}/{len(V)} (harus 0)")
    print(f"  NC2 lim_2=5        terima {sum(v['nc2'] for v in V)}/{len(V)} (harus 0)")
    print(f"  NC3 posisi saja    terima {sum(v['nc3'] for v in V)} vs oracle {len(acc)}; "
          f"pada NO-PLAN {sum(v['nc3'] and v['verdict'] == 'NO-PLAN' for v in V)} vs "
          f"{sum(v['oracle'] and v['verdict'] == 'NO-PLAN' for v in V)}")
    print(f"  NC4 tanpa TORQ'    terima {sum(v['nc4'] for v in V)} vs oracle {len(acc)}; "
          f"pada TORQUE-UNSAFE {sum(v['nc4'] and v['verdict'] == 'TORQUE-UNSAFE' for v in V)} vs "
          f"{sum(v['oracle'] and v['verdict'] == 'TORQUE-UNSAFE' for v in V)}")
    print(f"  NC5 bit-identik    {sum(v['nc5'] for v in V)}/{len(V)}")
    json.dump(dict(margin=m2.tolist(), rows=V), open(os.path.join(HERE, 'g23_validate.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
