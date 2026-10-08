#!/usr/bin/env python3
"""G36 A3: OLD (interarm_collision_old.py, frozen copy of G35) vs NEW (scripts/interarm_collision.py)
InterArmChecker.screen_trajectory (S18, same-gantry meshes) on every archived trajectory. Offline, no ROS, no motion.

Cases per archived plan row (task or MoveIt retract, every file G35 A3 used + G36 plan-only):
  plan     the arm's trajectory vs its partner HELD at the archived start (= screen_interarm)
  retract  BOTH arms of that gantry, the straight return_rest line from the plan's END to REST (both bodies move)
plus 'controls' (G33: measured HW windows, all 24 arm joints moving, S18 g1 and g2) and 'hw' (G34b straight
retracts from the measured state). Gate: verdict, d_min, pair and waypoint identical (bit-identical d_min).

    python3 g36_regress.py --source plans|v28|controls|hw [--part i/n] --out g36_regress_<x>.json
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time
import warnings

warnings.simplefilter('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RES = os.path.join(ROOT, 'docs', 'results')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(RES, 'p1_g35'))
sys.path.insert(0, os.path.join(RES, 'p1_g33'))
import glob  # noqa: E402
import interarm_collision as NEW  # noqa: E402
import regress as G35  # noqa: E402  (file lists, rows(), full_start(), traj_cases('controls'|'hw'))
from return_rest import PREFIX, REST, _rest_line  # noqa: E402

spec = importlib.util.spec_from_file_location('ia_old', os.path.join(HERE, 'interarm_collision_old.py'))
OLD = importlib.util.module_from_spec(spec)
spec.loader.exec_module(OLD)
G36_FILES = sorted(glob.glob(os.path.join(HERE, 'g36_planonly_plans_*.jsonl')))


def cases(src, part):
    n = -1
    if src in ('plans', 'v28'):
        for path in (G35.PLAN_FILES + G36_FILES) if src == 'plans' else G35.V28_FILES:
            if not os.path.exists(path):
                print('TIDAK ADA', path, flush=True)
                continue
            for i, r in enumerate(G35.rows(path)):
                n += 1
                if n % part[1] != part[0]:
                    continue
                s = G35.full_start(r['start'])
                arm, jn, pts = r['arm'], r['traj']['joint_names'], r['traj']['pos']
                g = int(PREFIX[arm][1])
                tag = f'{os.path.basename(path)}#{i}'
                yield tag + ' plan', g, jn, pts, s
                end = dict(s)
                end.update(zip(jn, pts[-1]))
                both = [f'{PREFIX[x]}joint_{k}' for x in PREFIX if PREFIX[x][1] == str(g) for k in range(1, 7)]
                if max(abs(end[j] - REST[int(j[-1]) - 1]) for j in both) >= math.radians(0.5):
                    yield tag + ' retract', g, both, _rest_line([end[j] for j in both]), end
    else:
        for c in G35.traj_cases(src):
            n += 1
            if n % part[1] != part[0]:
                continue
            tag, jn, pts, s = c[:4]
            for g in (1, 2):
                if any(j.startswith(f't{g}_a') for j in jn):
                    yield f'{tag} g{g}', g, jn, pts, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True, choices=('plans', 'v28', 'controls', 'hw'))
    ap.add_argument('--part', default='0/1')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    part = tuple(int(x) for x in a.part.split('/'))
    old = {g: OLD.InterArmChecker(gantry=f'gantry_{g}') for g in (1, 2)}
    new = {g: NEW.InterArmChecker(gantry=f'gantry_{g}') for g in (1, 2)}
    out, bad, to, tn, npts = [], 0, 0.0, 0.0, 0
    for tag, g, jn, pts, s in cases(a.source, part):
        t = time.time(); vo = old[g].screen_trajectory(jn, pts, s); t1 = time.time()
        vn = new[g].screen_trajectory(jn, pts, s); t2 = time.time()
        same = (vo[0], vo[1], tuple(vo[2]), vo[3]) == (vn[0], vn[1], tuple(vn[2]), vn[3])
        bad += not same
        to += t1 - t
        tn += t2 - t1
        npts += len(pts)
        out.append(dict(tag=tag, n=len(pts), old=list(vo), new=list(vn), same=same,
                        t_old=round(t1 - t, 3), t_new=round(t2 - t1, 3)))
        print(f'{"OK  " if same else "BEDA"} {tag} n {len(pts)} {vo[0]} {vo[1] * 1000:.3f} / {vn[0]} {vn[1] * 1000:.3f} '
              f'{t1 - t:.2f}->{t2 - t1:.2f}s', flush=True)
    summ = dict(source=a.source, part=a.part, n=len(out), points=npts, bad=bad, t_old=round(to, 1), t_new=round(tn, 1),
                verdicts={v: sum(o['old'][0] == v for o in out) for v in ('CLEAR', 'MARGIN', 'COLLIDE')})
    print('RINGKAS', json.dumps(summ), flush=True)
    json.dump(dict(summary=summ, cases=out), open(a.out, 'w'), indent=0)


if __name__ == '__main__':
    main()
