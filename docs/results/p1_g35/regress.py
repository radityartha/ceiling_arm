#!/usr/bin/env python3
"""G35 A3: OLD (env_collision_old.py, frozen copy of G34) vs NEW (scripts/env_collision.py) screen_trajectory
on every archived trajectory, same map (canonical g34n), same machine. Offline, no ROS graph, no motion.

Gate: verdict identical 100 %, |d_min diff| <= 0.5 mm, worst geometry + waypoint identical; (P+) G32-HW ev 8
COLLIDE and first rejecting prefix >= 1.7 s before contact under BOTH.
Also 'lipschitz': the axiom the pruning rests on, checked on coal itself --
|d_a - d_b| <= max displacement of the body between a and b, per geometry, on real trajectories.

    python3 regress.py --source plans|v28|controls|hw|lipschitz --out regress_<source>.json
"""
from __future__ import annotations

import argparse
import glob
import gzip
import json
import math
import os
import sys
import time
import warnings

warnings.simplefilter('ignore')
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RES = os.path.join(ROOT, 'docs', 'results')
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(RES, 'p1_g33'))
import env_collision as NEW  # noqa: E402
import env_collision_old as OLD  # noqa: E402

PLAN_FILES = (sorted(glob.glob(os.path.join(RES, 'p1_g34', 'g34_planonly_plans_*.jsonl')))
              + [os.path.join(RES, 'p1_g33', f'g33_planonly_plans_{v}.jsonl.gz') for v in ('R0', 'R10')]
              + [os.path.join(RES, 'p1_g32hw', f'g32hw_screen_plans_{v}.jsonl.gz') for v in ('R0', 'R10')]
              + [os.path.join(RES, 'p1_g32', 'g32_screen_plans.jsonl.gz'),
                 os.path.join(RES, 'p1_g31', 'g31_screen_plans.jsonl.gz'),
                 os.path.join(RES, 'p1_g31', 'g31_r0_control_plans.jsonl'),
                 os.path.join(RES, 'p1_g28', 'smoke_mock_plans.jsonl')])
V28_FILES = [os.path.join(RES, 'p1_g28', 'v28_plans.jsonl.gz'), os.path.join(RES, 'p1_g28y', 'v28_after_plans.jsonl.gz')]


def rows(path):
    f = gzip.open(path, 'rt') if path.endswith('.gz') else open(path)
    for ln in f:
        r = json.loads(ln)
        t = r.get('traj')
        if t and t.get('pos') and r.get('start'):
            yield r


def full_start(start):
    """Archived starts before G29 lack rotations; G32+ hold them. Missing -> rot 0 (rotation did not exist
    then); any other missing joint -> skip (strict, counted)."""
    s = dict(start)
    for j in ('t1_rotation_joint', 't2_rotation_joint'):
        s.setdefault(j, 0.0)
    return s if all(j in s for j in NEW.CONFIG_JOINTS) else None


def traj_cases(src, files=None, part=(0, 1)):
    if src == 'plans' or src == 'v28':
        for path in files or (PLAN_FILES if src == 'plans' else V28_FILES):
            if not os.path.exists(path):
                print('TIDAK ADA', path, flush=True)
                continue
            for n, r in enumerate(rows(path)):
                if n % part[1] != part[0]:
                    continue
                s = full_start(r['start'])
                yield (f"{os.path.basename(path)}#{n}", r['traj']['joint_names'], r['traj']['pos'], s)
    elif src == 'controls':
        from g33_controls import G32, STEP, T_CONTACT, States
        st = States(os.path.join(G32, 'g32hw_joint_states1.csv.gz'))
        J = list(NEW.CONFIG_JOINTS)

        def win(t0, t1, step):
            ts = np.arange(t0, t1 + 1e-9, step)
            return ts, [[st.at(t)[j] for j in J] for t in ts]
        e8 = json.load(open(os.path.join(G32, 'g32hw_R10_run.json')))['events'][8]
        ts, pts = win(e8['t_start'], T_CONTACT + 0.5, 0.1)
        yield ('P+ ev8', J, pts, dict(zip(J, pts[0])), ts - T_CONTACT)
        for name, run, upto in (('R0', 'g32hw_R0_run.json', None), ('R10', 'g32hw_R10_run.json', 8)):
            for i, e in enumerate(json.load(open(os.path.join(G32, run)))['events'][:upto]):
                ts, pts = win(e['t_start'], e['t_end'], STEP)
                yield (f'N- {name} ev{i:02d}', J, pts, dict(zip(J, pts[0])))
    elif src == 'hw':
        import hw_replay as H
        from g33_controls import States
        sys.path.insert(0, os.path.join(RES, 'p1_g29'))
        import g29_rot_screen as R
        js = States(os.path.join(H.HW, 'g34hw_joint_states.csv.gz'))
        import gc
        gc.collect()
        gc.freeze()
        for v in ('R0', 'R10'):
            for path in sorted(glob.glob(os.path.join(H.HW, f'g34hw_{v}_ev*_*.log'))):
                kind = path.rsplit('_', 1)[1][:-4]
                if kind == 'task':
                    continue
                ph = H.phases(path, kind)
                if ph.get('skipped'):
                    continue
                tag = f'{v} {os.path.basename(path)[6:-4]}'
                if kind == 'traverse':
                    s = js.at(ph['t_cmd'] + 1.5)
                    g = ph['gantry']
                    frm = (s[f't{g}_linear_joint'], s[f't{g}_rotation_joint'])
                    to = (ph['goal'][0], math.radians(ph['goal'][1]))
                    yield (tag, [f't{g}_linear_joint', f't{g}_rotation_joint'], R.rect_points(frm, to), s)
                else:
                    s = js.at(ph['t_screen'])
                    jn = [f'{H.PREFIX[x]}joint_{i}' for x in ph['arms'] for i in range(1, 7)]
                    s0 = [s[n] for n in jn]
                    pts = [[a + (b - a) * 0.5 * (1.0 - math.cos(math.pi * k / 60))
                            for a, b in zip(s0, H.REST * (len(s0) // 6))] for k in range(1, 61)]
                    yield (tag, jn, pts, s)


def first_reject(ec, jn, pts, base):
    """Smallest prefix length whose verdict is not CLEAR (monotone in length -> bisection)."""
    if ec.screen_trajectory(jn, pts, base)[0] == 'CLEAR':
        return None
    lo, hi = 1, len(pts)
    while lo < hi:
        mid = (lo + hi) // 2
        if ec.screen_trajectory(jn, pts[:mid], base)[0] == 'CLEAR':
            lo = mid + 1
        else:
            hi = mid
    return lo - 1


def lipschitz(new):
    """Per geometry, consecutive and 10-apart waypoints of real trajectories: |d_a - d_b| <= disp."""
    import coal
    worst, n = -1e9, 0
    objs = new.geom.geometryObjects
    for path in V28_FILES[:1] + PLAN_FILES[:2]:
        for c, r in enumerate(rows(path)):
            if c >= 6:
                break
            s = full_start(r['start'])
            jn, P = r['traj']['joint_names'], r['traj']['pos']
            poses = []
            for p in P[::2]:
                j = dict(s)
                j.update(zip(jn, p))
                o = new.place(new.q_from(j))
                poses.append([(np.array(o[i].rotation), np.array(o[i].translation)) for i in range(len(objs))])
            D = [[coal.distance(objs[i].geometry, coal.Transform3s(*pp[i]), new.octree, new._I, new._req,
                                coal.DistanceResult()) for i in range(len(objs))] for pp in poses]
            for a in range(len(poses)):
                for b in (a + 1, a + 10):
                    if b >= len(poses):
                        continue
                    for i in range(len(objs)):
                        (Ra, ta), (Rb, tb) = poses[a][i], poses[b][i]
                        disp = np.linalg.norm(tb - ta) + new._r[i] * np.linalg.norm(Rb - Ra)
                        if D[a][i] > 0 and D[b][i] > 0:
                            worst = max(worst, abs(D[a][i] - D[b][i]) - disp)
                            n += 1
    return dict(n_pairs=n, max_violation_m=worst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True, choices=('plans', 'v28', 'controls', 'hw', 'lipschitz'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--file', nargs='+', help='plans/v28: only these files (parallel split)')
    ap.add_argument('--part', default='0/1', help='i/n: every n-th trajectory from i (parallel split)')
    a = ap.parse_args()
    part = tuple(int(x) for x in a.part.split('/'))
    new = NEW.EnvChecker()
    if a.source == 'lipschitz':
        r = lipschitz(new)
        print(r)
        json.dump(r, open(a.out, 'w'), indent=1)
        return
    old = OLD.EnvChecker()
    assert old.n_voxels == new.n_voxels and old.map_file == new.map_file
    out, bad, skipped = [], 0, 0
    T_old = T_new = 0.0
    for case in traj_cases(a.source, a.file, part):
        tag, jn, pts, base = case[:4]
        if base is None:
            skipped += 1
            print('SKIP (start tidak lengkap)', tag, flush=True)
            continue
        t0 = time.perf_counter()
        vo = old.screen_trajectory(jn, pts, base)
        t1 = time.perf_counter()
        vn = new.screen_trajectory(jn, pts, base)
        t2 = time.perf_counter()
        T_old += t1 - t0
        T_new += t2 - t1
        ok = vo[0] == vn[0] and abs(vo[1] - vn[1]) <= 0.0005 and vo[2] == vn[2] and vo[3] == vn[3]
        row = dict(tag=tag, n=len(pts), old=list(vo), new=list(vn), ok=ok, bit_identical=vo[1] == vn[1],
                   t_old=t1 - t0, t_new=t2 - t1)
        if len(case) > 4:                                    # P+: first rejecting prefix, both impls
            trel = case[4]
            fo, fn = first_reject(old, jn, pts, base), first_reject(new, jn, pts, base)
            row.update(first_reject_t_rel_old=None if fo is None else float(trel[fo]),
                       first_reject_t_rel_new=None if fn is None else float(trel[fn]))
            ok &= fo == fn and fn is not None and trel[fn] <= -1.7 and vn[0] == 'COLLIDE'
            row['ok'] = ok
        bad += not ok
        out.append(row)
        print(f"{'OK ' if ok else 'BEDA'} {tag:44s} n {len(pts):4d} {vo[0]:7s}/{vn[0]:7s} "
              f"{vo[1] * 1000:8.2f}/{vn[1] * 1000:8.2f} mm {t1 - t0:6.2f}/{t2 - t1:5.2f} s"
              + (f"  tolak {row['first_reject_t_rel_old']:+.2f}/{row['first_reject_t_rel_new']:+.2f} s"
                 if 'first_reject_t_rel_new' in row and row['first_reject_t_rel_new'] is not None else ''),
              flush=True)
    summ = dict(source=a.source, n=len(out), bad=bad, skipped=skipped,
                bit_identical=sum(r['bit_identical'] for r in out),
                verdicts={v: sum(r['new'][0] == v for r in out) for v in ('CLEAR', 'MARGIN', 'COLLIDE')},
                t_old=T_old, t_new=T_new, speedup=T_old / T_new if T_new else None,
                n_samples=sum(r['n'] for r in out))
    print('RINGKAS', json.dumps(summ), flush=True)
    json.dump(dict(summary=summ, rows=out), open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
