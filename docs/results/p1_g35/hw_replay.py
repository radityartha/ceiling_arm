#!/usr/bin/env python3
"""G35 A1: per-event pre-send phases of G34b HW (log stamps) vs an offline replay of the
same environment screens (offline, no ROS graph, no motion).

HW phases from docs/results/p1_g34/hw/g34hw_{R0,R10}_evNN_*.log (stdout stamped by run_g32):
  traverse  start = CMD -> S12 | S28 = S23" -> S28 line | ENV = S28 -> G33 line | motion = G33 -> akhir
  task      plan = CMD -> torque line | INTERARM = torque -> antar-lengan line | ENV = antar-lengan -> lingkungan line
            | exec = lingkungan -> MOVED
  retract   load = 'rel' -> 'retract g' line (3 checkers) | SCREENS = 'retract g' -> 'rencana pulang' (S18+cross+env)
            | motion = 'mengirim' -> 'selesai'
Replay: the measured state (g34hw_joint_states.csv.gz) at the screen's stamp -> the SAME env inputs
(rect_points for traverse, _rest_line for retract; task = plan-only G34b-HW plan of the same k, a proxy:
HW plans were not archived) -> fresh EnvChecker build + screen_trajectory, timed.

    python3 hw_replay.py --impl old|new --out hw_replay_old.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
import time
import warnings

warnings.simplefilter('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'docs', 'results', 'p1_g29'))
sys.path.insert(0, os.path.join(ROOT, 'docs', 'results', 'p1_g33'))
import env_collision as E  # noqa: E402
import g29_rot_screen as R  # noqa: E402
from g33_controls import States  # noqa: E402

HW = os.path.join(ROOT, 'docs', 'results', 'p1_g34', 'hw')
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}


def stamps(path):
    out = []
    for ln in open(path):
        m = re.match(r'^#? ?(\d{10}\.\d+) (.*)$', ln.rstrip('\n'))
        if m:
            out.append((float(m.group(1)), m.group(2)))
    return out


def first(st, pat, after=0.0):
    for t, s in st:
        if t >= after and re.search(pat, s):
            return t, s
    return None, None


def phases(path, kind):
    st = stamps(path)
    t_cmd = st[0][0]
    ph = dict(t_cmd=t_cmd)
    if kind == 'traverse':
        t12, _ = first(st, r'^S12:')
        t28, s28 = first(st, r'^S28:')
        t33, s33 = first(st, r'^G33:')
        tend, _ = first(st, r'^akhir:')
        ph.update(start=t12 - t_cmd, s28=t28 - t12, env=t33 - t28, motion=tend - t33, pre_send=t33 - t_cmd,
                  n=int(re.search(r'n (\d+)', s28).group(1)), t_env_end=t33,
                  env_mm=float(re.search(r'min ([\d.]+) mm', s33).group(1)))
        m = re.search(r'--gantry (\d) .* ([\d.]+) (-?[\d.]+) --move', st[0][1])
        ph.update(gantry=int(m.group(1)), goal=(float(m.group(2)), float(m.group(3))))
    elif kind == 'task':
        tt, _ = first(st, r'torsi RNEA')
        ti, _ = first(st, r'antar-lengan minimum')
        te, se = first(st, r'peta lingkungan minimum')
        tm, _ = first(st, r'moveit MOVED')
        ph.update(plan=tt - t_cmd, interarm=ti - tt, env=te - ti, exec=(tm - te) if tm else None,
                  pre_send=te - t_cmd, t_env_end=te, env_mm=float(re.search(r'minimum: ([\d.]+) mm', se).group(1)))
        m = re.search(r'--arms (arm_\d) --target=([-\d.,]+)', st[0][1])
        ph.update(arm=m.group(1), xyz=[float(x) for x in m.group(2).split(',')])
    else:
        tr, _ = first(st, r'^rel ')
        tg, sg = first(st, r'^retract g')
        tp, _ = first(st, r'^rencana pulang')
        ts, _ = first(st, r'^mengirim')
        te, _ = first(st, r'^selesai')
        tenv, senv = first(st, r'penyaring lingkungan')
        if tg is None:
            ph.update(skipped=True)
            return ph
        ph.update(load=tg - tr, screens=tp - tg, motion=(te - ts) if te else None, pre_send=(ts or tp) - t_cmd,
                  t_screen=tg, env_mm=float(re.search(r'minimum ([\d.]+) mm', senv).group(1)),
                  arms=re.search(r"\[(.*?)\]", sg).group(1).replace("'", '').split(', '))
        ph['gantry'] = int(re.search(r'retract g(\d)', sg).group(1))
    return ph


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--impl', choices=('old', 'new'), default='old')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.impl == 'old':
        sys.path.insert(0, HERE)
        import env_collision_old as EO
        Checker = EO.EnvChecker
    else:
        Checker = E.EnvChecker
    t = time.perf_counter()
    js = States(os.path.join(HW, 'g34hw_joint_states.csv.gz'))
    print(f'joint_states dimuat {time.perf_counter() - t:.1f} s', flush=True)
    import gc
    gc.collect()
    gc.freeze()          # the big States lists otherwise make every gen-2 GC slow the timed build
    plans = {}
    for v in ('R0', 'R10'):
        for ln in open(os.path.join(ROOT, f'docs/results/p1_g34/g34_planonly_plans_g34b_hw_{v}.jsonl')):
            r = json.loads(ln)
            if r['sample'] == 0 and r.get('traj'):
                plans[(v, r['k'])] = r
    rows = []
    for v in ('R0', 'R10'):
        for path in sorted(glob.glob(os.path.join(HW, f'g34hw_{v}_ev*_*.log'))):
            k = int(re.search(r'_ev(\d+)_', path).group(1))
            kind = re.search(r'_ev\d+_(\w+)\.log', path).group(1)
            ph = phases(path, kind)
            row = dict(variant=v, ev=k, kind=kind, **ph)
            if ph.get('skipped'):
                rows.append(row)
                continue
            if kind == 'traverse':
                state = js.at(ph['t_cmd'] + 1.5)
                g = ph['gantry']
                frm = (state[f't{g}_linear_joint'], state[f't{g}_rotation_joint'])
                to = (ph['goal'][0], math.radians(ph['goal'][1]))
                jn, pts = [f't{g}_linear_joint', f't{g}_rotation_joint'], R.rect_points(frm, to)
                base = state
            elif kind == 'retract':
                state = js.at(ph['t_screen'])
                todo = [x for x in ph['arms']]
                jn = [f'{PREFIX[x]}joint_{i}' for x in todo for i in range(1, 7)]
                st0 = [state[n] for n in jn]
                pts = [[s + (gg - s) * 0.5 * (1.0 - math.cos(math.pi * kk / 60))
                        for s, gg in zip(st0, REST * (len(st0) // 6))] for kk in range(1, 61)]
                base = state
            else:
                p = plans.get((v, k))
                if p is None:
                    rows.append(row)
                    continue
                jn, pts, base = p['traj']['joint_names'], p['traj']['pos'], p['start']
            t0 = time.perf_counter()
            ec = Checker()
            t1 = time.perf_counter()
            res = ec.screen_trajectory(jn, pts, base)
            t2 = time.perf_counter()
            row.update(rep_build=t1 - t0, rep_screen=t2 - t1, rep_n=len(pts), rep_verdict=res[0],
                       rep_d_mm=res[1] * 1000, rep_geom=res[2], rep_k=res[3])
            hw_env = ph.get('env')
            print(f"{v:3s} ev{k:02d} {kind:8s} n {len(pts):4d} rep build {t1 - t0:5.2f} + screen {t2 - t1:6.2f} s"
                  f"  | HW env {hw_env if hw_env is None else round(hw_env, 2)}"
                  f"  rep {res[0]} {res[1] * 1000:.1f} mm (HW {ph.get('env_mm')})", flush=True)
            rows.append(row)
    if a.out:
        json.dump(rows, open(a.out, 'w'), indent=1, default=str)


if __name__ == '__main__':
    main()
