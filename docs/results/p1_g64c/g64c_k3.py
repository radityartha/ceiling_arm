"""G64-C fase 3 gerbang C5 (docs/p1_g64c.md A1) LURING, tanpa stack ROS: mode REST = sebelum perubahan, bit demi bit.

  K1 events(sched, p0, fr) baru == HEAD, juga dengan mode {REST, REST}, fr True/False, semua kandidat lama di bawah
  K2 task_cmd baru == HEAD untuk tiap tugas (move True dan DRY --plan-check)
  K3 argv retract / traverse run_g54 (rr_cmd / traverse_cmd, t1 False) == ekspresi inline HEAD
  K4 _rest_line + plan_retract (target None, strict None) == HEAD pada keadaan ujung tugas tersimpan (verdict, segmen, log)
  K5 kandidat TAS mode T1: urutan event = A0.4 (fold, retract T1, traverse, tugas retract_target T1, retract T1, pulang, unfold)
  K6 g64c_strict vs solver: garis lipat di p0 (= g64c_cert) dan garis tugas TAS T1 -> q (= screen_line_T): tau_excess sama
  K7 run_g32 menolak kandidat mode T1 (rc 2) sebelum menyentuh ROS

    python3 g64c_k3.py
"""
import importlib.util
import json
import math
import re
import os
import subprocess
import sys
import warnings

import numpy as np

warnings.simplefilter('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
SCR = '/home/user1/Documents/ceiling_arm/scripts'
for d in (SCR, os.path.join(R, 'p1_g31'), os.path.join(R, 'p1_g32'), os.path.join(R, 'p1_g54'), os.path.join(R, 'p1_g22')):
    sys.path.insert(0, d)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


import g31_screen as G31N  # noqa: E402
import run_g32 as R32N  # noqa: E402
import run_g54 as R54N  # noqa: E402
import return_rest as RRN  # noqa: E402
G31O = load('g31_screen_head', os.path.join(HERE, 'before/g31_screen_head.py'))
R32O = load('run_g32_head', os.path.join(HERE, 'before/run_g32_head.py'))
RRO = load('return_rest_head', os.path.join(HERE, 'before/return_rest_head.py'))
P = G31N.P
CANDS = [('p1_g55/g55_selected.json', 'R90sel'), ('p1_g55/g55_selected.json', 'R35sel'),
         ('p1_g52/g52_candidates.json', 'R90'), ('p1_g43/g43_candidates.json', 'R180'),
         ('p1_g32/g32_candidates.json', 'R10'), ('p1_demo_sys/run_synth/demo_candidates.json', 'DEMO')]
bad = []


def rows():
    for f, v in CANDS:
        for r in json.load(open(os.path.join(R, f))):
            if r.get(v, {}).get('schedule'):
                yield f, v, r


# K1 + K2 + K3
n_ev = n_task = n_rr = n_tr = 0


class A:
    rot_max = None


for f, v, r in rows():
    p0 = {g: r['p0'][str(g)] for g in (1, 2)}
    for fr in (False, True):
        old = G31O.events(r[v]['schedule'], p0, fr)
        for mode in (None, {'1': 'REST', '2': 'REST'}, r[v].get('mode')):
            new = G31N.events(r[v]['schedule'], p0, fr, mode)
            if new != old:
                bad.append(f'K1 {f} {v} s{r["seed"]} fr {fr} mode {mode}')
        n_ev += len(old)
        for e in old:
            g = e['gantry']
            if e['kind'] == 'task':
                n_task += 1
                for kw in (dict(), dict(move=False, start={'x': 1.0}), dict(goal_tol=0.05, tau_max=14.0)):
                    if R32N.task_cmd(e, '/tmp/m.csv', **kw) != R32O.task_cmd(e, '/tmp/m.csv', **kw):
                        bad.append(f'K2 {f} s{r["seed"]} {e["arm"]} {kw}')
            elif e['kind'] == 'retract':
                n_rr += 1
                if R54N.rr_cmd(g, e) != ['python3', '-u', 'return_rest.py', '--arms', *P.ARMS[g], '--move', '--no-plan']:
                    bad.append(f'K3 retract {f} s{r["seed"]}')
            else:
                n_tr += 1
                for rm in (None, 125.0):
                    a = A()
                    a.seed, a.variant, a.plan, a.rot_max = r['seed'], v, '/x/plan.json', rm
                    old_cmd = ['python3', '-u', os.path.join(R54N.R, 'p1_g32/pose_to_g.py'), '--gantry', str(g),
                               '--seed', str(a.seed), '--plan', a.plan, '--variant', a.variant,
                               f'{e["to"][0]:.3f}', f'{math.degrees(e["to"][1]) + (0.0):.3f}', '--move'] + \
                        (['--rot-max', f'{a.rot_max:g}'] if a.rot_max is not None else [])
                    if R54N.traverse_cmd(g, e, a, a.plan, 0.0, False) != old_cmd:
                        bad.append(f'K3 traverse {f} s{r["seed"]}')
print(f'K1-K3: {n_ev} event (x2 fr), tugas {n_task}, retract {n_rr}, traverse {n_tr}; beda {len(bad)}', flush=True)

# K4
from interarm_collision import CrossGantryChecker, InterArmChecker  # noqa: E402
from env_collision import EnvChecker  # noqa: E402
env, cross = EnvChecker(), CrossGantryChecker()
same = {1: InterArmChecker(gantry='gantry_1'), 2: InterArmChecker(gantry='gantry_2')}
states = []
for fn in ('g40_planonly_plans.jsonl', 'g40e_planonly_plans.jsonl'):
    for ln in open(os.path.join(R, 'p1_g40', fn)):
        d = json.loads(ln)
        if d.get('task') is None or not d.get('traj'):
            continue
        st = dict(d['start'])
        st.update(zip(d['traj']['joint_names'], d['traj']['pos'][-1]))
        states.append((d['arm'], st))
        if len(states) >= 16:
            break
k4, ties = 0, []
for arm, st in states:
    g = P.GANTRY[arm] if hasattr(P, 'GANTRY') else RRN.GANTRY[arm]
    cur = [st[f'{RRN.PREFIX[x]}joint_{i}'] for x in P.ARMS[g] for i in range(1, 7)]
    if RRN._rest_line(cur) != RRO._rest_line(cur):
        bad.append(f'K4 _rest_line {arm}')
    k4 += 1
    for first in ('baru', 'HEAD'):        # both call orders: a tie in d_min (static pair) may report another waypoint
        la, lb = [], []
        if first == 'baru':
            va, sa = RRN.plan_retract(g, P.ARMS[g], st, same[g], cross, env, None, log=la.append)
            vb, sb = RRO.plan_retract(g, P.ARMS[g], st, same[g], cross, env, None, log=lb.append)
        else:
            vb, sb = RRO.plan_retract(g, P.ARMS[g], st, same[g], cross, env, None, log=lb.append)
            va, sa = RRN.plan_retract(g, P.ARMS[g], st, same[g], cross, env, None, log=la.append)
        strip = lambda L: [re.sub(r'di titik \d+/', 'di titik ?/', x) for x in L]  # noqa: E731
        if la != lb:
            ties.append((k4, arm, first))
        if va != vb or strip(la) != strip(lb) or \
                [(x[0], x[1], x[2], x[3]) for x in (sa or [])] != [(x[0], x[1], x[2], x[3]) for x in (sb or [])]:
            bad.append(f'K4 plan_retract {arm} ({first} dulu): {va} vs {vb}')
print(f'K4: {k4} keadaan x 2 urutan, plan_retract + _rest_line; beda total {len(bad)}; log beda HANYA indeks titik seri '
      f'{ties}', flush=True)

# K5
dc = json.load(open(os.path.join(HERE, 'run_c3/demo_candidates.json')))[0]
evs = G31N.events(dc['DEMO']['schedule'], {g: dc['p0'][str(g)] for g in (1, 2)}, True, dc['DEMO']['mode'])
seq = [(e['gantry'], e['kind'], e.get('target') or e.get('retract_target') or ('home' if e.get('home') else ''))
       for e in evs]
print('K5 event TAS:', seq)
want2 = [(2, 'fold', ''), (2, 'retract', 'T1'), (2, 'traverse', ''), (2, 'task', 'T1'), (2, 'retract', 'T1'),
         (2, 'traverse', 'home'), (2, 'unfold', '')]
if [x for x in seq if x[0] == 2] != want2 or [x[1] for x in seq if x[0] == 1] != ['task', 'retract']:
    bad.append(f'K5 urutan {seq}')
tr_home = [e for e in evs if e.get('home')][0]
if tr_home['to'] != (0.0, 0.0):
    bad.append(f'K5 pulang {tr_home["to"]}')
tk = [e for e in evs if e['kind'] == 'task' and e['gantry'] == 2][0]
c = R32N.task_cmd(tk, '/tmp/m.csv')
if c[-2:] != ['--retract-target', 'T1']:
    bad.append(f'K5 task_cmd {c[-3:]}')
print('K5 argv g2:', [' '.join(R54N.rr_cmd(2, e)[5:]) if e['kind'] in ('retract', 'fold', 'unfold') else e['kind']
                      for e in evs if e['gantry'] == 2])

# K6
sys.path.insert(0, HERE)
import g64c_strict as ST  # noqa: E402
import g64c_make as C  # noqa: E402
if [round(x, 5) for x in ST.T1] != RRN.T1:
    bad.append('K6 T1 return_rest != json')
if not np.allclose(ST.MARGIN, C.MARGIN) or not np.allclose(ST.OFFSET, C.OFFSET) or not np.allclose(ST.LIM, C.LIM):
    bad.append('K6 konstanta torsi beda dari solver')
cert = {(r['g'], r['rail'], r['other_rail']): r for r in json.load(open(os.path.join(HERE, 'g64c_cert.json')))['rows']}
for (g, lin, oth), row in cert.items():
    st = C.G4.G.base_joints(g, lin, 0.0)
    st[f't{3 - g}_linear_joint'] = oth
    names = [f'{RRN.PREFIX[x]}joint_{i}' for x in P.ARMS[g] for i in range(1, 7)]
    pts = RRN._rest_line([st[n] for n in names], target=RRN.T1)
    ok, info = ST.check(g, names, pts, st, env)
    print(f'K6 lipat g{g} {lin}: {info} | cert tau {row["fold"]["tau_excess"]}')
    if not ok or abs(info['tau_excess'] - row['fold']['tau_excess']) > 0.01:
        bad.append(f'K6 lipat g{g} {lin}')
st2 = dc['DEMO']['schedule']['2'][0]
goal = list(st2['goal'].values())[0]
names = sorted(goal)
q6 = [goal[n] for n in names]
lin, rot = st2['rail_m'], math.radians(st2['rot_deg'])
held = C.held_T(2, lin, rot)
pts = C.cos_line(C.T1, q6)
ok, info = ST.check(2, names, pts, held, env)
sl = C.screen_line_T(2, 'arm_3', lin, rot, np.array(q6), *C.G4.checkers())
print(f'K6 garis tugas TAS T1->q: strict {info} | solver tau {sl["tau_excess"]} kotak {sl.get("box_line_d")}')
if not ok or abs(info['tau_excess'] - sl['tau_excess']) > 1e-3:
    bad.append('K6 garis tugas')

# K7
pr = subprocess.run([sys.executable, os.path.join(R, 'p1_g32/run_g32.py'), '--seed', '0', '--launch-log', '/dev/null',
                     '--mon', '/dev/null', '--plan', os.path.join(HERE, 'run_c3/demo_candidates.json'), '--variant', 'DEMO',
                     '--out-dir', '/tmp/claude-g64c-k7', '--dry'], capture_output=True, text=True, timeout=60)
print(f'K7 run_g32 mode T1: rc {pr.returncode}: {pr.stdout.strip()[-140:]}')
if pr.returncode != 2 or 'REFUSE: G64-C' not in pr.stdout:
    bad.append('K7')

print(f"\nC5 {'✓' if not bad else '✗'} -- beda: {bad}")
