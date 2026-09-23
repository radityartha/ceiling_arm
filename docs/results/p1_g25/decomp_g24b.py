"""G25 step 1b -- decomposition of the TEST set (G24b seed 1, 10 events). Run only
AFTER docs/p1_g25_task_cost.md A was locked (sha c080a44e, 11:40).

Stamps: runner prefix on every line of g24b_ev*.log and g24b_runner.log (wall clock
when the line reached the runner), ROS stamps inside probe lines, move_group
stamps in g24b_launch.log.gz, success@ from the runner (penilai event, t0-relative).

  task      t_start  CMD -> P          t_plan  P -> M        t_rnea  M -> R
            t_geom1  1st geometry build (R-line .. 2nd build warning)
            t_scr2   2nd build + screen compute (2nd warning .. X) -- NOT separable
            t_screen R -> X            t_go    X -> S         t_exec  S -> C
            t_succ   C -> success      t_end   success -> rc line
  retract   start (CMD -> 'lengan:'), screen ('lengan:' -> 'mengirim'),
            motion ('mengirim' -> 'selesai'), exit ('selesai' -> rc line)
  traverse  start (CMD -> S12), S24 (S23 -> S24 line), send (S24 -> t_send),
            t_traverse (tool json), settle (t_stop -> 'akhir'), exit ('akhir' -> rc)
  runner    gap = next event CMD - previous rc line
Writes g24b_components.json.
"""
import gzip
import json
import os
import re

import decomp_calib as DC

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, '../p1_g24')


def lines(path):
    out = []
    for l in open(path, errors='replace'):
        m = re.match(r'^#? ?(\d{10}\.\d+) ?(.*)$', l.rstrip('\n'))
        if m:
            out.append((float(m.group(1)), m.group(2)))
    return out


def runner():
    L = lines(os.path.join(D, 'g24b_runner.log'))
    t0 = float(re.search(r't0 (\d+\.\d+)', next(s for _, s in L if '===== t0' in s)).group(1))
    ev = {}
    for t, s in L:
        m = re.match(r'\s*\[(\d+)\] (task|retract|traverse)', s)
        if m:
            ev.setdefault(int(m.group(1)), {}).update(kind=m.group(2), start=t)
        m = re.match(r'\s*\[(\d+)\] rc=(\d+) ([\d.]+) s(?: success@([\d.]+))?', s)
        if m:
            e = ev[int(m.group(1))]
            e.update(end=t, rc=int(m.group(2)), dur=float(m.group(3)))
            if m.group(4):
                e['success'] = t0 + float(m.group(4))
    return t0, ev


def task(k, e, mg):
    L = lines(os.path.join(D, f'g24b_ev{k:02d}_task.log'))
    Ts, Te = L[0][0], e['end']
    ros = [(float(DC.TS.search(s).group(1)), s) for _, s in L if DC.TS.search(s)]
    R = next(t for t, s in ros if 'torsi RNEA' in s)
    X = next(t for t, s in ros if 'jarak antar-lengan' in s)
    W = [t for t, s in L if 'UserWarning' in s]
    j2 = float(re.search(r' 2=([\d.]+)->', next(s for _, s in ros if 'torsi RNEA' in s)).group(1))
    E = [x for x in mg if Ts <= x[0] <= Te]
    P = [t for t, c in E if c == 'P']
    M = [t for t, c in E if c == 'M']
    S = [t for t, c in E if c == 'S']
    C = [t for t, c in E if c == 'C']
    pj = json.load(open(os.path.join(D, f'g24b_ev{k:02d}_probe.json')))[0]
    r = dict(event=k, kind='task', wall=round(Te - Ts, 3), n_plan=len(P), n_geom=len(W),
             t_start=P[0] - Ts, t_plan=M[-1] - P[-1], t_rnea=R - M[-1],
             t_geom1=(W[1] - W[0]) if len(W) > 1 else None,
             t_scr2=(X - W[1]) if len(W) > 1 else None, t_screen=X - R,
             t_armspan=S[0] - P[0], t_go=S[0] - X, t_exec=C[0] - S[0],
             t_succ=e['success'] - C[0], t_end=Te - e['success'],
             to_success=e['success'] - Ts,
             tau_peak=pj['tau_peak'], rnea_j2=j2, verdict=pj['verdict'])
    return {a: (round(b, 3) if isinstance(b, float) else b) for a, b in r.items()}


def retract(k, e):
    L = lines(os.path.join(D, f'g24b_ev{k:02d}_retract.log'))
    t = {key: next(tt for tt, s in L if s.startswith(key)) for key in ('lengan:', 'mengirim', 'selesai')}
    Ts, Te = L[0][0], e['end']
    tau = float(re.search(r'torsi puncak ([\d.]+)', next(s for _, s in L if s.startswith('selesai'))).group(1))
    return dict(event=k, kind='retract', wall=round(Te - Ts, 3),
                start=round(t['lengan:'] - Ts, 3), screen=round(t['mengirim'] - t['lengan:'], 3),
                motion=round(t['selesai'] - t['mengirim'], 3), exit=round(Te - t['selesai'], 3),
                tau_peak=tau)


def traverse(k, e):
    L = lines(os.path.join(D, f'g24b_ev{k:02d}_traverse.log'))
    Ts, Te = L[0][0], e['end']
    g = lambda key: next(tt for tt, s in L if s.startswith(key))  # noqa: E731
    js = json.loads(next(s for _, s in L if s.startswith('{')))
    return dict(event=k, kind='traverse', wall=round(Te - Ts, 3), start=round(g('S12') - Ts, 3),
                s24=round(g('S24') - g('S23'), 3), send=round(js['t_send'] - g('S24'), 3),
                t_traverse=js['t_traverse'], settle=round(g('akhir') - js['t_stop'], 3),
                exit=round(Te - g('akhir'), 3), T_cmd=js['T_cmd'], T_lin=js['T_lin_model'],
                delta_m=round(abs(js['goal'] - js['start']), 4),
                overhead=round(Te - Ts - js['t_traverse'], 3))


if __name__ == '__main__':
    t0, ev = runner()
    mg = DC.mg_events(os.path.join(D, 'g24b_launch.log.gz'))
    rows = []
    for k in sorted(ev):
        e = ev[k]
        rows.append({'task': task, 'retract': lambda k, e, mg: retract(k, e),
                     'traverse': lambda k, e, mg: traverse(k, e)}[e['kind']](k, e, mg))
        rows[-1]['runner_dur'] = e['dur']
        rows[-1]['start_rel'] = round(e['start'] - t0, 3)
    gaps = [round(ev[k + 1]['start'] - ev[k]['end'], 3) for k in sorted(ev) if k + 1 in ev]
    first_gap = round(ev[0]['start'] - t0, 3)
    out = dict(t0=t0, events=rows, runner_gaps=gaps, t0_to_first_cmd=first_gap,
               makespan=round(ev[max(ev)]['success'] - t0, 3))
    json.dump(out, open(os.path.join(HERE, 'g24b_components.json'), 'w'), indent=1)
    for r in rows:
        print(r)
    print('runner gaps', gaps, 't0->first', first_gap, 'makespan', out['makespan'])
