"""G25 step 1 -- per-component timing of the CALIBRATION set (G19, G20), no model.

Instrument, not model: every number is a difference of two logged timestamps on
the same wall clock (bash `date +%s.%N`, ROS log stamps, move_group stamps).

  per probe process  [Ts, Te] from gXX_windows.txt (runner stamps around the call)
    t_start   Ts -> first move_group 'Planning request received'
    t_end     last monitor event the probe waits for (success / concurrent) -> Te
  per plan attempt   (move_group P -> M, probe log R and X)
    t_plan    P 'Planning request received' -> M 'Motion plan was computed successfully'
    t_rnea    M -> R 'torsi RNEA+offset' line (probe)
    t_screen  R -> X 'jarak antar-lengan minimum' line (probe; first arm in a process
              also builds the pinocchio geometry here)
  per executed arm
    t_go      X -> S 'Starting trajectory execution'
    t_exec    S -> C 'Completed trajectory execution'
    t_succ    C -> monitor '>>> arm_k: SUCCESS' (first after C)
  per G19 move (runner stamps T0..T3 around each tool call)
    retract_call  T1 - T0  (return_rest --move, both arms of gantry 1)
    rail_call     T2 - T1  (rail_to --move)
  + g19_components.json (recorder-based: retract-to-REST, traverse, gaps) as given.

Writes g25_calib_components.json. OFFLINE.
"""
import gzip
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
TS = re.compile(r'\[(\d{10}\.\d+)\]')


def mg_events(path):
    op = gzip.open if path.endswith('.gz') else open
    ev = []
    for line in op(path, 'rt', errors='replace'):
        if 'move_group' not in line:
            continue
        m = TS.search(line)
        if not m:
            continue
        t = float(m.group(1))
        if 'Planning request received for MoveGroup' in line:
            ev.append((t, 'P'))
        elif 'Motion plan was computed successfully' in line:
            ev.append((t, 'M'))
        elif 'Starting trajectory execution' in line:
            ev.append((t, 'S'))
        elif 'Completed trajectory execution' in line:
            ev.append((t, 'C'))
        elif re.search(r'\[(ERROR|WARN)\].*move_action_capability', line):
            ev.append((t, 'F'))       # planning failure reported by move_group
    return ev


def probe_events(path):
    ev = []
    for line in open(path, errors='replace'):
        m = TS.search(line)
        if not m:
            continue
        t = float(m.group(1))
        if 'torsi RNEA+offset' in line:
            ev.append((t, 'R'))
        elif 'jarak antar-lengan minimum' in line:
            ev.append((t, 'X'))
        elif 'DITOLAK' in line or 'TABRAKAN' in line:
            ev.append((t, 'J'))       # plan rejected by a screen
    return ev


def monitor_success(path):
    out = []
    for line in open(path, errors='replace'):
        m = TS.search(line)
        if not m:
            continue
        a = re.search(r'>>> (arm_\d): SUCCESS', line)
        if a:
            out.append((float(m.group(1)), a.group(1)))
        elif 'CONCURRENT dwell held' in line:
            out.append((float(m.group(1)), 'CONC'))
    return out


def decompose_probe(src, trial, Ts, Te, mg, pr, mon, arms):
    """Walk one probe process. Arms execute in `arms` order (dual_trial order)."""
    E = sorted([e for e in mg if Ts <= e[0] <= Te] + [e for e in pr if Ts <= e[0] <= Te])
    plans, execs = [], []
    ai, cur, first_P, arm_first_P = 0, None, None, None
    for t, k in E:
        if k == 'P':
            first_P = first_P or t
            arm_first_P = arm_first_P or t
            cur = dict(src=src, trial=trial, arm=arms[ai] if ai < len(arms) else None,
                       arm_idx=ai, P=t, first_in_process=ai == 0)
            plans.append(cur)
        elif cur is not None and k in 'MRXJF' and k not in cur:
            cur[k] = t
        elif k == 'S' and cur is not None:
            cur['S'] = t
        elif k == 'C' and cur is not None and 'S' in cur:
            arm = arms[ai]
            succ = next((ts for ts, a in mon if a == arm and ts >= t and ts <= Te + 5), None)
            n_att = sum(1 for p in plans if p['arm_idx'] == ai)
            execs.append(dict(src=src, trial=trial, arm=arm, arm_idx=ai,
                              first_in_process=ai == 0, n_attempts=n_att,
                              t_armspan=round(cur['S'] - arm_first_P, 3),
                              t_go=round(cur['S'] - cur['X'], 3) if 'X' in cur else None,
                              t_exec=round(t - cur['S'], 3),
                              t_succ=round(succ - t, 3) if succ else None))
            ai += 1
            cur, arm_first_P = None, None
    for p in plans:
        p['t_plan'] = round(p['M'] - p['P'], 3) if 'M' in p else None
        p['t_rnea'] = round(p['R'] - p['M'], 3) if 'M' in p and 'R' in p else None
        p['t_screen'] = round(p['X'] - p['R'], 3) if 'R' in p and 'X' in p else None
        p['verdict'] = 'PLANNED' if 'X' in p else ('REJECTED' if 'J' in p else
                                                  ('NO-PLAN' if 'M' not in p else '?'))
        for k in 'PMRXJFS':
            p.pop(k, None)
    last = [ts for ts, a in mon if Ts <= ts <= Te]
    probe = dict(src=src, trial=trial, wall=round(Te - Ts, 3), n_arms=len(arms),
                 n_exec=len(execs),
                 t_start=round(first_P - Ts, 3) if first_P else None,
                 t_end=round(Te - max(last), 3) if last else None)
    return probe, plans, execs


def g19():
    W = [l.split() for l in open(os.path.join(R, 'p1_g19/g19_windows.txt'))]
    mg = mg_events(os.path.join(R, 'p1_g19/g19_launch_excerpt.log.gz'))
    mon = monitor_success(os.path.join(R, 'p1_g19/g19_monitor.log'))
    comp = {c['trial']: c for c in json.load(open(os.path.join(R, 'p1_g19/g19_components.json')))}
    P, PL, EX, MV = [], [], [], []
    for i, rail, t0, t1, t2, t3 in W:
        i = int(i)
        pr = probe_events(os.path.join(R, f'p1_g19/g19_trial{i}.log'))
        a, b, c = decompose_probe('g19', i, float(t2), float(t3), mg, pr, mon, ['arm_1', 'arm_2'])
        P.append(a); PL += b; EX += c
        if i > 0:
            cc = comp[i]
            MV.append(dict(trial=i, retract_call=round(float(t1) - float(t0), 3),
                           rail_call=round(float(t2) - float(t1), 3),
                           retract_to_rest=cc.get('retract'), traverse=cc.get('traverse'),
                           T_lin=cc.get('T_lin'), gap_retract_to_rail=cc.get('gap_retract_to_rail'),
                           gap_rail_to_task=cc.get('gap_rail_to_task'),
                           rail_end_mm=cc.get('rail_end_mm')))
    return P, PL, EX, MV


def g20():
    W = [l.split() for l in open(os.path.join(R, 'p1_g20/g20_windows.txt'))]
    mg = mg_events(os.path.join(R, 'p1_g20/g20_launch.log.gz'))
    mon = monitor_success(os.path.join(R, 'p1_g20/g20_monitor.log'))
    P, PL, EX = [], [], []
    for i, t0, t1 in W:
        i = int(i)
        pr = probe_events(os.path.join(R, f'p1_g20/g20_trial{i}.log'))
        a, b, c = decompose_probe('g20', i, float(t0), float(t1), mg, pr, mon,
                                  ['arm_1', 'arm_2', 'arm_3', 'arm_4'])
        P.append(a); PL += b; EX += c
    return P, PL, EX


if __name__ == '__main__':
    P19, PL19, EX19, MV19 = g19()
    P20, PL20, EX20 = g20()
    out = dict(probes=P19 + P20, plans=PL19 + PL20, execs=EX19 + EX20, moves_g19=MV19)
    json.dump(out, open(os.path.join(HERE, 'g25_calib_components.json'), 'w'), indent=1)
    for k in ('probes', 'execs', 'moves_g19'):
        print(f'== {k}')
        for r in out[k]:
            print('  ', r)
    print('== plans (verdict counts)', {v: sum(p['verdict'] == v for p in out['plans'])
                                        for v in ('PLANNED', 'REJECTED', 'NO-PLAN', '?')})
