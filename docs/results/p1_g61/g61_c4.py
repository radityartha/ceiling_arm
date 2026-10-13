"""G61 C4 (docs/p1_g61_offline.md A4): where does G57 HW time go vs the g54_sim model (P1'' costs, G53) and vs the mock?
(1) every task event: probe-log phases  start (CMD -> 'probe LANGKAH'), plan+screens (-> reservation CLEAR; last attempt's
plan / screens split at its 'ujung rencana' line), exec (CLEAR -> 'moveit MOVED'), dwell (MOVED -> t_success), for HW
(p1_g57/hw) and the mock smoke of the same seed + variant (p1_g54/smoke, G55/G58/G60); (2) every traverse: HW wall,
screen (t_send - t_start), motion (t_jtc_done - t_send), closing vs the g54_sim cost l_bar + g x G25 model; (3) per run
the sums against the sim makespan. HW + mock logs, OFFLINE.

    python3 g61_c4.py [g61_c3_torque.json]   -> g61_c4.json + tables
"""
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g53'))
import g53_p1p as M53  # noqa: E402

K = json.load(open(os.path.join(R, 'p1_g53/g53_tests.json')))['calib_all']
HW = os.path.join(R, 'p1_g57', 'hw')
SM = os.path.join(R, 'p1_g54', 'smoke')
# HW run -> (mock prefix, sim makespan) -- sim values = g57_report.RUNS
RUNS = {'s7_R180': ('g58_s7L', 396.91), 's7_R90sel': ('g58_s7B', 283.62), 's10_R90sel': ('g58_s10B', 278.50),
        's10_G42': ('g58_s10L', 360.31), 's0_R90sel': ('g55_s0', 148.16), 's8_R90sel': ('g58_s8B', 264.44),
        's8_G42': ('g58_s8L', 409.42), 's18_R180': ('g58_s18L', 372.16), 's18_G60': ('g60_s18', 232.21),
        's3_G60': ('g60_s3', 295.80), 's3_R180': ('g58_s3L', 426.08), 's12_R90': ('g58_s12', 313.49),
        's12_G60': ('g60_s12', 270.70)}
TS = re.compile(r'^(?:# )?([0-9]{10}\.[0-9]+) ')


def phases(logf, t_success, t_end):
    t = dict(cmd=None, lang=None, plans=[], clear=None, moved=None, tries=None, ret=None, nres=None)
    for ln in open(logf, errors='ignore'):
        m = TS.match(ln)
        if not m:
            continue
        ts = float(m.group(1))
        if ' CMD ' in ln and t['cmd'] is None:
            t['cmd'] = ts
        elif 'probe LANGKAH' in ln:
            t['lang'] = ts
        elif 'ujung rencana vs target' in ln or ('torsi RNEA+offset' in ln and not t['plans']):
            t['plans'].append(ts)
        elif 'penyaring lingkungan:' in ln:
            t['ret'] = ts                                  # last line of the retract screen = reservation screen starts
        elif 'G54 reservasi g' in ln and '-> CLEAR' in ln:
            t['clear'] = ts
            t['nres'] = int(re.search(r'reservasi g[12]: ([0-9]+) keadaan', ln).group(1))
        elif 'moveit MOVED' in ln:
            t['moved'] = ts
            mt = re.search(r"percobaan rencana: \[(.*)\]", ln)
            t['tries'] = mt.group(1).count("'") // 2 if mt else None
    if None in (t['cmd'], t['lang'], t['clear'], t['moved']) or t_success is None:
        return None
    pl = [x for x in t['plans'] if x <= t['clear']]
    last = pl[-1] if pl else t['lang']
    return dict(start=t['lang'] - t['cmd'], plan_screen=t['clear'] - t['lang'], screens_last=t['clear'] - last,
                exec=t['moved'] - t['clear'], dwell=t_success - t['moved'], tail=t_end - t_success,
                total=t_success - t['cmd'], tries=t['tries'], reserve=None if t['ret'] is None else t['clear'] - t['ret'],
                n_res=t['nres'])


def run_events(fn):
    return json.load(open(fn))['events'] if os.path.exists(fn) else []


def main():
    c3 = {}
    if len(sys.argv) > 1:
        for r in json.load(open(sys.argv[1]))['rows']:
            c3[(r['run'], r['idx'])] = r
    out = dict(task=[], traverse=[], runs={})
    for run, (mpre, sim) in RUNS.items():
        hw = run_events(os.path.join(HW, f'p1hw_par_{run}_run.json'))
        mk = {e['idx']: e for e in run_events(os.path.join(SM, f'{mpre}_run.json'))}
        rj = json.load(open(os.path.join(HW, f'p1hw_par_{run}_run.json')))
        sums = dict(task_hw=0.0, task_model=0.0, trav_hw=0.0, trav_model=0.0, n_task=0, n_trav=0)
        for e in hw:
            if e['kind'] == 'task':
                ph = phases(os.path.join(HW, f'p1hw_par_{run}_ev{e["idx"]:02d}_task.log'), e.get('t_success'), e['t_end'])
                me = mk.get(e['idx'])
                pm = phases(os.path.join(SM, f'{mpre}_ev{e["idx"]:02d}_task.log'), me.get('t_success'), me['t_end']) \
                    if me else None
                rr = c3.get((run, e['idx']), {})
                out['task'].append(dict(run=run, idx=e['idx'], arm=e['arm'], xyz=e['xyz'], hw=ph, mock=pm,
                                        rec_exec=rr.get('exec_s'), rec_move_rel=rr.get('t_move_rel')))
                if ph:
                    sums['task_hw'] += ph['total']
                    sums['task_model'] += K['a_t']
                    sums['n_task'] += 1
            elif e['kind'] == 'traverse' and e.get('rail'):
                rl = e['rail']
                model = K['l_bar'] + K['g'] * M53.model(e, True)[1]
                d = dict(run=run, idx=e['idx'], g=e['gantry'], wall=e['t_end'] - e['t_start'],
                         screen=rl['t_send'] - e['t_start'], motion=rl['t_jtc_done'] - rl['t_send'],
                         close=e['t_end'] - rl['t_jtc_done'], T_cmd=rl['T_cmd'], model=model,
                         model_motion=K['g'] * M53.model(e, True)[1])
                out['traverse'].append(d)
                sums['trav_hw'] += d['wall']
                sums['trav_model'] += model
                sums['n_trav'] += 1
        sums.update(makespan=rj.get('makespan'), sim=sim)
        out['runs'][run] = sums
    tr = out['traverse']
    out['summary'] = dict(
        traverse_n=len(tr), traverse_excess_mean=float(np.mean([d['wall'] - d['model'] for d in tr])),
        screen_mean=float(np.mean([d['screen'] for d in tr])), screen_p50=float(np.median([d['screen'] for d in tr])),
        motion_over_model_p50=float(np.median([d['motion'] / d['model_motion'] for d in tr if d['model_motion'] > 1])),
        motion_over_Tcmd_p50=float(np.median([d['motion'] / d['T_cmd'] for d in tr if d['T_cmd'] > 1])),
        close_mean=float(np.mean([d['close'] for d in tr])))
    print('TRAVERSE (HW): run ev g | wall = layar + gerak + tutup | T_cmd | model sim (layar 12.61 + gerak)')
    for d in tr:
        print(f'  {d["run"]:11s} {d["idx"]:2d} g{d["g"]} | {d["wall"]:6.2f} = {d["screen"]:5.2f} + {d["motion"]:6.2f} + '
              f'{d["close"]:4.2f} | {d["T_cmd"]:6.2f} | {d["model"]:6.2f} ({d["wall"] - d["model"]:+6.2f})')
    print('RUN: makespan HW - sim | Σtraverse HW - model (n) | Σtugas HW - 29.04·n (n)')
    for run, s in out['runs'].items():
        mk = s['makespan']
        dm = '--' if mk is None else f'{mk - s["sim"]:+7.2f}'
        print(f'  {run:11s} {dm} | {s["trav_hw"] - s["trav_model"]:+7.2f} '
              f'({s["n_trav"]}) | {s["task_hw"] - s["task_model"]:+7.2f} ({s["n_task"]})')
    print('TUGAS s0 (HW | mock): mulai, rencana+layar (layar terakhir), eksekusi, dwell = total; percobaan; gerak perekam')
    for t in out['task']:
        if t['run'] != 's0_R90sel':
            continue
        f = lambda p: '--' if p is None else (f'{p["start"]:4.1f} {p["plan_screen"]:5.1f} ({p["screens_last"]:4.1f}) '  # noqa
                                               f'{p["exec"]:5.1f} {p["dwell"]:4.1f} = {p["total"]:5.1f} [{p["tries"]}]')
        print(f'  ev {t["idx"]:2d} {t["arm"]} | {f(t["hw"])} | {f(t["mock"])} | rec {t["rec_exec"]}')
    for src in ('hw', 'mock'):
        xs = [(t[src]['n_res'], t[src]['reserve']) for t in out['task'] if t[src] and t[src]['reserve'] is not None]
        n, y = np.array(xs, float).T
        b, a0 = np.polyfit(n, y, 1)
        out['summary'][f'reserve_fit_{src}'] = dict(n=len(xs), a=float(a0), b_per_state=float(b),
                                                     r=float(np.corrcoef(n, y)[0, 1]), max=float(y.max()), n_max=int(n.max()))
    out['summary']['screens_other_hw_p50'] = float(np.median([t['hw']['screens_last'] - t['hw']['reserve'] for t in out['task']
                                                              if t['hw'] and t['hw']['reserve'] is not None]))
    print(json.dumps(out['summary'], indent=1))
    json.dump(out, open(os.path.join(HERE, 'g61_c4.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
