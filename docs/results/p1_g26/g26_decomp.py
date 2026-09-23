"""G26 A4-5: decomp_g24b.py (g25) adapted -- archive prefix g26_s<S>_, live launch log,
plus a SKIPPED-retract branch (return_rest printed 'sudah di rest', no 'mengirim').
Then measured vs the LOCKED prediction g26_predict_locked.json, per event.

    python3 g26_decomp.py S [--launch-log /tmp/g26_t1.log]  -> g26_s<S>_components.json
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g25'))
import decomp_calib as DC  # noqa: E402
import decomp_g24b as B  # noqa: E402


def runner(pfx):
    L = B.lines(os.path.join(HERE, f'{pfx}runner.log'))
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


def task(pfx, k, e, mg):
    """decomp_g24b.task verbatim, file names from pfx."""
    L = B.lines(os.path.join(HERE, f'{pfx}ev{k:02d}_task.log'))
    Ts, Te = L[0][0], e['end']
    ros = [(float(DC.TS.search(s).group(1)), s) for _, s in L if DC.TS.search(s)]
    R = [t for t, s in ros if 'torsi RNEA' in s][-1]      # last attempt (the one executed)
    X = [t for t, s in ros if 'jarak antar-lengan' in s][-1]
    W = [t for t, s in L if 'UserWarning' in s]
    j2 = float(re.search(r' 2=([\d.]+)->', [s for _, s in ros if 'torsi RNEA' in s][-1]).group(1))
    E = [x for x in mg if Ts <= x[0] <= Te]
    P = [t for t, c in E if c == 'P']
    M = [t for t, c in E if c == 'M']
    S = [t for t, c in E if c == 'S']
    C = [t for t, c in E if c == 'C']
    pj = json.load(open(os.path.join(HERE, f'{pfx}ev{k:02d}_probe.json')))[0]
    r = dict(event=k, kind='task', wall=round(Te - Ts, 3), n_plan=len(P), n_geom=len(W),
             t_start=P[0] - Ts, t_plan=M[-1] - P[-1], t_rnea=R - M[-1],
             t_screen=X - R, t_armspan=S[0] - P[0], t_go=S[0] - X, t_exec=C[0] - S[0],
             t_succ=e['success'] - C[0], t_end=Te - e['success'],
             to_success=e['success'] - Ts,
             tau_peak=pj['tau_peak'], rnea_j2=j2, verdict=pj['verdict'])
    return {a: (round(b, 3) if isinstance(b, float) else b) for a, b in r.items()}


def traverse(pfx, k, e):
    """decomp_g24b.traverse verbatim, file names from pfx."""
    L = B.lines(os.path.join(HERE, f'{pfx}ev{k:02d}_traverse.log'))
    Ts, Te = L[0][0], e['end']
    g = lambda key: next(tt for tt, s in L if s.startswith(key))  # noqa: E731
    js = json.loads(next(s for _, s in L if s.startswith('{')))
    return dict(event=k, kind='traverse', wall=round(Te - Ts, 3), start=round(g('S12') - Ts, 3),
                s24=round(g('S24') - g('S23'), 3), send=round(js['t_send'] - g('S24'), 3),
                t_traverse=js['t_traverse'], settle=round(g('akhir') - js['t_stop'], 3),
                exit=round(Te - g('akhir'), 3), T_cmd=js['T_cmd'], T_lin=js['T_lin_model'],
                delta_m=round(abs(js['goal'] - js['start']), 4),
                overhead=round(Te - Ts - js['t_traverse'], 3))


def retract_full(pfx, k, e):
    """decomp_g24b.retract verbatim, file names from pfx."""
    L = B.lines(os.path.join(HERE, f'{pfx}ev{k:02d}_retract.log'))
    t = {key: next(tt for tt, s in L if s.startswith(key)) for key in ('lengan:', 'mengirim', 'selesai')}
    Ts, Te = L[0][0], e['end']
    tau = float(re.search(r'torsi puncak ([\d.]+)', next(s for _, s in L if s.startswith('selesai'))).group(1))
    return dict(event=k, kind='retract', wall=round(Te - Ts, 3),
                start=round(t['lengan:'] - Ts, 3), screen=round(t['mengirim'] - t['lengan:'], 3),
                motion=round(t['selesai'] - t['mengirim'], 3), exit=round(Te - t['selesai'], 3),
                tau_peak=tau)


def retract(pfx, k, e):
    L = B.lines(os.path.join(HERE, f'{pfx}ev{k:02d}_retract.log'))
    if any('sudah di rest' in s for _, s in L):
        Ts, Te = L[0][0], e['end']
        t_l = next(tt for tt, s in L if s.startswith('lengan:'))
        return dict(event=k, kind='retract', skipped=True, wall=round(Te - Ts, 3),
                    start=round(t_l - Ts, 3), exit=round(Te - t_l, 3))
    r = retract_full(pfx, k, e)
    r['skipped'] = False
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seed', type=int)
    ap.add_argument('--launch-log', default='/tmp/g26_t1.log')
    a = ap.parse_args()
    pfx = f'g26_s{a.seed}_'
    t0, ev = runner(pfx)
    mg = DC.mg_events(a.launch_log)
    pred = {e['event']: e for e in json.load(open(os.path.join(HERE, 'g26_predict_locked.json')))[str(a.seed)]['events']}
    rows = []
    for k in sorted(ev):
        e = ev[k]
        if e['kind'] == 'task':
            r = task(pfx, k, e, mg)
        elif e['kind'] == 'retract':
            r = retract(pfx, k, e)
        else:
            r = traverse(pfx, k, e)
        r.update(runner_dur=e['dur'], start_rel=round(e['start'] - t0, 3))
        last = k == max(ev)
        meas = r.get('to_success') if (last and e['kind'] == 'task') else r['wall']
        r.update(pred=round(pred[k]['pred'], 3), meas=meas, err=round(pred[k]['pred'] - meas, 3),
                 err_pct=round(100 * (pred[k]['pred'] - meas) / meas, 2) if meas else None)
        rows.append(r)
    gaps = [round(ev[k + 1]['start'] - ev[k]['end'], 3) for k in sorted(ev) if k + 1 in ev]
    ms = round(ev[max(ev)]['success'] - t0, 3)
    P = json.load(open(os.path.join(HERE, 'g26_predict_locked.json')))[str(a.seed)]
    out = dict(seed=a.seed, t0=t0, events=rows, runner_gaps=gaps, t0_to_first_cmd=round(ev[0]['start'] - t0, 3),
               makespan=ms, pred_serial=P['makespan_serial'], err=round(P['makespan_serial'] - ms, 3),
               err_pct=round(100 * (P['makespan_serial'] - ms) / ms, 2))
    json.dump(out, open(os.path.join(HERE, f'{pfx}components.json'), 'w'), indent=1)
    for r in rows:
        print(f"[{r['event']:2d}] {r['kind']:8s} pred {r['pred']:7.2f} terukur {r['meas']:7.2f} galat {r['err']:+7.2f} "
              f"({r['err_pct']:+6.1f} %)  " + ', '.join(f'{x}={r[x]}' for x in
                                                   ('t_start', 't_screen', 't_exec', 't_succ', 'tau_peak', 'rnea_j2',
                                                    'skipped', 'screen', 'motion', 't_traverse', 'T_cmd', 'overhead')
                                                   if x in r))
    print(f"makespan terukur {ms:.2f} vs P1'-serial {P['makespan_serial']:.2f} -> galat {out['err']:+.2f} s "
          f"({out['err_pct']:+.2f} %); sela runner Σ {sum(gaps):.2f}, t0->pertama {out['t0_to_first_cmd']}")


if __name__ == '__main__':
    main()
