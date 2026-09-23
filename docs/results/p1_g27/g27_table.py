"""G27 step 1 -- INSTRUMENT: every task plan in the g22/g23/g24/g26 (iv) screens and every
executed task (G24b, G26), joined to its ORACLE''' tuple. No rule is chosen here.

Per plan: z, rail, arm, verdict, live RNEA joint_2 (raw, before +7.7), oracle''' static bound
for joint_2 (14 - 7.7 - m2 = 5.64), oracle''' tuple (n_sol, saturated, tilt_fail, taumax j2, ok).
Tuple source, in order: g24_oracle3_cache.jsonl -> g24_validate3.json V/W -> computed now with
oracle2.solve(tilt=True, envelope=True) UNCHANGED (flagged 'g27'). OFFLINE, zero hardware.

    python3 g27_table.py  -> g27_table.json + printed table
"""
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g24'))
import make_instance_g24 as MI4  # noqa: E402
import oracle2 as O2  # noqa: E402

REF = MI4.REF
ARM_GS = {v: k for k, v in MI4.SLOT_ARM.items()}
BOUND2 = 14.0 - 7.7 - MI4.MARGIN[1]
TASK = re.compile(r'task t(\d+) (arm_\d) -> \[([-\d.]+), ([-\d.]+), ([-\d.]+)\] @rail ([\d.]+): ([A-Z-]+)')
TORQ = re.compile(r'torsi RNEA\+offset.*?2=([\d.]+)->')
SEED = re.compile(r'seed (\d+) sampel (\d+)/')


def node_of(xyz):
    d = np.linalg.norm(REF.nodes - np.asarray(xyz), axis=1)
    i = int(np.argmin(d))
    return i if d[i] < 2e-3 else None


RETR = re.compile(r'retract g(\d)')
GARMS = {1: ('arm_1', 'arm_2'), 2: ('arm_3', 'arm_4')}


def parse_screen(path, src):
    """start = 'REST' or the task whose plan end the arm starts from (sched_screen.walk placement)."""
    rows, pend, seed, samp, last = [], None, None, None, {}
    for ln in open(path):
        m = SEED.search(ln)
        if m:
            seed, samp, last = int(m.group(1)), int(m.group(2)), {}
            continue
        m = RETR.search(ln)
        if m and 'task' not in ln:
            for a in GARMS[int(m.group(1))]:
                last.pop(a, None)
            continue
        m = TORQ.search(ln)
        if m:
            pend = float(m.group(1))
            continue
        m = TASK.search(ln)
        if m:
            t, arm, x, y, z, rail, v = m.groups()
            rows.append(dict(src=src, seed=seed, sample=samp, task=int(t), arm=arm,
                             xyz=[float(x), float(y), float(z)], rail=float(rail), verdict=v,
                             rnea2=pend if v in ('PLANNED', 'TORQUE-UNSAFE') else None,
                             start=last.get(arm, 'REST')))
            last[arm] = f't{t}'
            pend = None
    return rows


def parse_exec():
    """Executed tasks: live RNEA from the task log, measured peak from probe json.
    Rail from the passing seed's screen line (same task, same arm)."""
    out = []
    sets = [('G24b', sorted(glob.glob(os.path.join(R, 'p1_g24/g24b_ev*_task.log'))), 'p1_g24/g24_screen.log', 1)]
    for s in (1, 2, 13):
        sets.append((f'G26 s{s}', sorted(glob.glob(os.path.join(R, f'p1_g26/g26_s{s}_ev*_task.log'))),
                     'p1_g26/g26_screen.log', s))
    for src, logs, scr, seed in sets:
        rails = {(r['task'], r['arm']): r['rail'] for r in parse_screen(os.path.join(R, scr), '') if r['seed'] == seed}
        for lg in logs:
            txt = open(lg).read()
            m = re.search(r'--arms (arm_\d) --target=([-\d.]+),([-\d.]+),([-\d.]+)', txt)
            arm, xyz = m.group(1), [float(m.group(i)) for i in (2, 3, 4)]
            rn = TORQ.findall(txt)
            pj = json.load(open(lg.replace('_task.log', '_probe.json')))[0]
            # task id: match xyz in screen rows of that seed
            cand = [r for r in parse_screen(os.path.join(R, scr), '')
                    if r['seed'] == seed and r['arm'] == arm and np.allclose(r['xyz'], xyz, atol=2e-3)]
            t = cand[0]['task']
            out.append(dict(src=src, seed=seed, sample='exec', task=t, arm=arm, xyz=xyz, start=cand[0]['start'],
                            rail=rails[(t, arm)], verdict=pj['verdict'],
                            rnea2=float(rn[0]) if rn else None, tau_meas2=pj.get('tau_peak')))
    return out


def main():
    rows = []
    for src, p in (('G22', 'p1_g22/g22_screen.log'), ('G23', 'p1_g23/g23_screen.log'),
                   ('G24b', 'p1_g24/g24_screen.log'), ('G26', 'p1_g26/g26_screen.log')):
        rows += parse_screen(os.path.join(R, p), src)
    rows += parse_exec()

    cache = MI4.load_cache()
    v3 = json.load(open(os.path.join(R, 'p1_g24/g24_validate3.json')))
    vw = {(r['arm'], tuple(r['xyz']), r['rail']): r for r in v3['V'] + v3['W']}
    computed = {}
    for r in rows:
        n = node_of(r['xyz'])
        g, s = ARM_GS[r['arm']]
        p = int(np.argmin(np.abs(REF.lin - r['rail'])))
        r.update(node=n, g=g, p=p, s=s, z=r['xyz'][2])
        key = (n, g, p, s)
        o, how = None, None
        if key in cache:
            o, how = cache[key], 'cache'
        elif (r['arm'], tuple(r['xyz']), r['rail']) in vw:
            o, how = vw[(r['arm'], tuple(r['xyz']), r['rail'])], 'validate3'
        else:
            if key not in computed:
                res = O2.solve(REF.nodes[n] if n is not None else r['xyz'], float(REF.lin[p]) if n is not None
                               else r['rail'], r['arm'], tilt=True, envelope=True)
                computed[key] = dict(n_sol=res['n_sol'], saturated=res['saturated'],
                                     taumax=res['taumax'].tolist(), tilt_fail=res['tilt_fail'],
                                     ok=bool(O2.torq2(res, MI4.MARGIN)))
            o, how = computed[key], 'g27'
        r.update(o_src=how, n_sol=o['n_sol'], n_roll=o.get('n_roll'), saturated=o['saturated'], tilt_fail=o.get('tilt_fail'),
                 o_j2=o['taumax'][1] if o['n_sol'] else None,
                 o_ok=bool(o.get('ok', o.get('torq', False))))
        r['gap2'] = (r['rnea2'] - r['o_j2']) if (r['rnea2'] is not None and r['o_j2'] is not None) else None
    json.dump(dict(bound2=BOUND2, rows=rows), open(os.path.join(HERE, 'g27_table.json'), 'w'), indent=1)

    print(f'batas statis oracle j2 = 14 - 7.7 - m2 = {BOUND2:.4f};  RNEA plan ditolak bila > 6.30')
    hdr = f"{'src':8s} {'sd':>3s} {'sm':>4s} {'t':>2s} {'arm':5s} {'z':>5s} {'rail':>5s} {'verdict':14s} " \
          f"{'start':>5s} {'RNEA2':>6s} {'o_j2':>6s} {'gap':>6s} {'o_ok':>5s} {'n_sol':>5s} {'sat':>4s} {'tfail':>5s} {'nroll':>5s} src"
    print(hdr)
    for r in sorted(rows, key=lambda r: (-r['z'], r['src'], r['seed'], str(r['sample']), r['task'])):
        f = lambda v, w=6: f'{v:{w}.2f}' if isinstance(v, float) else f"{'-':>{w}s}"  # noqa: E731
        print(f"{r['src']:8s} {r['seed']:3d} {str(r['sample']):>4s} {r['task']:2d} {r['arm']:5s} {r['z']:5.2f} "
              f"{r['rail']:5.2f} {r['verdict']:14s} {r['start']:>5s} {f(r['rnea2'])} {f(r['o_j2'])} {f(r['gap2'])} "
              f"{str(r['o_ok']):>5s} {r['n_sol']:5d} {str(r['saturated'])[0]:>4s} "
              f"{str(r['tilt_fail']):>5s} {str(r['n_roll']):>5s} {r['o_src']}")


if __name__ == '__main__':
    main()
