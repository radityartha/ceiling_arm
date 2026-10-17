"""G64 0b (docs/p1_g64_joint_plan.md): every (probe prediction, measured peak) pair on REAL hardware. OFFLINE.

Sources
  * single-task probe logs (docs/results/**/*.log): the LAST 'torsi RNEA+offset' line before 'moveit MOVED' = the
    executed plan's screen; 'torsi puncak X N.m' = the probe's own /joint_states watch over move + dwell (max |eff|
    over the arm's joints). Mock logs are dropped: path contains mock/smoke, or measured peak < 1.0 N.m (fake HW
    reports ~0 effort) -- the second rule is printed per file so a dropped HW log is visible.
  * G57 parallel runs: G61 C3 (docs/results/p1_g61/g61_c3_torque.json, 87 tasks, j2 peak from the CSV per task).
  * g18-g20: G21 torque_offsets (docs/results/p1_g21/g21_torque_offsets.json, j2 peak from the CSV per plan).
Output g64_torque_pairs.json: rows (source, arm, j2 RNEA plan, j2 predicted = RNEA + 7.7, measured peak, residual
measured - RNEA) + per-arm p50/p95/max of the residual and the re-screen of TORQUE-UNSAFE refusals.

    python3 g64_torque_pairs.py
"""
import glob
import json
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
RE_TQ = re.compile(r'torsi RNEA\+offset\(per-aktuator\) vs rating: (.*?)\s+\(terketat (t\d_a\d)_joint')
RE_J = re.compile(r'(\d)=([0-9.]+)->([0-9.]+)/(\d+)')
RE_PEAK = re.compile(r'torsi puncak ([0-9.]+) N\.m')
ARM_OF = {'t1_a1': 'arm_1', 't1_a2': 'arm_2', 't2_a1': 'arm_3', 't2_a2': 'arm_4'}
OFF2 = 7.7


def from_logs():
    rows, dropped = [], []
    for fn in sorted(glob.glob(os.path.join(R, '**', '*.log'), recursive=True)):
        rel = os.path.relpath(fn, R)
        if re.search(r'mock|smoke', rel, re.I) or rel.startswith('p1_g57') or rel.startswith('p1_g64'):
            continue
        try:
            txt = open(fn, errors='replace').read()
        except OSError:
            continue
        if 'moveit MOVED' not in txt or 'torsi puncak' not in txt:
            continue
        last, moved = None, []
        for ln in txt.splitlines():
            m = RE_TQ.search(ln)
            if m:
                j = {int(a): (float(b), float(c)) for a, b, c, _ in RE_J.findall(m.group(1))}
                last = (ARM_OF[m.group(2)], j)
            elif 'moveit MOVED' in ln and last is not None:
                moved.append(last)
                last = None
            else:
                p = RE_PEAK.search(ln)
                if p and moved:
                    arm, j = moved[-1]
                    meas = float(p.group(1))
                    row = dict(source='log', file=rel, arm=arm, j2_rnea=j[2][0], j2_pred=round(j[2][0] + OFF2, 3),
                               meas_peak=meas, resid=round(meas - j[2][0], 3), n_moved_in_file=None)
                    (rows if meas >= 1.0 else dropped).append(row)
                    moved = []
    return rows, dropped


def from_g61():
    d = json.load(open(os.path.join(R, 'p1_g61', 'g61_c3_torque.json')))
    out = []
    for r in d['rows']:
        if not r.get('moved') or not r.get('probe_rnea'):
            continue
        out.append(dict(source='G57(G61 C3)', file=f"{r['run']} ev {r['idx']}", arm=r['arm'], j2_rnea=r['probe_rnea'][1],
                        j2_pred=r['probe_corr'][1], meas_peak=r['peak_j2'], resid=round(r['peak_j2'] - r['probe_rnea'][1], 3),
                        phase=r.get('phase'), hold_j2=r.get('hold_j2'), start_rest=r.get('start_rest')))
    return out


def from_g21():
    """g18-g20: G21 torque_offsets (joint_2 peak over the probe window from the CSV vs the executed plan's RNEA)."""
    d = json.load(open(os.path.join(R, 'p1_g21', 'g21_torque_offsets.json')))
    return [dict(source=f"g18-g20({r['sess']})", file=f"{r['sess']} {r['log']}", arm=r['arm'], j2_rnea=r['rnea'],
                 j2_pred=round(r['rnea'] + OFF2, 3), meas_peak=r['meas'], resid=round(r['meas'] - r['rnea'], 3))
            for r in d['rows'] if r.get('meas') is not None]


def stats(v):
    v = np.asarray(v, float)
    return dict(n=int(len(v)), p50=round(float(np.percentile(v, 50)), 3), p95=round(float(np.percentile(v, 95)), 3),
                max=round(float(v.max()), 3), min=round(float(v.min()), 3)) if len(v) else dict(n=0)


def main():
    lg, dropped = from_logs()
    g61, g23 = from_g61(), from_g21()
    # de-duplicate: a G57 task could also exist as a single log (it does not: p1_g57 skipped), g18 logs vs g23 rows
    lg = [r for r in lg if not r['file'].startswith(('p1_g18/', 'p1_g19/', 'p1_g20/'))]   # = g21 rows (dual-arm logs: g21 maps plans correctly)
    rows = g23 + g61 + lg
    by_arm = {a: stats([r['resid'] for r in rows if r['arm'] == a]) for a in ('arm_1', 'arm_2', 'arm_3', 'arm_4')}
    over = [r for r in rows if r['meas_peak'] > r['j2_pred']]
    out = dict(rows=rows, dropped_lowpeak=dropped, by_arm=by_arm, all=stats([r['resid'] for r in rows]),
               by_source={s: stats([r['resid'] for r in rows if r['source'].startswith(s)])
                          for s in ('g18-g20', 'G57', 'log')},
               meas_over_pred=[(r['file'], r['arm'], r['meas_peak'], r['j2_pred']) for r in over],
               meas_peak_max=max(r['meas_peak'] for r in rows))
    json.dump(out, open(os.path.join(HERE, 'g64_torque_pairs.json'), 'w'), indent=1)
    print(f"pasangan {len(rows)} (g18-g20 {len(g23)}, G57 {len(g61)}, log {len(lg)}); dibuang puncak<1 {len(dropped)}")
    for d in dropped:
        print('   dibuang', d['file'], d['meas_peak'])
    print('sisa ukur-RNEA j2 per lengan:', json.dumps(by_arm))
    print('per sumber:', json.dumps(out['by_source']))
    print('semua:', json.dumps(out['all']))
    print(f'ukur > RNEA+7.7: {len(over)}', over and [(r["file"], r["meas_peak"], r["j2_pred"]) for r in over])
    for r in lg:
        print(f"   {r['file']:60s} {r['arm']} rnea {r['j2_rnea']:5.2f} pred {r['j2_pred']:5.2f} ukur {r['meas_peak']:5.2f} sisa {r['resid']:+.2f}")


if __name__ == '__main__':
    main()
