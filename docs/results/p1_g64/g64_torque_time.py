"""G64 0d (docs/p1_g64_joint_plan.md): TIME-dependent torque on REAL hardware. OFFLINE, HW CSVs only.

Per archived /joint_states CSV (t, joint, pos, eff; HW sessions, mock windows excluded) and per arm joint_2:
  peak |eff|; total time and longest contiguous run with |eff| above 12 / 14 / 16 N.m (sample-hold, dt capped at
  0.5 s so recorder gaps count as nothing); max moving average of |eff| over 1 / 5 / 30 s windows; max RMS with a
  first-order filter of eff^2 at tau 22 s (KA-75+ winding thermal time constant, Kinova AS-ACT-KA58-KA75-SP 2018 --
  NOT verified to be the Gen3 lite actuator) = an I^2 t proxy, since effort is derived from motor current
  (Gen3 lite user guide Table 21).

    python3 g64_torque_time.py            -> g64_torque_time.json + table
"""
import glob
import gzip
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
J2 = ['t1_a1_joint_2', 't1_a2_joint_2', 't2_a1_joint_2', 't2_a2_joint_2']
ARM = dict(zip(J2, ('arm_1', 'arm_2', 'arm_3', 'arm_4')))
TH = (12.0, 14.0, 16.0)
WIN = (1.0, 5.0, 30.0)
TAU = 22.0
DT_CAP = 0.5


def files():
    out = []
    for fn in sorted(glob.glob(os.path.join(R, '**', '*joint_states*.csv*'), recursive=True)):
        rel = os.path.relpath(fn, R)
        if 'mock' in rel or '_window' in rel or rel.startswith('p1_g64'):
            continue
        out.append(fn)
    return out


def load(fn):
    parts = []
    for ch in pd.read_csv(fn, usecols=['t', 'joint', 'eff'], chunksize=2_000_000,
                          dtype={'t': float, 'joint': str, 'eff': float}):
        parts.append(ch[ch['joint'].isin(J2)])
    return pd.concat(parts) if parts else pd.DataFrame(columns=['t', 'joint', 'eff'])


def analyse(t, e):
    o = np.argsort(t, kind='stable')
    t, e = t[o], np.abs(e[o])
    ok = np.isfinite(e)
    t, e = t[ok], e[ok]
    if len(t) < 2:
        return None
    dt = np.minimum(np.diff(t, append=t[-1]), DT_CAP)
    res = dict(n=int(len(t)), span_s=round(float(t[-1] - t[0]), 1), peak=round(float(e.max()), 3),
               t_peak=round(float(t[int(e.argmax())]), 2))
    for th in TH:
        above = e > th
        tot = float(dt[above].sum())
        run, best = 0.0, 0.0
        for a, d in zip(above, dt):
            run = run + d if a else 0.0
            best = max(best, run)
        res[f'above_{th:g}_s'] = round(tot, 3)
        res[f'longest_{th:g}_s'] = round(best, 3)
    # time-weighted moving averages on a 0.02 s grid (sample-hold)
    g = np.arange(t[0], t[-1], 0.02)
    idx = np.searchsorted(t, g, side='right') - 1
    eg = e[idx]
    gap = (g - t[idx]) > DT_CAP
    eg = np.where(gap, 0.0, eg)
    c = np.concatenate([[0.0], np.cumsum(eg) * 0.02])
    for w in WIN:
        k = int(round(w / 0.02))
        res[f'avg_{w:g}s_max'] = round(float(((c[k:] - c[:-k]) / w).max()), 3) if len(c) > k else None
    a = 0.02 / TAU
    y, ymax = 0.0, 0.0
    for v in eg * eg:            # first-order filter of eff^2
        y += a * (v - y)
        ymax = max(ymax, y)
    res['rms_tau22_max'] = round(float(np.sqrt(ymax)), 3)
    return res


def main():
    out = []
    for fn in files():
        rel = os.path.relpath(fn, R)
        try:
            df = load(fn)
        except Exception as ex:                       # noqa: BLE001
            print(f'GAGAL {rel}: {ex!r}', flush=True)
            out.append(dict(file=rel, error=repr(ex)))
            continue
        for j in J2:
            d = df[df['joint'] == j]
            r = analyse(d['t'].to_numpy(float), d['eff'].to_numpy(float))
            if r is None:
                continue
            r.update(file=rel, arm=ARM[j])
            out.append(r)
            print(f"{rel:62s} {ARM[j]} puncak {r['peak']:6.2f} >12 {r['above_12_s']:7.2f}s (terpanjang "
                  f"{r['longest_12_s']:5.2f}) >14 {r['above_14_s']:6.2f}s (maks {r['longest_14_s']:5.2f}) >16 "
                  f"{r['above_16_s']:5.2f}s  avg1 {r['avg_1s_max']} avg5 {r['avg_5s_max']} avg30 {r['avg_30s_max']} "
                  f"rms22 {r['rms_tau22_max']}", flush=True)
    json.dump(out, open(os.path.join(HERE, 'g64_torque_time.json'), 'w'), indent=1)
    ok = [r for r in out if 'peak' in r]
    print('RINGKAS: puncak maks', max(r['peak'] for r in ok), '| >14 total', round(sum(r['above_14_s'] for r in ok), 2),
          's, terpanjang', max(r['longest_14_s'] for r in ok), '| >12 terpanjang', max(r['longest_12_s'] for r in ok),
          '| avg5 maks', max(r['avg_5s_max'] or 0 for r in ok), '| avg30 maks', max(r['avg_30s_max'] or 0 for r in ok),
          '| rms22 maks', max(r['rms_tau22_max'] for r in ok))


if __name__ == '__main__':
    main()
