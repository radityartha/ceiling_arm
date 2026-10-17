"""G64 option B risk: gantry BRIDGE tracking error vs the commanded JTC trajectory, G57 HW traverses. OFFLINE.

pose_to_g sends ONE cosine from the measured start to the goal over T_cmd (both axes); the bridge chases the JTC setpoint
with stop-start go_to_absolute calls (gantry-bridge-debounce). Commanded q_cmd(t) = s + (g - s) (1 - cos(pi (t - t_send)
/ T_cmd)) / 2 is reconstructed from the run JSON (start, goal, t_send, T_cmd); measured = /joint_states (G57 CSVs).
Per traverse: max |measured - commanded| (mm / deg) during the motion, and the time lag that best aligns the two.
A whole-body plan (arm folding WHILE the rail moves) is screened at the commanded rail position, so this error is the
distance the real arm can be away from where the screen put it.

    python3 g64_bridge_track.py   -> g64_bridge_track.json + table
"""
import glob
import json
import math
import os

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.join(HERE, '..', 'p1_g57', 'hw')
NAMES = ['t1_linear_joint', 't1_rotation_joint', 't2_linear_joint', 't2_rotation_joint']


def load():
    parts = []
    for fn in sorted(glob.glob(os.path.join(HW, 'g57_joint_states_part*.csv.gz'))):
        for ch in pd.read_csv(fn, usecols=['t', 'joint', 'pos'], chunksize=2_000_000):
            parts.append(ch[ch['joint'].isin(NAMES)])
    df = pd.concat(parts)
    return {j: df[df['joint'] == j].sort_values('t')[['t', 'pos']].to_numpy(float) for j in NAMES}


def main():
    S = load()
    out = []
    for fn in sorted(glob.glob(os.path.join(HW, 'p1hw_par_*_run.json'))):
        run = os.path.basename(fn)[len('p1hw_par_'):-len('_run.json')]
        for e in json.load(open(fn))['events']:
            r = e.get('rail') if e['kind'] == 'traverse' else None
            if not r or not r.get('t_send') or not r.get('T_cmd'):
                continue
            g = e['gantry']
            t0, T = r['t_send'], r['T_cmd']
            row = dict(run=run, idx=e['idx'], gantry=g, T_cmd=T, start=r['start'], goal=r['goal'])
            for ax, k, sc, unit in (('lin', 0, 1000.0, 'mm'), ('rot', 1, 1.0, 'deg')):
                s, gl = r['start'][k], r['goal'][k]
                if abs(gl - s) < (1e-4 if ax == 'lin' else 0.05):
                    continue
                a = S[f't{g}_{"linear" if ax == "lin" else "rotation"}_joint']
                m = (a[:, 0] >= t0) & (a[:, 0] <= t0 + T + 5.0)
                t, q = a[m, 0], a[m, 1] * (1.0 if ax == 'lin' else 180.0 / math.pi)
                if len(t) < 10:
                    continue

                def cmd(tt):
                    u = np.clip((tt - t0) / T, 0.0, 1.0)
                    return s + (gl - s) * 0.5 * (1.0 - np.cos(math.pi * u))
                err = (q - cmd(t)) * (sc if ax == 'lin' else 1.0)
                inmove = t <= t0 + T
                lags = np.arange(-3.0, 6.01, 0.05)
                rms = [float(np.sqrt(np.mean(((q - cmd(t - L)) * (sc if ax == 'lin' else 1.0))[inmove] ** 2))) for L in lags]
                row[ax] = dict(dist=round((gl - s) * (sc if ax == 'lin' else 1.0), 2), unit=unit,
                               max_abs_err=round(float(np.max(np.abs(err[inmove]))), 2),
                               p95_abs_err=round(float(np.percentile(np.abs(err[inmove]), 95)), 2),
                               best_lag_s=round(float(lags[int(np.argmin(rms))]), 2),
                               rms_at_best=round(float(min(rms)), 2), end_err=round(float(err[-1]), 2))
            if 'lin' in row or 'rot' in row:
                out.append(row)
                print(run, e['idx'], g, f"T {T:.1f}", {k: row[k] for k in ('lin', 'rot') if k in row})
    json.dump(out, open(os.path.join(HERE, 'g64_bridge_track.json'), 'w'), indent=1)
    for ax in ('lin', 'rot'):
        v = [r[ax] for r in out if ax in r]
        if v:
            print(f"{ax}: n {len(v)} maks |galat| {max(x['max_abs_err'] for x in v)} {v[0]['unit']}, p95 median "
                  f"{np.median([x['p95_abs_err'] for x in v]):.2f}, lag terbaik median {np.median([x['best_lag_s'] for x in v]):.2f} s "
                  f"(min {min(x['best_lag_s'] for x in v)}, maks {max(x['best_lag_s'] for x in v)})")


if __name__ == '__main__':
    main()
