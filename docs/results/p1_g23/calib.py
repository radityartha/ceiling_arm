"""G23 A2: margin m_j from the calibration set C (g18-g20 plans), NOT from G22.

C = every 'torsi RNEA+offset' line in g18/g19/g20 trial logs (PLANNED or
refused). arm = prefix of the 'terketat' joint (the probe's RNEA covers only
the planning arm's joints); target = the log's 'sumber target' line,
cross-checked against <trial>.json last record 'targets' where present; rail per A2.

    python3 calib.py   -> g23_calib.json + table
"""
import json
import os
import re

import numpy as np

import oracle as O

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
RAIL = {'g18': lambda k: {1: 0.550},
        'g20': lambda k: {1: 0.550, 2: 0.000}}
g19_rail = {int(c[0]): float(c[1]) for c in
            (ln.split() for ln in open(f'{R}/p1_g19/g19_windows.txt'))}
RAIL['g19'] = lambda k: {1: g19_rail[k]}
TRIALS = {'g18': range(1, 11), 'g19': range(0, 11), 'g20': range(1, 11)}
PFX2ARM = {v: k for k, v in O.PREFIX.items()}
LINE = re.compile(r'torsi RNEA\+offset\(per-aktuator\) vs rating: (.*?)  \(terketat (t\d_a\d_)joint')


def plans():
    out = []
    for s, ks in TRIALS.items():
        for k in ks:
            log = f'{R}/p1_{s}/{s}_trial{k}.log'
            txt = open(log).read()
            src = re.search(r'sumber target: (.*)', txt)[1]
            tg = dict(zip(re.findall(r'arm_\d', src),
                          ([float(v) for v in t] for t in
                           re.findall(r'\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)', src))))
            js = json.load(open(f'{R}/p1_{s}/{s}_trial{k}.json'))[-1].get('targets')
            assert js is None or js == tg, (log, js, tg)  # json cross-check (g18 t6 has none)
            ms = list(LINE.finditer(txt))
            for i, m in enumerate(ms):
                seg = txt[m.end():ms[i + 1].start() if i + 1 < len(ms) else len(txt)]
                rnea = [float(re.search(rf'{j}=([0-9.]+)->', m[1])[1]) for j in range(1, 7)]
                arm = PFX2ARM[m[2]]
                out.append(dict(sess=s, trial=k, arm=arm, xyz=tg[arm],
                                rail=RAIL[s](k)[O.GANTRY[arm]], rnea=rnea,
                                executed=f'{arm}: moveit MOVED' in seg))
    return out


def main():
    C = plans()
    rows, deltas = [], []
    for c in C:
        taus, _ = O.solve(c['xyz'], c['rail'], c['arm'])
        s = O.best_solution(taus)
        r = dict(c, n_conv=int(np.sum(~np.isnan(taus[:, 0]))))
        if s is not None:
            r['static'] = taus[s].round(4).tolist()
            r['delta'] = (np.array(c['rnea']) - taus[s]).round(4).tolist()
            deltas.append(r['delta'])
        rows.append(r)
    D = np.array(deltas)
    m = np.maximum(0.0, D.max(0))
    nfail = sum(r['n_conv'] == 0 for r in rows)
    ex = [r for r in rows if r['executed']]
    nok = sum(r['n_conv'] > 0 for r in ex)
    print(f'PC1: dieksekusi {len(ex)}, IK_V konvergen {nok}')
    per = ', '.join(f'{s} {sum(c["sess"] == s for c in C)}' for s in TRIALS)
    print(f'C = {len(C)} rencana ({per}); '
          f'IK_V tidak konvergen: {nfail} ({100 * nfail / len(C):.1f} %)')
    for j in range(6):
        print(f'  joint_{j + 1}: delta min {D[:, j].min():+.3f} median {np.median(D[:, j]):+.3f} '
              f'p95 {np.percentile(D[:, j], 95):+.3f} max {D[:, j].max():+.3f}  -> m_{j + 1} = {m[j]:.4f}')
    json.dump(dict(margin=m.tolist(), n=len(C), n_ik_fail=nfail, rows=rows),
              open(os.path.join(HERE, 'g23_calib.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
