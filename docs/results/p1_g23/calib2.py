"""G23 B0.1 (A2'): margin from the MEASURED final configuration, not the s* proxy.

Executed plans of calib.py's set C; q_final = last /joint_states sample of the
arm's 6 joints at or before the probe window end (torque_offsets.py g21 end
columns). Valid only if FK(q_final) is within 5 mm / 5 deg of the target.

    python3 calib2.py   (needs g23_calib.json from calib.py) -> g23_calib2.json
"""
import csv
import gzip
import json
import os

import numpy as np

import oracle as O

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
SESS = {'g18': ('p1_g18/g18_joint_states_all_trials.csv.gz', 'p1_g18/g18_windows.txt', 3),
        'g19': ('p1_g19/g19_joint_states_all.csv.gz', 'p1_g19/g19_windows.txt', 5),
        'g20': ('p1_g20/g20_joint_states_all.csv.gz', 'p1_g20/g20_windows.txt', 2)}


def main():
    C = [r for r in json.load(open(os.path.join(HERE, 'g23_calib.json')))['rows'] if r['executed']]
    final = {}
    for s, (js, wf, c1) in SESS.items():
        tend = {int(c[0]): float(c[c1]) for c in (ln.split() for ln in open(f'{R}/{wf}'))}
        want = {(k, O.PREFIX[r['arm']] + f'joint_{j}'): tend[k]
                for r in C if r['sess'] == s for k in [r['trial']] for j in range(1, 7)}
        best = {}
        names = {n for _, n in want}
        with gzip.open(f'{R}/{js}', 'rt') as f:
            for row in csv.DictReader(f):
                if row['joint'] not in names:
                    continue
                t = float(row['t'])
                for k, te in tend.items():
                    key = (k, row['joint'])
                    if key in want and t <= te and t > best.get(key, (-1, 0))[0]:
                        best[key] = (t, float(row['pos']))
        for (k, n), v in best.items():
            final[(s, k, n)] = v
    rows, D = [], []
    for r in C:
        names = [O.PREFIX[r['arm']] + f'joint_{j}' for j in range(1, 7)]
        got = [final.get((r['sess'], r['trial'], n)) for n in names]
        out = dict(r)
        if any(g is None for g in got):
            out['why'] = 'tidak ada sampel'
            rows.append(out)
            continue
        q = np.array([g[1] for g in got])
        tau, p, z = O.static_at(q, r['rail'], r['arm'])
        e_mm = float(np.linalg.norm(p - np.array(r['xyz'])) * 1000)
        e_deg = float(np.degrees(np.arccos(np.clip(z @ O.DOWN, -1, 1))))
        out.update(q_final=q.round(5).tolist(), t_sample=max(g[0] for g in got),
                   fk_err_mm=round(e_mm, 2), fk_err_deg=round(e_deg, 2),
                   static_final=tau.round(4).tolist())
        if e_mm < 5.0 and e_deg < 5.0:
            out['delta_final'] = (np.array(r['rnea']) - tau).round(4).tolist()
            D.append(out['delta_final'])
        else:
            out['why'] = 'FK di luar 5 mm / 5 deg'
        rows.append(out)
    D = np.array(D)
    m = np.maximum(0.0, D.max(0))
    bad = [r for r in rows if 'why' in r]
    print(f"C' = {len(C)} dieksekusi; sah {len(D)}; dikeluarkan {len(bad)}: "
          f"{[(r['sess'], r['trial'], r['arm'], r['why'], r.get('fk_err_mm')) for r in bad]}")
    for j in range(6):
        print(f'  joint_{j + 1}: delta min {D[:, j].min():+.3f} median {np.median(D[:, j]):+.3f} '
              f'p95 {np.percentile(D[:, j], 95):+.3f} max {D[:, j].max():+.3f}  -> m_{j + 1} = {m[j]:.4f}')
    json.dump(dict(margin=m.tolist(), n=len(C), n_valid=len(D), rows=rows),
              open(os.path.join(HERE, 'g23_calib2.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
