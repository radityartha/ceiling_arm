"""G23 B3 diagnosis: re-solve ORACLE'-accepted tuples with 64 seeds instead of 8.
Does the 8-seed branch enumeration miss unsafe branches? Offline, diagnosis only."""
import json
import numpy as np
from multiprocessing import Pool
import oracle as O

def job(a):
    xyz, L, arm = a
    M = O._model()
    if len(M['seeds']) == 8:
        rng = np.random.default_rng(2323)
        lo, hi = M['arm_1']['lo'], M['arm_1']['hi']
        M['seeds'] = M['seeds'] + [rng.uniform(lo, hi) for _ in range(56)]
        O.N_SEEDS = 64
    return O.solve(xyz, L, arm)[0]

if __name__ == '__main__':
    o = dict(np.load('g23_oracle.npz'))
    m = np.array(json.load(open('g23_calib2.json'))['margin'])
    ok = O.torq_all(o['taus'], m)
    idx = np.argwhere(ok)
    rng = np.random.default_rng(0)
    pick = idx[rng.choice(len(idx), min(300, len(idx)), replace=False)]
    jobs = [(tuple(o['xyz'][i]), float(o['lins'][l]), str(o['arms'][a])) for i, l, a in pick]
    with Pool(14) as p:
        res = p.map(job, jobs)
    still = [bool(O.torq_all(t, m)) for t in res]
    print(f'tuple ORACLE\' benar di tensor: {len(idx)} / {ok.size}')
    print(f'sampel {len(jobs)}: tetap lolos dengan 64 benih {sum(still)}, '
          f'jatuh (cabang tak aman ditemukan) {len(jobs) - sum(still)}')
    json.dump(dict(n_true=int(len(idx)), n=len(jobs), still=int(sum(still))), open('g23_branch_probe.json', 'w'))
