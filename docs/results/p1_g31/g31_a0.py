"""G31 A0 (instrument, before section A is locked): size of the rotated-oracle job + cost per solve.

17 (iv) seeds of g26_candidates.json (G26 1/2/13 + G28 14). For each seed's 6 nodes: L1-true tuples
(node, g, lin_idx, rot_idx, slot) at |rot| <= 35 deg, rot != 0 (5 deg grid of REF.rot); the R10 subset.
Then oracle''' (g29_oracle_rot._job) timed on 8 tuples. OFFLINE.
"""
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
import g29_oracle_rot as GO  # noqa: E402

REF, sched, MI4 = GO.REF, GO.sched, GO.MI4
SEEDS = [1, 2, 13, 16, 17, 18, 20, 21, 22, 23, 26, 29, 33, 35, 36, 40, 42]
cand = {r['seed']: r for r in json.load(open(os.path.join(HERE, '../p1_g26/g26_candidates.json')))}
rdeg = np.degrees(REF.rot)
R35 = [i for i, r in enumerate(rdeg) if abs(r) <= 35 + 1e-9 and abs(r) > 1e-9]
R10 = [i for i in R35 if abs(rdeg[i]) <= 10 + 1e-9]
print('R35 deg', [round(rdeg[i]) for i in R35])
nodes = sorted({n for s in SEEDS for n in cand[s]['nodes']})
tup = []
for n in nodes:
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(MI4.CAPS[g], np.array([n]))
        r = r[0].reshape(len(REF.lin), len(REF.rot), 2)
        for ri in R35:
            for p, s in np.argwhere(r[:, ri, :]):
                tup.append((n, g, int(p), ri, int(s)))
n10 = sum(t[3] in R10 for t in tup)
print(f'node unik {len(nodes)} (17 seed x 6); tuple L1-benar rot!=0 |rot|<=35: {len(tup)}, |rot|<=10: {n10}')
rng = np.random.default_rng(31)
pick = [tup[i] for i in rng.choice(len(tup), 8, replace=False)]
t0 = time.time()
res = [GO._job(t) for t in pick]
dt = (time.time() - t0) / len(pick)
print(f'oracle\'\'\' + PATH per tuple (1 proses): {dt:.2f} s; ok3 {sum(r["ok3"] for r in res)}/8')
print(f'perkiraan wall 15 proses: R35 {len(tup) * dt / 15 / 60:.1f} menit')
