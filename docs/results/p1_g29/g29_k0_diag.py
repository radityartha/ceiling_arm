"""K0 failure diagnosis: same 72 tuples, current oracle2 (default) vs ORIGINAL oracle2 (pre-G29 copy) vs cache."""
import sys, json, os
import numpy as np
from multiprocessing import Pool
sys.path.insert(0, '/tmp/claude-1001/-home-user1-Documents-ceiling-arm/d1764067-0f09-431b-bc6f-d9144b96c291/scratchpad')
import g29_oracle_rot as R
import oracle2o as O2o
MI4, REF, SLOT_ARM, MARGIN = R.MI4, R.REF, R.SLOT_ARM, R.MARGIN
F = R.FIELDS + ('ok',)

def pick():
    cache = list(MI4.load_cache().values())
    rng = np.random.default_rng(29)
    z = lambda d: round(float(REF.nodes[d['node']][2]), 2)
    ok = [d for d in cache if d['ok']]; no = [d for d in cache if not d['ok']]
    p = [ok[i] for i in rng.choice(len(ok), 36, replace=False)] + [no[i] for i in rng.choice(len(no), 36, replace=False)]
    n = sum(z(d) == 1.4 for d in p)
    if n < 12:
        ex = [d for d in cache if z(d) == 1.4 and d not in p]
        p += [ex[i] for i in rng.choice(len(ex), 12 - n, replace=False)]
    return p

def job(d):
    xyz, L, arm = REF.nodes[d['node']], float(REF.lin[d['p']]), SLOT_ARM[(d['g'], d['s'])]
    out = {}
    for tag, mod in (('cur', R.O2), ('orig', O2o)):
        r = mod.solve(xyz, L, arm, tilt=True, envelope=True)
        out[tag] = dict(n_sol=r['n_sol'], n_roll=r['n_roll'], rounds=r['rounds'], saturated=r['saturated'],
                        taumax=r['taumax'].tolist(), tilt_fail=r['tilt_fail'], ok=bool(mod.torq2(r, MARGIN)))
    return d, out

with Pool(15) as pool:
    res = pool.map(job, pick())
cc = sum(o['cur'] == o['orig'] for _, o in res)
oc = sum(all(o['orig'][f] == d[f] for f in F) for d, o in res)
print(f'cur == orig (semua field): {cc}/{len(res)}; orig == cache: {oc}/{len(res)}')
diff = {}
for d, o in res:
    for f in F:
        if o['orig'][f] != d[f]:
            diff[f] = diff.get(f, 0) + 1
print('field beda orig vs cache:', diff)
okflip = sum(o['orig']['ok'] != d['ok'] for d, o in res)
dt = [np.nanmax(np.abs(np.array(o['orig']['taumax']) - np.array(d['taumax']))) for d, o in res]
print(f'ok berbeda vs cache: {okflip}; maks |dtaumax| {np.nanmax(dt):.3e}; tuple beda: '
      f"{[(d['node'], d['g'], d['p'], d['s']) for d, o in res if o['orig'] != {f: d[f] for f in F}][:6]}")
