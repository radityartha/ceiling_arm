"""G27 A3-H3: ORACLE'''' = ORACLE''' AND PATH (docs/p1_g27_z140_diag.md, A locked 16:23).

PATH(tuple) <=> for EVERY untilted oracle''' grid solution s:
                for every joint j: P_j(REST -> s) + offset_j + m_j^p <= lim_j
P_j = max static |gravity torque| of joint j along the straight joint-space line REST -> s (101 pts);
m^p = max(0, max over C' of RNEA_j - P_j(REST -> q_final)) -- g27_t3.json (T3a).
Only oracle'''-true tuples can be oracle''''-true (K1 by construction). OFFLINE, zero hardware.

    python3 g27_oracle4.py cache    -> g27_oracle4_cache.jsonl (every oracle'''-ok tuple; K0 on all)
    python3 g27_oracle4.py report   -> K1/K2/K3, S12, V28 predictions, A5 (a)(b); g27_oracle4.json
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, HERE)
import g27_diag as D  # noqa: E402

O2, O, MI4, REF = D.O2, D.O, D.MI4, D.REF
T3 = json.load(open(os.path.join(HERE, 'g27_t3.json')))
MP = np.array(T3['t3a']['m_p'])
LIM, OFF = np.array(O.LIM, float), np.array(O.OFFSET, float)
CACHE4 = os.path.join(HERE, 'g27_oracle4_cache.jsonl')
ARMS = {'arm1': 'arm_1', 'arm2': 'arm_2', 'arm3': 'arm_3', 'arm4': 'arm_4'}


def path_verdict(arm, rail, sols):
    if not sols:
        return False, [float('nan')] * 6
    P = np.max([D.path_max(arm, np.array(s), rail) for _, s in sols], axis=0)
    return bool(np.all(P + OFF + MP <= LIM)), P.tolist()


def _job(key):
    arm = D.arm_of(key)
    xyz, rail = D.xyz_rail(key)
    r = O2.solve(xyz, rail, arm, tilt=True, envelope=True, keep_solutions=True)
    ok3 = bool(O2.torq2(r, D.MARGIN))
    pv, P = path_verdict(arm, rail, r['solutions'])
    return dict(key=list(key), n_sol=r['n_sol'], n_roll=r['n_roll'], rounds=r['rounds'],
                saturated=r['saturated'], taumax=r['taumax'].tolist(), tilt_fail=r['tilt_fail'],
                ok3=ok3, path=pv, P=P, ok4=bool(ok3 and pv))


def load4():
    out = {}
    if os.path.exists(CACHE4):
        for ln in open(CACHE4):
            d = json.loads(ln)
            out[tuple(d['key'])] = d
    return out


def build():
    cache = MI4.load_cache()
    have = load4()
    todo = sorted(k for k, d in cache.items() if d['ok'] and k not in have)
    print(f"oracle''' benar di cache: {sum(d['ok'] for d in cache.values())}; dihitung sekarang {len(todo)}", flush=True)
    with Pool(D.NPROC) as pool, open(CACHE4, 'a') as f:
        for i, d in enumerate(pool.imap_unordered(_job, todo, chunksize=2), 1):
            f.write(json.dumps(d) + '\n')
            f.flush()
            if i % 200 == 0:
                print(f'  {i}/{len(todo)}', flush=True)
    c4 = load4()
    bad = [k for k, d in c4.items()
           if not all(D._same(d[f], cache[k][f]) for f in ('n_sol', 'n_roll', 'rounds', 'saturated',
                                                            'taumax', 'tilt_fail')) or d['ok3'] != cache[k]['ok']]
    print(f"K0 (penuh): oracle''' dihitung ulang == cache pada {len(c4) - len(bad)}/{len(c4)} tuple-ok; beda {bad[:10]}")


def _c_job(c):
    arm, rail = c['arm'], c['rail']
    r = O2.solve(c['xyz'], rail, arm, tilt=True, envelope=True, keep_solutions=True)
    ok3 = bool(O2.torq2(r, D.MARGIN))
    pv, P = path_verdict(arm, rail, r['solutions'])
    return dict(sess=c['sess'], trial=c['trial'], arm=arm, z=c['xyz'][2], ok3=ok3, path=pv, P2=P[1],
                rnea2=c['rnea'][1])


def key_of(node, g, rail, armtag):
    s = {'arm1': 0, 'arm2': 1, 'arm3': 0, 'arm4': 1}[armtag]
    return (int(node), int(g), int(np.argmin(np.abs(REF.lin - rail))), s)


def report():
    cache = MI4.load_cache()
    c4 = load4()

    def ok4(k):
        return bool(k in c4 and c4[k]['ok4'])

    out = {}
    # K1
    k1 = [k for k in c4 if c4[k]['ok4'] and not cache[k]['ok']]
    out['K1_violations'] = len(k1)
    # layer counts
    lay = {}
    for k, d in cache.items():
        z = round(float(REF.nodes[k[0]][2]), 2)
        e = lay.setdefault(z, [0, 0, 0])
        e[0] += 1
        e[1] += d['ok']
        e[2] += ok4(k)
    out['layers'] = {str(z): dict(tuples=v[0], ok3=v[1], ok4=v[2]) for z, v in sorted(lay.items())}
    print("per lapis z: tuple cache / oracle''' benar / oracle'''' benar")
    for z, v in sorted(lay.items()):
        print(f'  z {z:.2f}: {v[0]:5d} / {v[1]:4d} / {v[2]:4d}')
    nodes3 = {k[0] for k, d in cache.items() if d['ok']}
    nodes4 = {k[0] for k in c4 if c4[k]['ok4']}
    print(f"node layak: oracle''' {len(nodes3)}, oracle'''' {len(nodes4)} dari {len({k[0] for k in cache})} node kandidat")
    out['nodes'] = dict(ok3=len(nodes3), ok4=len(nodes4), cand=len({k[0] for k in cache}))

    # K2
    acc = {(r['node'], r['g'], r['p'], r['s']) for r in D.TABLE if r['o_ok'] and r['z'] < 1.4}
    keep = sum(ok4(k) for k in acc)
    out['K2'] = dict(tuples=len(acc), kept=keep)
    print(f"K2: tuple distinct diterima-z<1.40 (A0) {len(acc)}, oracle'''' menerima {keep} ({keep / len(acc):.1%})")
    # S12 in-sample
    s12 = D.s12()
    out['S12_ok4'] = {json.dumps(list(k)): ok4(k) for k in s12}
    print(f"S12 (dalam-sampel): oracle'''' menerima {sum(ok4(k) for k in s12)}/12")
    # K3 C'
    C = json.load(open(os.path.join(R, 'p1_g24/g24_validate3.json')))['C']
    with Pool(D.NPROC) as pool:
        k3 = pool.map(_c_job, C, chunksize=2)
    out['K3'] = k3
    n3 = sum(x['ok3'] for x in k3)
    n4 = sum(x['ok3'] and x['path'] for x in k3)
    print(f"K3 C' (81 dieksekusi aman): oracle''' terima {n3}, oracle'''' terima {n4}; "
          f"PATH gagal per z: {sorted(round(x['z'], 2) for x in k3 if x['ok3'] and not x['path'])}")

    # V28 (A4): unplanned oracle'''-ok tuples, z = 1.40 all + z = 1.32 twenty (default_rng(28))
    planned = {(r['node'], r['g'], r['p'], r['s']) for r in D.TABLE}
    v40 = sorted(k for k, d in cache.items() if d['ok'] and abs(REF.nodes[k[0]][2] - 1.4) < 1e-6 and k not in planned)
    p32 = sorted(k for k, d in cache.items() if d['ok'] and abs(REF.nodes[k[0]][2] - 1.32) < 1e-6 and k not in planned)
    rng = np.random.default_rng(28)
    v32 = [p32[i] for i in sorted(rng.choice(len(p32), min(20, len(p32)), replace=False))]
    v28 = [dict(key=list(k), arm=D.arm_of(k), xyz=[float(x) for x in REF.nodes[k[0]]], rail=float(REF.lin[k[2]]),
                ok3=True, ok4=ok4(k), P2=c4[k]['P'][1] if k in c4 else None,
                predict='3/3 PLANNED' if ok4(k) else '>=1/3 TORQUE-UNSAFE') for k in v40 + v32]
    out['V28'] = v28
    print(f"V28: z=1.40 {len(v40)} tuple (oracle'''' terima {sum(ok4(k) for k in v40)}), "
          f"z=1.32 {len(v32)} (terima {sum(ok4(k) for k in v32)})")

    # A5 (a): G26 P1' schedules as-is
    sys.path.insert(0, os.path.join(R, 'p1_g26'))
    cand = json.load(open(os.path.join(R, 'p1_g26/g26_candidates.json')))
    a = []
    for row in cand:
        if not row.get('ok_ii'):
            continue
        ks = [key_of(row['nodes'][int(t)], int(g), st['rail_m'], arm)
              for g, stops in row['schedule_FISIK'].items() for st in stops for t, arm in st['tasks'].items()]
        z140 = any(abs(REF.nodes[k[0]][2] - 1.4) < 1e-6 for k in ks)
        a.append(dict(seed=row['seed'], all_ok4=all(ok4(k) for k in ks), has_z140=z140))
    out['A5a'] = a
    na = sum(x['all_ok4'] for x in a)
    agree = sum(x['all_ok4'] == (not x['has_z140']) for x in a)
    print(f"A5(a): jadwal P1' G26 yang setiap tuple-nya diterima oracle'''': {na}/{len(a)} "
          f"(tanpa z=1.40: {sum(not x['has_z140'] for x in a)}; sepakat dengan 'tanpa z=1.40' {agree}/{len(a)})")

    # A5 (b): tasks fixed, reach := oracle'''', P1' re-solved, p0 0/0
    import g26_make as GM
    c4mask = {k: dict(d, ok=ok4(k)) for k, d in cache.items()}
    b = []
    for r24 in GM.CAND24:
        inst = GM.instance(r24, c4mask, {1: 0.0, 2: 0.0})
        x = GM.RS.p1prime(inst)
        try:
            new = GM.sched.solve_exact(x)
        except ValueError:
            b.append(dict(seed=r24['seed'], ok_i=False))
            continue
        dec = GM.MI.decompose(inst, new, GM.C_TF)
        mv = {g: dec[g]['moves'] for g in (1, 2)}
        conf = GM.sched_coll.schedule_conflict(inst, new.stops)
        ok_ii = all(mv[g] >= 1 for g in (1, 2)) and conf is None
        b.append(dict(seed=r24['seed'], ok_i=True, ok_ii=bool(ok_ii), moves=mv, makespan=new.makespan,
                      serial=sum(new.finish.values())))
    out['A5b'] = b
    print(f"A5(b): tugas tetap, reach := oracle'''': (i) {sum(x['ok_i'] for x in b)}/50, "
          f"(i)-(iii) {sum(bool(x.get('ok_ii')) for x in b)}/50; seed (i)-(iii): "
          f"{[x['seed'] for x in b if x.get('ok_ii')]}")
    json.dump(out, open(os.path.join(HERE, 'g27_oracle4.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    dict(cache=build, report=report)[sys.argv[1]]()
