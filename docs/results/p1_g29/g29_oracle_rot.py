"""G29 A4 (d): oracle''' at gantry rotation != 0 (oracle2 `rot`, default unchanged). OFFLINE.

    python3 g29_oracle_rot.py k       -> K0 (72 tuples bit-identical to g24_oracle3_cache), K-FK, K-EQ
    python3 g29_oracle_rot.py sample  -> value of rotation: 150 rot-0-infeasible + 50 feasible nodes x
                                         R = {+-30, +-60, +-90} x (g, slot) x 2 nearest rails; cache jsonl
    python3 g29_oracle_rot.py report  -> g29_oracle_rot.json
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for p in ('../p1_g22', '../p1_g23', '../p1_g24', '../../../ros2_ws/src/reachability_gng'):
    sys.path.insert(0, os.path.join(HERE, p))
import make_instance_g24 as MI4  # noqa: E402
import oracle2 as O2  # noqa: E402
from make_instance import sched  # noqa: E402
from reachability_gng import irm_sweep as IS  # noqa: E402

O = O2.O
REF, SLOT_ARM, MARGIN = MI4.REF, MI4.SLOT_ARM, MI4.MARGIN
IRM = {'arm_1': 'arm1', 'arm_2': 'arm2', 'arm_3': 'arm3', 'arm_4': 'arm4'}
Y_G = {1: 0.36, 2: -0.36}
MP = np.array(json.load(open(os.path.join(HERE, '../p1_g27/g27_t3.json')))['t3a']['m_p'])
LIM, OFF = np.array(O.LIM, float), np.array(O.OFFSET, float)
R_DEG = (30, -30, 60, -60, 90, -90)
RIDX = {r: int(np.argmin(np.abs(np.degrees(REF.rot) - r))) for r in (0,) + R_DEG}
CACHE = os.path.join(HERE, 'g29_oracle_rot_cache.jsonl')
FIELDS = ('n_sol', 'n_roll', 'rounds', 'saturated', 'taumax', 'tilt_fail')


# ------------------------------------------------------------------------------------------- K0
def _k0(args):
    d, explicit = args
    xyz, L, arm = REF.nodes[d['node']], float(REF.lin[d['p']]), SLOT_ARM[(d['g'], d['s'])]
    r = O2.solve(xyz, L, arm, tilt=True, envelope=True, **({'rot': 0.0} if explicit else {}))
    got = dict(n_sol=r['n_sol'], n_roll=r['n_roll'], rounds=r['rounds'], saturated=r['saturated'],
               taumax=r['taumax'].tolist(), tilt_fail=r['tilt_fail'], ok=bool(O2.torq2(r, MARGIN)))
    same = lambda f: (np.array_equal(np.array(got[f], float), np.array(d[f], float), equal_nan=True)
                      if f == 'taumax' else got[f] == d[f])     # NaN != NaN for n_sol = 0 (B, K0 run 1)
    return all(same(f) for f in FIELDS + ('ok',))


def k0(pool):
    cache = list(MI4.load_cache().values())
    rng = np.random.default_rng(29)
    z = lambda d: round(float(REF.nodes[d['node']][2]), 2)
    ok = [d for d in cache if d['ok']]
    no = [d for d in cache if not d['ok']]
    pick = [ok[i] for i in rng.choice(len(ok), 36, replace=False)] + \
           [no[i] for i in rng.choice(len(no), 36, replace=False)]
    n140 = sum(z(d) == 1.4 for d in pick)
    if n140 < 12:
        extra = [d for d in cache if z(d) == 1.4 and d not in pick]
        pick += [extra[i] for i in rng.choice(len(extra), 12 - n140, replace=False)]
    res_def = pool.map(_k0, [(d, False) for d in pick])
    res_exp = pool.map(_k0, [(d, True) for d in pick])
    out = dict(n=len(pick), z140=sum(z(d) == 1.4 for d in pick), default_identical=sum(res_def),
               explicit_identical=sum(res_exp))
    print(f"K0: {out['n']} tuple ({out['z140']} di z = 1.40): default == cache {out['default_identical']}/{out['n']}, "
          f"rot=0.0 eksplisit == cache {out['explicit_identical']}/{out['n']}", flush=True)
    return out


def kfk():
    import pinocchio as pin
    worst = 0.0
    rng = np.random.default_rng(29)
    for arm in SLOT_ARM.values():
        for r in R_DEG:
            A = O2.model(arm, np.radians(r))
            m, d = A['m'], A['d']
            for L in rng.uniform(0, 1.6, 5):
                q = np.zeros(m.nq)
                q[A['rail']] = L
                q[A['iq']] = O.REST
                pin.framesForwardKinematics(m, d, q)
                p = d.oMf[m.getFrameId(f'{O.PREFIX[arm]}base_link')].translation
                _, pb = IS.base_pose(IRM[arm], L, np.radians(r))
                worst = max(worst, float(np.abs(p - pb).max()))
    print(f'K-FK: base_link model tereduksi (rot) vs irm_sweep.base_pose: maks {worst:.2e} m', flush=True)
    return dict(max_dev_m=worst)


def rotate_target(xyz, L, g, r):
    c = np.array([L, Y_G[g]])
    ca, sa = np.cos(-r), np.sin(-r)
    v = np.asarray(xyz[:2], float) - c
    return np.array([c[0] + ca * v[0] - sa * v[1], c[1] + sa * v[0] + ca * v[1], xyz[2]])


def _keq(args):
    d, rdeg = args
    g, arm = d['g'], SLOT_ARM[(d['g'], d['s'])]
    xyz, L, r = REF.nodes[d['node']], float(REF.lin[d['p']]), np.radians(rdeg)
    a = O2.solve(xyz, L, arm, tilt=True, envelope=True, rot=r)
    b = O2.solve(rotate_target(xyz, L, g, r), L, arm, tilt=True, envelope=True)
    return dict(rdeg=rdeg, ok_a=bool(O2.torq2(a, MARGIN)), ok_b=bool(O2.torq2(b, MARGIN)),
                n_roll=(a['n_roll'], b['n_roll']), n_sol=(a['n_sol'], b['n_sol']),
                dtau=float(np.nanmax(np.abs(a['taumax'] - b['taumax']))) if a['n_sol'] and b['n_sol'] else
                (0.0 if not a['n_sol'] and not b['n_sol'] else float('inf')))


def keq(pool):
    cache = list(MI4.load_cache().values())
    rng = np.random.default_rng(2929)
    ok = [d for d in cache if d['ok']]
    no = [d for d in cache if not d['ok']]
    pick = [ok[i] for i in rng.choice(len(ok), 12, replace=False)] + [no[i] for i in rng.choice(len(no), 12, replace=False)]
    jobs = [(d, 90 if k % 2 == 0 else -60) for k, d in enumerate(pick)]
    R = pool.map(_keq, jobs)
    same_ok = sum(r['ok_a'] == r['ok_b'] for r in R)
    same_roll = sum(r['n_roll'][0] == r['n_roll'][1] for r in R)
    med = float(np.median([r['dtau'] for r in R]))
    out = dict(n=len(R), same_ok=same_ok, same_n_roll=same_roll, median_dtau=med,
               max_dtau=float(max(r['dtau'] for r in R)), rows=R,
               ok=same_ok == len(R) and same_roll >= 22 and med <= 1e-3)
    print(f"K-EQ: {len(R)} tuple (rot +90 / -60 vs target diputar di rot 0): ok sama {same_ok}/{len(R)}, n_roll sama "
          f"{same_roll}, median |dtau| {med:.2e}, maks {out['max_dtau']:.2e} -> {'LULUS' if out['ok'] else 'GAGAL'}",
          flush=True)
    return out


# --------------------------------------------------------------------------------------- sample
def node_status():
    cache = MI4.load_cache()
    st = {}
    for (n, g, p, s), d in cache.items():
        st[n] = st.get(n, False) | d['ok']
    return st


def pick_nodes():
    st = node_status()
    N = np.array(sorted(st))
    ok = np.array([st[n] for n in N])
    rng = np.random.default_rng(29)
    ay = np.round(np.abs(REF.nodes[N][:, 1]), 3)
    bad_idx = np.flatnonzero(~ok)
    layers = sorted(set(ay[bad_idx]))
    tot = len(bad_idx)
    chosen = []
    for y in layers:
        idx = bad_idx[ay[bad_idx] == y]
        k = int(round(150 * len(idx) / tot))
        chosen += list(rng.choice(idx, min(k, len(idx)), replace=False))
    while len(chosen) > 150:
        chosen.pop()
    rest = [i for i in bad_idx if i not in chosen]
    while len(chosen) < 150:
        chosen.append(rest.pop(int(rng.integers(len(rest)))))
    good = list(rng.choice(np.flatnonzero(ok), 50, replace=False))
    return [int(N[i]) for i in chosen], [int(N[i]) for i in good]


def tuples_for(node):
    out = []
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(MI4.CAPS[g], np.array([node]))
        r = r[0].reshape(len(REF.lin), len(REF.rot), 2)
        for rdeg in R_DEG:
            ri = RIDX[rdeg]
            for s in (0, 1):
                ps = np.flatnonzero(r[:, ri, s])
                if not len(ps):
                    continue
                arm = SLOT_ARM[(g, s)]
                rho = [np.linalg.norm(IS.base_pose(IRM[arm], float(REF.lin[p]), float(REF.rot[ri]))[1][:2]
                                      - REF.nodes[node][:2]) for p in ps]
                for p in ps[np.argsort(rho)[:2]]:
                    out.append((node, g, int(p), ri, s))
    return out


def path_max(arm, q_end, rail, rot, n=101):
    start = np.array(O.REST)
    return np.max([O2.gravity(arm, start + (q_end - start) * t, rail, rot) for t in np.linspace(0, 1, n)], axis=0)


def _job(t):
    node, g, p, ri, s = t
    arm, rot = SLOT_ARM[(g, s)], float(REF.rot[ri])
    r = O2.solve(REF.nodes[node], float(REF.lin[p]), arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    pv, P = None, None
    if ok3:
        P = np.max([path_max(arm, np.array(q), float(REF.lin[p]), rot) for _, q in r['solutions']], axis=0)
        pv = bool(np.all(P + OFF + MP <= LIM))
        P = P.tolist()
    return dict(key=[node, g, p, ri, s], rot_deg=float(np.degrees(rot)), n_sol=r['n_sol'], n_roll=r['n_roll'],
                saturated=r['saturated'], taumax=r['taumax'].tolist(), tilt_fail=r['tilt_fail'], ok3=ok3,
                path=pv, P=P, ok4=bool(ok3 and pv))


def load():
    c = {}
    if os.path.exists(CACHE):
        for ln in open(CACHE):
            d = json.loads(ln)
            c[tuple(d['key'])] = d
    return c


def sample(pool):
    bad, good = pick_nodes()
    have = load()
    todo = [t for n in bad + good for t in tuples_for(n) if t not in have]
    print(f'node: {len(bad)} tidak-layak rot 0, {len(good)} layak; tuple baru {len(todo)} (sudah {len(have)})', flush=True)
    with open(CACHE, 'a') as f:
        for i, d in enumerate(pool.imap_unordered(_job, todo, chunksize=1), 1):
            f.write(json.dumps(d) + '\n')
            f.flush()
            if i % 500 == 0:
                print(f'  {i}/{len(todo)}', flush=True)
    json.dump(dict(bad=bad, good=good), open(os.path.join(HERE, 'g29_oracle_rot_nodes.json'), 'w'))


def report():
    nodes = json.load(open(os.path.join(HERE, 'g29_oracle_rot_nodes.json')))
    c = load()
    by = {}
    for (n, g, p, ri, s), d in c.items():
        e = by.setdefault(n, {})
        rd = int(round(d['rot_deg']))
        o3, o4 = e.get(rd, (False, False))
        e[rd] = (o3 or d['ok3'], o4 or d['ok4'])
    res = dict(K={})
    for grp in ('bad', 'good'):
        N = nodes[grp]
        row = {}
        for rd in R_DEG:
            row[rd] = (sum(by.get(n, {}).get(rd, (False, False))[0] for n in N),
                       sum(by.get(n, {}).get(rd, (False, False))[1] for n in N))
        any3 = sum(any(v[0] for v in by.get(n, {}).values()) for n in N)
        any4 = sum(any(v[1] for v in by.get(n, {}).values()) for n in N)
        res[grp] = dict(n=len(N), per_rot=row, any_ok3=any3, any_ok4=any4)
        print(f"{grp}: {len(N)} node; layak-oracle''' pada suatu r: {any3}; oracle'''': {any4}")
        print('    per r (o3/o4): ' + ', '.join(f'{rd:+d}: {a}/{b}' for rd, (a, b) in row.items()))
    # strata for bad nodes
    strata = {}
    for n in nodes['bad']:
        x = REF.nodes[n]
        k = (round(abs(float(x[1])), 3), round(float(x[2]), 2))
        v = strata.setdefault('y%.3f' % k[0], [0, 0, 0])
        v[0] += 1
        v[1] += any(t[0] for t in by.get(n, {}).values())
        v[2] += any(t[1] for t in by.get(n, {}).values())
        w = strata.setdefault('z%.2f' % k[1], [0, 0, 0])
        w[0] += 1
        w[1] += any(t[0] for t in by.get(n, {}).values())
        w[2] += any(t[1] for t in by.get(n, {}).values())
    res['bad_strata'] = strata
    print('    tidak-layak rot 0, per lapis (n, o3, o4): ' + ', '.join(f'{k}:{v}' for k, v in sorted(strata.items())))
    z140 = [d for d in c.values() if abs(REF.nodes[d['key'][0]][2] - 1.4) < 1e-9 and d['ok3']]
    res['z140_ok3'] = len(z140)
    res['z140_ok4'] = sum(d['ok4'] for d in z140)
    print(f"z = 1.40: tuple ok''' {len(z140)}, lulus PATH (oracle'''') {res['z140_ok4']}")
    res['n_tuples'] = len(c)
    res['n_ok3'] = sum(d['ok3'] for d in c.values())
    res['n_ok4'] = sum(d['ok4'] for d in c.values())
    json.dump(res, open(os.path.join(HERE, 'g29_oracle_rot.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'report':
        report()
    else:
        with Pool(15) as pool:
            if cmd == 'k':
                out = dict(K0=k0(pool), KFK=kfk(), KEQ=keq(pool))
                json.dump(out, open(os.path.join(HERE, 'g29_oracle_k.json'), 'w'), indent=1, default=float)
            elif cmd == 'sample':
                sample(pool)
