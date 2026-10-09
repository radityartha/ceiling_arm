"""G38-OFF (docs/p1_g38_hard_instances.md A1-A4): hard instances -- 3 CENTRE + 3 WALL tasks, rotation R35,
p0 g1 PARKED (1.50, 0). OFFLINE, no ROS, no hardware.

Tuple = (node, g, rail li, rot ri, slot s). FEASIBLE = oracle''' torque ok AND env-EXISTS: some oracle IK solution
(any roll) whose end point, other arms REST, other gantry 0/0, is >= 50 mm from the frozen map (task-arm geometry).
Torque results are reused from the G24 (rot 0) and G29/G31 (rot != 0) caches; env needs the solutions, so any tuple
whose torque is ok or unknown is (re)solved here and cached in g38_tuple_cache.jsonl (append-only, restartable).
Scheduling = g31_sched (P1', rot_cmd, lazy-exact) with build() copied ONCE here: lazy also at rot 0, feasibility =
torque AND env, p0 injected. g31_sched.py itself is not changed.

    python3 g38_make.py k          -> K-checks (torque bit-identical to cache, FK tool == target)
    python3 g38_make.py run        -> g38_candidates.json (+ cache), log on stdout
"""
import json
import math
import os
import sys
import time
from dataclasses import replace
from multiprocessing import Pool

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
ROOT = os.path.join(R, '..', '..')
sys.path.insert(0, os.path.join(R, 'p1_g31'))
import g31_sched as S  # noqa: E402

sys.path.insert(0, os.path.join(ROOT, 'scripts'))
M, MI, sched, RS = S.M, S.MI, S.sched, S.RS
O2 = M.O2
REF, NR, RDEG, ROT0 = S.REF, S.NR, S.RDEG, M.ROT0
SLOT_ARM, MARGIN, PREFIX = M.SLOT_ARM, M.MARGIN, M.O2.O.PREFIX
CACHE = os.path.join(HERE, 'g38_tuple_cache.jsonl')
SEEDS = range(20)
N_CAND = 40                       # A2: tuples examined per candidate
ENV_MIN = 0.05                    # A2: = ENV_MARGIN_M
P0_LIN = {1: 1.50, 2: 0.00}       # A3 (G36 B6.6: g1 parked)
N_CTRL = 5                        # A3: R10 control on the first 5 drawn seeds
X = REF.nodes
Z_MAX = 1.32 + 1e-6
D_ENV = cKDTree(np.load(os.path.join(R, 'p1_g34/env_static_map.npz'))['centers']).query(X)[0]
POOL = {'TENGAH': np.flatnonzero((np.abs(X[:, 1]) < 0.15) & (X[:, 2] <= Z_MAX)),
        'DINDING': np.flatnonzero((D_ENV >= 0.10) & (D_ENV < 0.20) & (np.abs(X[:, 1]) >= 0.15) & (X[:, 2] <= Z_MAX))}

_ENV = []


def env_checker():
    if not _ENV:
        from env_collision import EnvChecker
        _ENV.append(EnvChecker())
    return _ENV[0]


def base_joints(g, lin, rot):
    import env_collision as E
    j = {n: 0.0 for n in E.CONFIG_JOINTS}
    for p in PREFIX.values():
        for i, v in enumerate(M.O2.O.REST, 1):
            j[f'{p}joint_{i}'] = v
    j[f't{g}_linear_joint'], j[f't{g}_rotation_joint'] = lin, rot
    return j


def _job(key):
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    env, best, n_chk = False, -math.inf, 0
    if ok3:
        E, p = env_checker(), PREFIX[arm]
        base = base_joints(g, lin, rot)
        for _, q6 in r['solutions']:
            j = dict(base)
            j.update({f'{p}joint_{i}': float(v) for i, v in enumerate(q6, 1)})
            d, _ = E.check(E.q_from(j), only=(p,))
            n_chk += 1
            best = max(best, d)
            if d >= ENV_MIN:
                env = True
                break
    return dict(key=[int(v) for v in key], rot_deg=float(np.degrees(rot)), n_sol=r['n_sol'],
                saturated=bool(r['saturated']), taumax=r['taumax'].tolist(), ok3=ok3, env=env,
                env_best_m=None if best == -math.inf else round(best, 5), env_checked=n_chk)


class Feas:
    """Known feasibility per tuple: True / False / None (unknown -> must be computed)."""

    def __init__(self):
        self.t0, self.trot = M.load_cache(), S.load_rot_cache()       # read-only (G24, G29 + G31)
        self.mine = {}
        if os.path.exists(CACHE):
            for ln in open(CACHE):
                d = json.loads(ln)
                self.mine[tuple(d['key'])] = d

    def get(self, key):
        if key in self.mine:
            d = self.mine[key]
            return bool(d['ok3'] and d['env'])
        n, g, li, ri, s = key
        tq = (self.t0.get((n, g, li, s), {}).get('ok') if ri == ROT0 else self.trot.get(key, {}).get('ok3'))
        return False if tq is False else None

    def compute(self, keys, pool):
        keys = [k for k in keys if self.get(k) is None]
        if not keys:
            return
        with open(CACHE, 'a') as f:
            for d in pool.imap_unordered(_job, keys, chunksize=1):
                self.mine[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


def l1_tuples(n, rset):
    out = []
    for g in (1, 2):
        r = sched._gantry_oracles(M.CAPS[g], np.array([n]))[0][0].reshape(len(REF.lin), NR, 2)
        out += [(int(n), g, int(li), int(ri), int(s)) for li in range(len(REF.lin)) for ri in rset
                for s in (0, 1) if r[li, ri, s]]
    return out


def candidates(n):
    """The first N_CAND L1-true R35 tuples by (|arm base xy - target xy|, g, s) -- the G29 tuples_for order.
    B1 (written): A2 locked (|rail - x|, -|rot|), but the base sits +-0.4 m in x from the rail point
    (irm_sweep.base_pose), so that order spent all 40 on bases 0.4 m off target and found 2 ok of 693."""
    t = l1_tuples(n, S.RSETS['R35'])

    def rho(k):
        arm = SLOT_ARM[(k[1], k[4])]
        p = S.GO.IS.base_pose(S.GO.IRM[arm], float(REF.lin[k[2]]), float(REF.rot[k[3]]))[1]
        return float(np.linalg.norm(np.asarray(p)[:2] - X[n, :2]))
    t.sort(key=lambda k: (rho(k), k[1], k[4]))
    return t[:N_CAND]


def accept(n, F, pool):
    c = candidates(n)
    if any(F.get(k) for k in c):
        return True
    for i in range(0, len(c), 15):                         # short-circuit; 'exists' is order-free
        F.compute(c[i:i + 15], pool)
        if any(F.get(k) for k in c):
            return True
    return False


def draw(seed, F, pool, max_reject=10_000):
    rng = np.random.default_rng(seed)
    chosen, rej, dup = [], {'TENGAH': 0, 'DINDING': 0}, 0
    for name in ('TENGAH', 'DINDING'):
        got = 0
        while got < 3:
            n = int(POOL[name][rng.integers(0, len(POOL[name]))])
            if n in chosen:
                dup += 1
                continue
            if accept(n, F, pool):
                chosen.append(n)
                got += 1
            else:
                rej[name] += 1
                if rej[name] > max_reject:
                    raise RuntimeError(f'seed {seed}: kolam {name} > {max_reject} tolak')
    return chosen, rej, dup


def build(nodes, F, rset):
    """g31_sched.build verbatim except: feasibility = F (lazy at ANY rot), p0 = P0_LIN at rot 0."""
    nodes = np.array(nodes)
    cols = [li * NR + ri for li in range(len(REF.lin)) for ri in rset]
    full = np.stack(np.meshgrid(REF.lin, REF.rot, indexing='ij'), -1).reshape(-1, 2)
    poses = full[cols]
    reach, zone, hand, unver = {}, {}, {}, {}
    for g in (1, 2):
        r, z, h = sched._gantry_oracles(M.CAPS[g], nodes)
        r, z, h = r[:, cols, :].copy(), z[:, cols, :], h[:, cols]
        unver[g] = np.zeros_like(r)
        for i, n in enumerate(nodes):
            for j, c in enumerate(cols):
                li, ri = divmod(c, NR)
                for s in (0, 1):
                    if r[i, j, s]:
                        f = F.get((int(n), g, li, ri, s))
                        if f is None:
                            unver[g][i, j, s] = True
                        else:
                            r[i, j, s] = f
        reach[g], zone[g], hand[g] = r, z, h
    p0 = {g: int(np.flatnonzero((np.abs(poses[:, 0] - P0_LIN[g]) < 1e-9) & (np.abs(poses[:, 1]) < 1e-9))[0])
          for g in (1, 2)}
    inst = sched.Instance(np.array(['SR'] * len(nodes), dtype='<U2'), {g: poses for g in (1, 2)}, reach, zone,
                          hand, p0, X[nodes], sched.DWELL, 0.0, f'g38({len(rset)}rot)',
                          dict(nodes=nodes.tolist(), cols=cols))
    return replace(RS.p1prime(inst), rot_cmd=S.ROT_CMD), unver


def solve_lazy(nodes, F, rset, pool, log):
    it = 0
    while True:
        it += 1
        inst, unver = build(nodes, F, rset)
        sol = sched.solve_exact(inst)                       # ValueError = (i) fails
        used = S.used_unverified(inst, sol, unver)
        log(f'    iterasi {it}: makespan {sol.makespan:.3f}, tuple terpakai belum terverifikasi {len(used)}')
        if not used:
            return inst, sol, it
        F.compute(S.neighbours(inst, unver, used), pool)


def served(inst, sol, nodes):
    out = {}
    for g, stops in sol.stops.items():
        for st in stops:
            li, ri = divmod(inst.meta['cols'][st['pose']], NR)
            for i, who in st['assign'].items():
                out[int(nodes[i])] = dict(gantry=g, arm=who, rail=round(float(REF.lin[li]), 3),
                                          rot_deg=round(float(RDEG[ri]), 1))
    return out


def run():
    F = Feas()
    rows = []
    log = lambda s: print(s, flush=True)                    # noqa: E731
    print(f"kolam TENGAH {len(POOL['TENGAH'])}, DINDING {len(POOL['DINDING'])}; cache G38 {len(F.mine)}", flush=True)
    with Pool(15) as pool:
        for seed in SEEDS:
            try:
                nodes, rej, dup = draw(seed, F, pool)
            except RuntimeError as e:
                rows.append(dict(seed=seed, drawn=False, why=str(e)))
                print(f'seed {seed:2d}: TARIK GAGAL -- {e}', flush=True)
                continue
            rec = dict(seed=seed, drawn=True, nodes=nodes, kind=['TENGAH'] * 3 + ['DINDING'] * 3,
                       xyz=[[round(float(v), 4) for v in X[n]] for n in nodes],
                       d_env=[round(float(D_ENV[n]), 3) for n in nodes], rejects=rej, dup=dup,
                       p0={str(g): v for g, v in P0_LIN.items()}, p0_rot_deg={'1': 0.0, '2': 0.0},
                       rot_cmd=S.ROT_CMD)
            print(f'seed {seed:2d}: tertarik {nodes}, tolak {rej}, dup {dup}  [{time.strftime("%T")}]', flush=True)
            rows.append(rec)
        # R35 for every drawn seed first, then the R10 control on the first N_CTRL drawn (A3); order only.
        jobs = [(r, 'R35') for r in rows if r['drawn']] + [(r, 'R10') for r in rows if r['drawn']][:N_CTRL]
        for rec, name in jobs:
            rset, seed, nodes = S.RSETS[name], rec['seed'], rec['nodes']
            log(f'  seed {seed} {name}')
            try:
                inst, sol, it = solve_lazy(nodes, F, rset, pool, log)
            except ValueError as e:
                rec[name] = dict(ok_i=False, why=str(e))
                log(f'  seed {seed} {name}: (i) GAGAL -- {e}')
                continue
            dec = S.describe(inst, sol)
            conf = S.sched_coll.schedule_conflict(inst, sol.stops)
            ok_ii = all(v['moves'] >= 1 for v in dec.values())
            rec[name] = dict(ok_i=True, ok_ii=ok_ii, ok_iii=conf is None, makespan=sol.makespan,
                             serial=sum(sol.finish.values()), iters=it, conflict=repr(conf),
                             moves={str(g): v['moves'] for g, v in dec.items()},
                             rot_deg={str(g): v['rot_deg'] for g, v in dec.items()},
                             ntask={str(g): v['ntask'] for g, v in dec.items()},
                             served={str(k): v for k, v in served(inst, sol, nodes).items()},
                             schedule={str(g): v for g, v in MI.sched_json(inst, sol).items()})
            log(f"  seed {seed} {name}: makespan {sol.makespan:.2f}, pindah {rec[name]['moves']}, rot {rec[name]['rot_deg']}, "
                f"tugas {rec[name]['ntask']}, konflik {conf!r}, (ii) {ok_ii}  [{time.strftime('%T')}]")
            json.dump(rows, open(os.path.join(HERE, 'g38_candidates.json'), 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r.get('R35', {}).get('ok_i') and r['R35']['ok_ii'] and r['R35']['ok_iii']]
    print(f"\nlolos (i)-(iii) R35: {len(ok)}/{len(rows)} seed {ok}; cache G38 {len(F.mine)} tuple", flush=True)


def k_checks():
    """K1: _job torque fields == cached (G29 rot != 0, G24 rot 0). K2: env-model FK tool == target (< 5 mm)."""
    F = Feas()
    import pinocchio as pin
    rot_keys = [k for k, d in F.trot.items() if d['ok3']][:3]
    t0 = [(n, g, li, ROT0, s) for (n, g, li, s), d in list(F.t0.items()) if d['ok']][:3]
    bad = 0
    for key in rot_keys + t0:
        d = _job(key)
        ref = F.trot[key] if key[3] != ROT0 else F.t0[(key[0], key[1], key[2], key[4])]
        same = d['ok3'] == (ref['ok3'] if 'ok3' in ref else ref['ok']) and d['taumax'] == ref['taumax']
        bad += not same
        print(f'K1 {key}: ok3 {d["ok3"]} env {d["env"]} best {d["env_best_m"]} taumax identik {same}')
        arm = SLOT_ARM[(key[1], key[4])]
        r = O2.solve(X[key[0]], float(REF.lin[key[2]]), arm, tilt=True, envelope=True, keep_solutions=True,
                     rot=float(REF.rot[key[3]]))
        E = env_checker()
        j = base_joints(key[1], float(REF.lin[key[2]]), float(REF.rot[key[3]]))
        j.update({f'{PREFIX[arm]}joint_{i}': float(v) for i, v in enumerate(r['solutions'][0][1], 1)})
        q = E.q_from(j)
        pin.framesForwardKinematics(E.model, E.data, q)
        err = np.linalg.norm(E.data.oMf[E.model.getFrameId(f'{PREFIX[arm]}tool_frame')].translation - X[key[0]])
        bad += err > 0.005
        print(f'K2 {key}: |FK tool - target| {err * 1000:.2f} mm')
    print('K: LULUS' if not bad else f'K: GAGAL ({bad})')
    return not bad


if __name__ == '__main__':
    if sys.argv[1:] == ['k']:
        sys.exit(0 if k_checks() else 1)
    elif sys.argv[1:] == ['run']:
        run()
