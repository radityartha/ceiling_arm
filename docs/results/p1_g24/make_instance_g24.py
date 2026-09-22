"""G24 A5: tasks drawn from ORACLE''-feasible nodes, then g22 A2 (i)-(iii) verbatim.

gen_real's loop is copied with ONE predicate replaced (A5); K1 checks the copy
reproduces gen_real exactly under gen_real's own predicate. ORACLE'' is computed
LAZILY (A3) for every L1-true rot=0 tuple of each drawn candidate node, cached in
g24_oracle_cache.jsonl (one line per tuple, restartable). OFFLINE, zero hardware.

    python3 make_instance_g24.py k1          -> K1 only
    python3 make_instance_g24.py run         -> g24_candidates.json
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
sys.path.insert(0, os.path.join(HERE, '../p1_g23'))
import make_instance as MI  # noqa: E402
from make_instance import sched, sched_coll  # noqa: E402
import oracle2 as O2  # noqa: E402
from reachability_gng.capability import CapabilityMap  # noqa: E402

MARGIN = np.array(json.load(open(os.path.join(HERE, '../p1_g23/g23_calib2.json')))['margin'])
SLOT_ARM = {(1, 0): 'arm_1', (1, 1): 'arm_2', (2, 0): 'arm_3', (2, 1): 'arm_4'}   # sched.GANTRY_ARMS
CACHE = os.path.join(HERE, 'g24_oracle3_cache.jsonl')      # ORACLE''' (A')
CAPS = {g: CapabilityMap.load(MI.MAPS[g - 1]) for g in (1, 2)}
REF = CAPS[1]
ROT0 = int(np.argmin(np.abs(REF.rot)))
assert abs(REF.rot[ROT0]) < 1e-12 and np.array_equal(CAPS[2].nodes, REF.nodes)


def l1_rot0(node):
    """{(g, lin_idx, slot)} L1-true at rot = 0 (gen_real's own masks)."""
    out = []
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(CAPS[g], np.array([node]))
        r = r[0].reshape(len(REF.lin), len(REF.rot), 2)[:, ROT0, :]
        out += [(g, int(p), int(s)) for p, s in np.argwhere(r)]
    return out


def draw(seed, accept, n_tasks=6, max_reject=10_000):
    """gen_real's loop (sched.py) verbatim except the acceptance predicate."""
    rng = np.random.default_rng(seed)
    chosen, rejects = [], 0
    while len(chosen) < n_tasks:
        cand = int(rng.integers(0, len(REF.nodes)))
        if (yield cand):
            chosen.append(cand)
        else:
            rejects += 1
            if rejects > max_reject:
                raise RuntimeError('generator could not find feasible tasks')
    return chosen, rejects


def accept_l1_any(cand):
    ok = False
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(CAPS[g], np.array([cand]))
        ok |= bool(r[0].any())
    return ok


def run_draw(seed, accept):
    gen = draw(seed, accept)
    cand = next(gen)
    while True:
        try:
            cand = gen.send(accept(cand))
        except StopIteration as e:
            return e.value


def k1():
    bad = []
    for seed in range(50):
        base = sched.gen_real(6, seed, 0, (1, 2), maps=MI.MAPS)
        ch, rj = run_draw(seed, accept_l1_any)
        if ch != base.meta['nodes'] or rj != base.meta['rejects']:
            bad.append(seed)
    print(f'K1: loop salinan == gen_real pada {50 - len(bad)}/50 seed; beda {bad}')
    return not bad


# ---------------------------------------------------------------- lazy oracle
def _job(t):
    node, g, p, s = t
    r = O2.solve(REF.nodes[node], float(REF.lin[p]), SLOT_ARM[(g, s)], tilt=True, envelope=True)
    return dict(node=node, g=g, p=p, s=s, n_sol=r['n_sol'], n_roll=r['n_roll'],
                rounds=r['rounds'], saturated=r['saturated'], taumax=r['taumax'].tolist(),
                tilt_fail=r['tilt_fail'],
                ok=bool(O2.torq2(r, MARGIN)))       # L1 is true by construction of the job list


def load_cache():
    c = {}
    if os.path.exists(CACHE):
        for ln in open(CACHE):
            d = json.loads(ln)
            c[(d['node'], d['g'], d['p'], d['s'])] = d
    return c


def ensure(nodes, cache, pool):
    todo = [(n, g, p, s) for n in nodes for g, p, s in l1_rot0(n) if (n, g, p, s) not in cache]
    if not todo:
        return
    with open(CACHE, 'a') as f:
        for d in pool.imap_unordered(_job, todo, chunksize=1):
            cache[(d['node'], d['g'], d['p'], d['s'])] = d
            f.write(json.dumps(d) + '\n')
            f.flush()


def node_ok(n, cache):
    return any(cache[(n, g, p, s)]['ok'] for g, p, s in l1_rot0(n))


def draw_all(seeds, cache, pool):
    """Waves: every unfinished seed's next candidate is computed together."""
    gens, state, res = {}, {}, {}
    for sd in seeds:
        gens[sd] = draw(sd, None)
        state[sd] = next(gens[sd])
    wave = 0
    while state:
        wave += 1
        ensure(sorted(set(state.values())), cache, pool)
        for sd in list(state):
            try:
                state[sd] = gens[sd].send(node_ok(state[sd], cache))
            except StopIteration as e:
                res[sd] = e.value
                del state[sd]
        print(f'  gelombang {wave}: selesai {len(res)}/{len(seeds)} seed, tuple di cache {len(cache)}',
              flush=True)
    return res


def instance(seed, nodes, rejects, cache):
    node_idx = np.array(nodes)
    reach, zone, hand = {}, {}, {}
    for g in (1, 2):
        reach[g], zone[g], hand[g] = sched._gantry_oracles(CAPS[g], node_idx)
    poses = np.stack(np.meshgrid(REF.lin, REF.rot, indexing='ij'), -1).reshape(-1, 2)
    p0_idx = int(np.argmin(np.hypot(poses[:, 0] - REF.lin[0], poses[:, 1])))
    base = sched.Instance(np.array(['SR'] * 6, dtype='<U2'), {g: poses for g in (1, 2)}, reach,
                          zone, hand, {g: p0_idx for g in (1, 2)}, REF.nodes[node_idx], sched.DWELL,
                          0.0, f'real_oracle2(n=6,mr=0,seed={seed})',
                          dict(nodes=node_idx.tolist(), rejects=rejects, n_poses=len(poses)))
    inst = MI.restrict_rot0(base, {1: 0.55, 2: 0.00})
    cnt = {}
    for g in (1, 2):
        r = inst.reach[g].copy()
        li = [int(np.argmin(abs(REF.lin - L))) for L in inst.poses[g][:, 0]]
        assert all(abs(REF.lin[k] - L) < 1e-9 for k, L in zip(li, inst.poses[g][:, 0]))
        before = int(r.sum())
        for i, n in enumerate(nodes):
            for pi, p in enumerate(li):
                for s in (0, 1):
                    if r[i, pi, s]:
                        r[i, pi, s] = cache[(n, g, p, s)]['ok']
        reach[g] = r
        cnt[g] = (before, int(r.sum()))
    return sched.Instance(inst.kind, inst.poses, {g: reach[g] for g in (1, 2)}, inst.zone, inst.hand,
                          inst.p0, inst.xyz, inst.dwell, inst.t_fold, inst.label + "+oracle'''",
                          dict(inst.meta)), cnt


def main_run(fallback=False):
    cache = load_cache()
    with Pool(15) as pool:
        drawn = draw_all(range(50), cache, pool)
    rows = []
    for seed in range(50):
        nodes, rejects = drawn[seed]
        inst, cnt = instance(seed, nodes, rejects, cache)
        row = dict(seed=seed, nodes=nodes, rejects=rejects,
                   reach_true_L1_to_oracle2={str(g): v for g, v in cnt.items()})
        # ---- g22 A2 (i)-(iii), as make_instance_g23.main ----
        try:
            sols = {k: sched.solve_exact(MI.with_tfold(inst, tf)) for k, tf in MI.T_FOLD.items()}
        except ValueError as e:
            dead = [i for i in range(len(inst.kind))
                    if not any(inst.reach[g][i].any() for g in inst.gantries)]
            row.update(ok_i=False, why=str(e), infeasible_tasks=dead)
            rows.append(row)
            print(f'seed {seed:2d}: (i) GAGAL -- {dead}  [K2: bug]')
            continue
        f = sols['FISIK']
        dec = {k: MI.decompose(inst, s, MI.T_FOLD[k]) for k, s in sols.items()}
        mv = {g: dec['FISIK'][g]['moves'] for g in (1, 2)}
        ntask = {g: bin(f.assign[g]).count('1') for g in (1, 2)}
        ok_ii = all(mv[g] >= 1 for g in (1, 2)) if not fallback else \
            (sum(mv.values()) >= 1 and all(ntask[g] >= 1 for g in (1, 2)))
        conf = sched_coll.schedule_conflict(inst, f.stops)
        same = all([(s['pose'], s['tasks']) for s in sols['FISIK'].stops[g]] ==
                   [(s['pose'], s['tasks']) for s in sols['DINDING'].stops[g]] for g in (1, 2))
        row.update(ok_i=True, ok_ii=bool(ok_ii) and conf is None, conflict=repr(conf),
                   moves=mv, ntask=ntask, same_schedule_FISIK_DINDING=same,
                   makespan={k: s.makespan for k, s in sols.items()},
                   decomp={k: {str(g): v for g, v in d.items()} for k, d in dec.items()},
                   schedule_FISIK={str(g): v for g, v in MI.sched_json(inst, f).items()},
                   schedule_DINDING={str(g): v for g, v in MI.sched_json(inst, sols['DINDING']).items()})
        rows.append(row)
        print(f'seed {seed:2d}: tolak-tarik {rejects:3d}, pindah g1/g2 {mv[1]}/{mv[2]}, tugas {ntask[1]}/{ntask[2]}, '
              f'makespan FISIK {f.makespan:7.2f} DINDING {sols["DINDING"].makespan:7.2f}, '
              f'jadwal sama {same}, konflik {conf!r}  (ii) {"LOLOS" if row["ok_ii"] else "-"}')
    out = os.path.join(HERE, 'g24_candidates.json' if not fallback else 'g24_candidates_fallback.json')
    json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r.get('ok_ii')]
    nd = {}
    for (n, g, p, s), d in cache.items():
        nd.setdefault(n, False)
        nd[n] |= d['ok']
    print(f"\n(i) {sum(r['ok_i'] for r in rows)}/50 (K2 harus 50); lolos (i)-(iii): {len(ok)} seed: {ok}")
    print(f"node kandidat dihitung {len(nd)}, layak-oracle'' {sum(nd.values())}; "
          f"tuple {len(cache)}, ORACLE'' benar {sum(d['ok'] for d in cache.values())}  -> {out}")


if __name__ == '__main__':
    if sys.argv[1:] == ['k1']:
        sys.exit(0 if k1() else 1)
    elif sys.argv[1:2] == ['run']:
        main_run(fallback='--fallback' in sys.argv)
