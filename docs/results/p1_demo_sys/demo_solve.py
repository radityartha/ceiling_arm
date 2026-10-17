"""DEMO-SYS (docs/p1_demo_sys.md): arbitrary task points -> the P1'' solver (G55 = G49 predicates, G46 E1-E3, mirror,
P1'' cost) -> a candidates row run_g54 reads. OFFLINE, no ROS.

Operator 2026-10-17 (tombol): (1) a task point that is not a map node becomes a NEW node: its L1 mask + canon come from
capability.build_payload with the map's own cloud/filter/policy (K: rebuilt existing nodes BIT-IDENTICAL), appended to
BOTH maps inside CapabilityMap.load, so every module of the chain sees the same X; (2) every task is first solved ALONE by
the same solver -- (i) fails = REJECTED with the solver's reason + per-filter counts -- then the accepted tasks are solved
together (allocation + stops + order); (3) rotation set R90.
Predicates, solver and margins are NOT changed. Caches: COPIES of the G55 caches inside the run dir (G55 files never
written); a run dir is bound to its node list (nodes.json) -- reusing it with other points is refused.

    python3 demo_solve.py <tasks.json> <run_dir>
tasks.json: {"p0": {"1": 1.40, "2": 0.0}, "p0_rot_deg": {"1": 0.0, "2": 0.0},
             "tasks": [{"id": "M1", "p_cmd": [x, y, z]}, ...]}      (p_cmd = object top + 0.25 m, tool down)
"""
import json
import os
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
PKG = os.path.join(R, '../../ros2_ws/src/reachability_gng')
sys.path.insert(0, PKG)
from reachability_gng import capability as CAP  # noqa: E402

RSET = 'R90'
VARIANT = 'DEMO'
P0_TOL_M = 0.002                    # = run_g54 rail guard; p0 must sit on the 0.05 m grid within it
EXTRA = []                          # appended task points, set in main() BEFORE the chain is imported


def _patch_load():
    from scipy.spatial import cKDTree
    from reachability_gng.irm_sweep import approach_filter, canon_score
    cl = np.load(os.path.join(PKG, 'data/irm_cloud_pol.npz'))              # data/README: 45 deg, manip, k 64
    keep = approach_filter(cl['axis'], 45.0)
    score = canon_score('manip', cl['manip'][keep].astype(np.float64), cl['sigmin'][keep].astype(np.float64),
                        cl['q'][keep].astype(np.float64))
    tree = cKDTree(cl['poly'][keep].astype(np.float64)[:, -1])
    orig = CAP.CapabilityMap.load.__func__

    def payload(cm, pts):
        G = np.stack(np.meshgrid(cm.lin, cm.rot, indexing='ij'), -1).reshape(-1, 2)
        return CAP.build_payload(CAP.GANTRY_ARM[cm.gantry], np.asarray(pts, float), tree, score, G, cm.tols, 64)

    def load(cls, path):
        cm = orig(cls, path)
        if not EXTRA or len(cm.nodes) != N0:
            raise RuntimeError(f'{path}: {len(cm.nodes)} node, diharapkan {N0} -- TOLAK')
        idx = np.random.default_rng(cm.gantry).choice(N0, 50, replace=False)       # K on every load
        m, c = payload(cm, cm.nodes[idx])
        if not (np.array_equal(m, cm.masks[:, idx]) and np.array_equal(c, cm.canon[idx])):
            raise RuntimeError(f'{path}: build_payload != peta tersimpan -- TOLAK')
        m, c = payload(cm, EXTRA)
        cm.nodes = np.vstack([cm.nodes, np.asarray(EXTRA, float)])
        cm.masks = np.concatenate([cm.masks, m], axis=1)
        cm.canon = np.concatenate([cm.canon, c], axis=0)
        return cm
    CAP.CapabilityMap.load = classmethod(load)


N0 = 3132                           # nodes of cap_g{1,2}_rail160.npz


def _bind(run_dir, pts):
    os.makedirs(run_dir, exist_ok=True)
    f = os.path.join(run_dir, 'nodes.json')
    want = [[round(float(v), 6) for v in p] for p in pts]
    if os.path.exists(f):
        if json.load(open(f))['extra'] != want:
            raise SystemExit(f'{run_dir}: dir terikat titik lain (nodes.json) -- TOLAK, pakai dir baru')
    else:
        json.dump(dict(n0=N0, extra=want), open(f, 'w'), indent=1)


def diag(n, inst_cols, F, G41, G4, sched, M, NR):
    """Per-filter counts for node n over the R90 columns (facts only: what the solver's F knows)."""
    out = {}
    for g in (1, 2):
        r, _, _ = sched._gantry_oracles(G4.G.M.CAPS[g], np.array([n]))
        for s in (0, 1):
            arm = G4.SLOT_ARM[(g, s)]
            l1 = [(li, ri) for li, ri in (divmod(c, NR) for c in inst_cols) if r[0, li * NR + ri, s]]
            pose_bad = sum(not F.pose_ok[g][li, ri] for li, ri in l1)
            ds = [F.mine.get((n, g, li, ri, s)) for li, ri in l1 if F.pose_ok[g][li, ri]]
            known = [d for d in ds if d is not None]
            whys = {}
            for d in known:
                for k, v in d.get('whys', {}).items():
                    whys[k] = whys.get(k, 0) + v
            out[arm] = dict(L1=len(l1), pose_a_tolak=pose_bad, diverifikasi=len(known),
                            belum=len(ds) - len(known), ok=sum(bool(d['ok41']) for d in known),
                            tanpa_ik=sum(d['n_sol'] == 0 for d in known),
                            torsi_ok_cabang=sum(d['n_torque_ok'] for d in known), whys_cabang=whys)
    return out


def main():
    spec = json.load(open(sys.argv[1]))
    run_dir = os.path.abspath(sys.argv[2])
    tasks = spec['tasks']
    pts = [t['p_cmd'] for t in tasks]
    _bind(run_dir, pts)
    EXTRA[:] = pts
    _patch_load()

    sys.path.insert(0, os.path.join(R, 'p1_g55'))
    import g55_make as G55
    M, K, G41, G4, W, S = G55.M, G55.K, G55.G41, G55.G4, G55.W, G55.S
    sched, NR = K.sched, G4.NR
    assert len(G4.X) == N0 + len(pts) and np.allclose(G4.X[N0:], pts), 'X tidak diperluas -- TOLAK'

    # = g55_make.run(R90): P1'' cost, rot_cmd, Feas49; caches = copies in run_dir
    S.RS.p1prime = G55.p1pp
    S.ROT_CMD = G55.P2['g'] * G55.ROT_CMD0
    K.FIX_CHOICES, K.VAR = True, 'G49'
    G4.RSET = M.RSETS[RSET]
    for name in ('g55_stop49_cache.jsonl', 'g55_tuple49_cache.jsonl'):
        dst = os.path.join(run_dir, name)
        if not os.path.exists(dst):
            shutil.copy(os.path.join(R, 'p1_g55', name), dst)
    K.STOP_CACHE = os.path.join(run_dir, 'g55_stop49_cache.jsonl')
    W.TUPLE_CACHE = G41.TUPLE_CACHE = os.path.join(run_dir, 'g55_tuple49_cache.jsonl')

    # p0 = measured K0: on the grid, rotation 0 (build() places p0 at rot 0)
    for g in (1, 2):
        lin, rot = float(spec['p0'][str(g)]), float(spec.get('p0_rot_deg', {}).get(str(g), 0.0))
        snap = float(G4.REF.lin[int(np.argmin(np.abs(G4.REF.lin - lin)))])
        if abs(snap - lin) > P0_TOL_M or abs(rot) > 0.5:
            raise SystemExit(f'p0 g{g} ({lin}, {rot} deg) bukan titik grid rot 0 (±{P0_TOL_M} m) -- TOLAK')
        G4.P0_LIN[g] = snap
    assert G4.G.P0_LIN is G4.P0_LIN

    log = lambda s: print(s, flush=True)                                     # noqa: E731
    F = W.Feas49(G41.pose_ok_b())
    E = M.EdgeMatrix()
    K.solve_gantry_f = M.solve_gantry_e1
    G41.STOP_TORQ = True
    SX = K.Stops()
    cols = [li * NR + ri for li in range(len(G4.REF.lin)) for ri in G4.RSET]
    nodes = [N0 + i for i in range(len(pts))]
    res = dict(p0={str(g): G4.P0_LIN[g] for g in (1, 2)}, rset=RSET, tasks=[])
    with Pool(M.NPROC) as pool:
        acc = []
        for t, n in zip(tasks, nodes):
            t0 = time.time()
            log(f'== tugas {t["id"]} node {n} p_cmd {t["p_cmd"]}  [{time.strftime("%T")}]')
            r = dict(id=t['id'], node=n, p_cmd=t['p_cmd'])
            try:
                inst, sol, it, bnd = M.solve_lazy46([n], F, E, SX, pool, log)
                r.update(diterima=True, makespan_sendiri=sol.makespan, iterasi=it)
                acc.append((t, n))
            except ValueError as e:
                r.update(diterima=False, alasan=str(e))
            r['diag'] = diag(n, cols, F, G41, G4, sched, M, NR)
            r['wall_s'] = round(time.time() - t0, 1)
            log(f'   {"DITERIMA" if r["diterima"] else "DITOLAK: " + r["alasan"]}  ({r["wall_s"]} s)')
            for arm, d in r['diag'].items():
                log(f'     {arm}: {d}')
            res['tasks'].append(r)
            json.dump(res, open(os.path.join(run_dir, 'screen.json'), 'w'), indent=1, default=float)

        if not acc:
            log('0 tugas diterima -- tidak ada jadwal')
            return 2
        an = [n for _, n in acc]
        log(f'== solve bersama {len(an)} tugas {[t["id"] for t, _ in acc]}  [{time.strftime("%T")}]')
        t0 = time.time()
        inst, sol, it, bnd = M.solve_lazy46(an, F, E, SX, pool, log)
    dec = S.describe(inst, sol)
    conf = S.sched_coll.schedule_conflict(inst, sol.stops)
    sj = {str(g): v for g, v in M.MI.sched_json(inst, sol).items()}
    for g, gl in G41.goals(inst, sol, F).items():
        for st, gg in zip(sj[str(g)], gl):
            st['goal'] = gg
    sjn = json.loads(json.dumps(sj, default=float))
    mx, dmx = M.rot_stats(sjn, res['p0'])
    row = dict(seed=0, drawn=False, nodes=an, kind=['DEMO'] * len(an), xyz=[t['p_cmd'] for t, _ in acc],
               task_ids=[t['id'] for t, _ in acc], p0=res['p0'], p0_rot_deg={'1': 0.0, '2': 0.0}, rot_cmd=S.ROT_CMD)
    row[VARIANT] = dict(ok_i=True, ok_ii=all(v['moves'] >= 1 for v in dec.values()), ok_iii=conf is None,
                        makespan=sol.makespan, serial=sum(sol.finish.values()), iters=it, conflict=repr(conf),
                        wall_s=time.time() - t0, moves={str(g): v['moves'] for g, v in dec.items()},
                        ntask={str(g): v['ntask'] for g, v in dec.items()},
                        stops_s42=[dict(key=list(k), ok=SX.c[k]['ok']) for k, _ in K.stop_items(inst, sol, F)],
                        rot_max_deg=mx, drot_max_deg=dmx, bounds=bnd, schedule=sj)
    json.dump([row], open(os.path.join(run_dir, 'demo_candidates.json'), 'w'), indent=1, default=float)
    log(f'JADWAL: makespan {sol.makespan:.2f} s, konflik {conf!r}, |rot| maks {mx:.0f}, LB/UB {bnd}')
    for g in ('1', '2'):
        for st in sjn[g]:
            log(f'  g{g} stop rel {st["rail_m"]:.2f} rot {st["rot_deg"]:+.0f}: '
                f'{ {row["task_ids"][int(k)]: v for k, v in st["tasks"].items()} }')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
