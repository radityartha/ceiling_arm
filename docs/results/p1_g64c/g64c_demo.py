"""G64-C C3/C4 (docs/p1_g64c.md A1): demo_solve.py (DEMO-SYS adapter, unchanged) with the 4-mode solver. OFFLINE.

Same flow as demo_solve.main: arbitrary points -> NEW nodes (build_payload, K on every load), each task solved ALONE
(every mode combination; rejected = no combination solves it, with per-filter counts per mode), then the accepted
tasks solved together; R90; caches (REST copies + T1) inside the run dir, bound to its nodes.json.

    [G64C_ENV=1] python3 g64c_demo.py <tasks.json> <run_dir>    -> screen.json + demo_candidates.json
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_demo_sys'))
import demo_solve as DS  # noqa: E402


def main():
    spec = json.load(open(sys.argv[1]))
    run_dir = os.path.abspath(sys.argv[2])
    tasks = spec['tasks']
    pts = [t['p_cmd'] for t in tasks]
    DS._bind(run_dir, pts)
    DS.EXTRA[:] = pts
    DS._patch_load()
    sys.path.insert(0, HERE)
    import g64c_make as C
    M, K, G41, G4, S = C.M, C.K, C.G41, C.G4, C.S
    assert len(G4.X) == DS.N0 + len(pts) and np.allclose(G4.X[DS.N0:], pts), 'X tidak diperluas -- TOLAK'
    C.setup('g55R90', run_dir)
    for g in (1, 2):                                  # = demo_solve: p0 on the grid, rot 0
        lin, rot = float(spec['p0'][str(g)]), float(spec.get('p0_rot_deg', {}).get(str(g), 0.0))
        snap = float(G4.REF.lin[int(np.argmin(np.abs(G4.REF.lin - lin)))])
        if abs(snap - lin) > DS.P0_TOL_M or abs(rot) > 0.5:
            raise SystemExit(f'p0 g{g} ({lin}, {rot} deg) bukan titik grid rot 0 (±{DS.P0_TOL_M} m) -- TOLAK')
        G4.P0_LIN[g] = snap
    log = lambda s: print(s, flush=True)                                     # noqa: E731
    O = C.objects()
    cols = [li * G4.NR + ri for li in range(len(G4.REF.lin)) for ri in G4.RSET]
    nodes = [DS.N0 + i for i in range(len(pts))]
    res = dict(p0={str(g): G4.P0_LIN[g] for g in (1, 2)}, rset='R90', envelope=C.ENV, c_fold=C.C_FOLD,
               cost_provisional=C.PROVISIONAL, certified={str(g): C.certified(g) for g in (1, 2)}, tasks=[])
    log(f"p0 {res['p0']}, amplop {C.ENV}, c_fold=c_unfold {C.C_FOLD:.2f} s (sementara {C.PROVISIONAL}), "
        f"T1 tersertifikasi {res['certified']}")
    with Pool(C.NPROC) as pool:
        acc = []
        for t, n in zip(tasks, nodes):
            t0 = time.time()
            log(f'== tugas {t["id"]} node {n} p_cmd {t["p_cmd"]}  [{time.strftime("%T")}]')
            out, best = C.solve_combos([n], O, pool, log)
            r = dict(id=t['id'], node=n, p_cmd=t['p_cmd'], diterima=best is not None,
                     combos=[dict(modes={str(g): v for g, v in x['modes'].items()}, ok=x['ok'], why=x.get('why'),
                                  makespan=x.get('makespan')) for x in out])
            if best is not None:
                r.update(mode={str(g): v for g, v in out[best]['modes'].items()}, makespan_sendiri=out[best]['makespan'])
                acc.append((t, n))
            r['diag'] = {m: DS.diag(n, cols, O[m][0], G41, G4, K.sched, M, G4.NR) for m in ('REST', 'T1')}
            r['wall_s'] = round(time.time() - t0, 1)
            log(f"   {'DITERIMA mode ' + str(r['mode']) if r['diterima'] else 'DITOLAK semua kombinasi'}  ({r['wall_s']} s)")
            for m, dd in r['diag'].items():
                for arm, d in dd.items():
                    log(f'     {m} {arm}: {d}')
            res['tasks'].append(r)
            json.dump(res, open(os.path.join(run_dir, 'screen.json'), 'w'), indent=1, default=float)
        if not acc:
            log('0 tugas diterima -- tidak ada jadwal')
            return 2
        an = [n for _, n in acc]
        log(f'== solve bersama {len(an)} tugas {[t["id"] for t, _ in acc]}  [{time.strftime("%T")}]')
        out, best = C.solve_combos(an, O, pool, log)
    if best is None:
        log('solve bersama: semua kombinasi GAGAL')
        return 3
    v = C.record(out[best], dict(combos=[dict(modes={str(g): x for g, x in o['modes'].items()}, ok=o['ok'],
                                              why=o.get('why'), makespan=o.get('makespan')) for o in out]))
    row = dict(seed=0, drawn=False, nodes=an, kind=['DEMO'] * len(an), xyz=[t['p_cmd'] for t, _ in acc],
               task_ids=[t['id'] for t, _ in acc], p0=res['p0'], p0_rot_deg={'1': 0.0, '2': 0.0}, rot_cmd=S.ROT_CMD,
               DEMO=v)
    json.dump([row], open(os.path.join(run_dir, 'demo_candidates.json'), 'w'), indent=1, default=float)
    sjn = json.loads(json.dumps(v['schedule'], default=float))
    log(f"JADWAL: mode {v['mode']}, makespan {v['makespan']:.2f} s (lipat/buka sementara {C.PROVISIONAL}), konflik "
        f"{v['conflict']}, |rot| maks {v['rot_max_deg']:.0f}, LB/UB {v['bounds']}")
    for g in ('1', '2'):
        for st in sjn[g]:
            log(f'  g{g} [{v["mode"][g]}] stop rel {st["rail_m"]:.2f} rot {st["rot_deg"]:+.0f}: '
                f'{ {row["task_ids"][int(k)]: x for k, x in st["tasks"].items()} }')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
