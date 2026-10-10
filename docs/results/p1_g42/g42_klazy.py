"""G42 K-lazy (B): the (s42) forbid path on a case where it BINDS -- G41 predicate (box > 0, no margin) + (s42) on
seed 19 (G41: t3 -> t4 transit through the grown box, plan-only NO-PLAN). Caches = scratch copies (G41 untouched)."""
import json
import os
import sys
from multiprocessing import Pool

import g42_make as K

SC = sys.argv[1]
K.G41.VAR, K.G41.BOX_MIN, K.G41.TUPLE_CACHE = 'G41', 0.0, os.path.join(SC, 'kl_tuple.jsonl')
K.G4.EDGE_CACHE, K.STOP_CACHE = os.path.join(SC, 'kl_edge.jsonl'), os.path.join(SC, 'kl_stop.jsonl')
r = [x for x in json.load(open(os.path.join(K.R, 'p1_g41/g41_candidates.json'))) if x['seed'] == 19][0]
F, E, SX = K.G41.Feas41(K.G4.load_pose_ok()), K.G4.Edges(), K.Stops()
with Pool(K.NPROC) as pool:
    try:
        inst, sol, it = K.solve_lazy42(r['nodes'], F, E, SX, pool, print)
        sj = K.MI.sched_json(inst, sol)
        print(f"G41+(s42) s19: makespan {sol.makespan:.2f} (G41 {r['G41']['makespan']:.2f}), iterasi {it}")
        for g in (1, 2):
            print(f'  g{g} G41      : ' + ' | '.join(f"({s['rail_m']}, {s['rot_deg']}) {s['tasks']}" for s in r['G41']['schedule'][str(g)]))
            print(f'  g{g} G41+s42  : ' + ' | '.join(f"({s['rail_m']}, {s['rot_deg']}) {s['tasks']}" for s in sj[g]))
    except ValueError as e:
        print(f'G41+(s42) s19: (i) GAGAL -- {e}')
print('stop (s42): ' + ', '.join(f"{k} {'ok' if d['ok'] else 'TOLAK'}" for k, d in SX.c.items()))
