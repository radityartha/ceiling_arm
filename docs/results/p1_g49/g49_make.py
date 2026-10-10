"""G49 (docs/p1_g49_exists_branch.md): the G43/G46 R180 scheduler with the tuple torque predicate on the CHOSEN branch
(exists) instead of every branch (torq2 forall). OFFLINE, no ROS.

    python3 g49_make.py ksup   -> K-SUP: 40 G43 ok41 tuples recomputed -> ok49 and same goal
    python3 g49_make.py run    -> g49_candidates.json for the seeds G43 did not pass
"""
import json
import math
import os
import random
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g43'))
import g43_make as M  # noqa: E402

K, G41, G4, S = M.K, M.G41, M.G4, M.S
O2, REF, SLOT_ARM, PREFIX = K.O2, K.REF, K.SLOT_ARM, K.PREFIX
REST, LIM, OFFSET, MARGIN = G41.REST, G41.LIM, G41.OFFSET, K.MARGIN
TUPLE_CACHE = os.path.join(HERE, 'g49_tuple_cache.jsonl')
G43_TUPLES = os.path.join(R, 'p1_g43/g43_tuple_cache.jsonl')
SEEDS = [1, 4, 5, 6, 9, 12, 13, 15, 19]
NPROC = int(os.environ.get('G49_NPROC', 15))


def _branch_ok(tau):
    """torq2's expression, for ONE branch's envelope."""
    return bool(np.all(np.asarray(tau) + OFFSET + MARGIN <= LIM))


def _tuple_job49(key):
    """G41._tuple_job (c41b) with the torque gate per branch: the first branch (G41 order) whose OWN envelope passes,
    whose end point passes env/box, and whose straight line passes screen_line."""
    t0 = time.process_time()
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(G4.X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    goal, n_e1, n_chk, n_tq, whys = None, 0, 0, 0, {}
    ec, bx = G4.checkers()
    base, p = G4.G.base_joints(g, lin, rot), PREFIX[arm]
    sols, taus = r['solutions'], r['solutions_tau']
    order = sorted(range(len(sols)), key=lambda i: (float(np.max(np.abs(sols[i][1] - REST))), i))
    for i in order:
        roll, q6 = sols[i]
        n_chk += 1
        if not _branch_ok(taus[i]):
            whys['torsi-cabang'] = whys.get('torsi-cabang', 0) + 1
            continue
        n_tq += 1
        j = dict(base)
        j.update({f'{p}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
        q = ec.q_from(j)
        d, _ = ec.check(q, only=(G4.gp(g),))
        if d < G4.ENV_MIN:
            whys['e1-env'] = whys.get('e1-env', 0) + 1
            continue
        b, _ = bx.dist(ec, q, G4.gp(g))
        if b <= 0.0 or b < G41.BOX_MIN:
            whys['e1-kotak'] = whys.get('e1-kotak', 0) + 1
            continue
        n_e1 += 1
        sl = G41.screen_line(g, arm, lin, rot, q6, ec, bx)
        if not sl['ok']:
            whys[sl['why']] = whys.get(sl['why'], 0) + 1
            continue
        goal = dict(roll_k=int(roll), q6=[float(v) for v in q6], dist_rest_deg=round(math.degrees(
            float(np.max(np.abs(q6 - REST)))), 2), line=sl, tau_branch=[float(v) for v in taus[i]])
        break
    return dict(key=[int(v) for v in key], n_sol=r['n_sol'], saturated=bool(r['saturated']),
                taumax=r['taumax'].tolist(), ok3=ok3, n_checked=n_chk, n_torque_ok=n_tq, n_e1=n_e1, whys=whys,
                ok41=goal is not None, ok49=goal is not None, goal=goal, cpu_s=round(time.process_time() - t0, 2))


class Feas49(G41.Feas41):
    """Feasibility = ok49. Facts: G43 tuples with ok3 True (ok49 == ok41 there, same first branch) + own computations;
    every forall-torque refusal of earlier caches is IGNORED (it is not an ok49 refusal). 180-deg mirror as G43 B0."""

    def __init__(self, pose_ok):
        G4.Feas2.__init__(self, pose_ok, 'exact')
        self.mine, self.g40, self.g41 = {}, {}, {}
        for ln in open(G43_TUPLES):
            d = json.loads(ln)
            if d['ok3']:
                self.mine[tuple(d['key'])] = d
        if os.path.exists(TUPLE_CACHE):
            for ln in open(TUPLE_CACHE):
                d = json.loads(ln)
                self.mine[tuple(d['key'])] = d
        M._mirror_into(self.mine, list(self.mine), M.mirror_tuple)

    def get(self, key, tab):
        if not self.pose_ok[key[1]][key[2], key[3]]:
            return False
        d = self.mine.get(key)
        return None if d is None else bool(d['ok41'])

    def compute(self, keys, pool, full=False):
        keys = sorted({k for k in keys if k not in self.mine})
        if not keys:
            return
        new = []
        with open(TUPLE_CACHE, 'a') as f:
            for d in pool.imap_unordered(_tuple_job49, keys, chunksize=1):
                self.mine[tuple(d['key'])] = d
                new.append(tuple(d['key']))
                f.write(json.dumps(d) + '\n')
                f.flush()
        M._mirror_into(self.mine, new, M.mirror_tuple)


def setup(rset, tag):
    """g43_make.setup with G49 files: candidates + stop cache here, tuple cache = TUPLE_CACHE (Feas49)."""
    K.FIX_CHOICES = True
    K.VAR = 'G49'
    G4.RSET = M.RSETS[rset]
    dst = os.path.join(HERE, 'g49_stop_cache.jsonl')
    if not os.path.exists(dst):
        shutil.copy(os.path.join(R, 'p1_g43/g43_stop_cache.jsonl'), dst)
    K.STOP_CACHE = dst
    G41.TUPLE_CACHE = TUPLE_CACHE
    return os.path.join(HERE, f'{tag}_candidates.json')


def ksup():
    rows = [json.loads(ln) for ln in open(G43_TUPLES)]
    pick = random.Random(49).sample([d for d in rows if d['ok41']], 40)
    with Pool(NPROC) as p:
        res = p.map(_tuple_job49, [tuple(d['key']) for d in pick], chunksize=1)
    same = sum(a['ok41'] == b['ok49'] and a['goal']['q6'] == b['goal']['q6'] for a, b in zip(pick, res))
    print(f'K-SUP: {same}/40 ok49 dan tujuan sama')


def run():
    M.setup = setup
    M.Feas43 = Feas49
    M.G46 = True
    M.run('R180', 'g49', seeds=SEEDS)


if __name__ == '__main__':
    {'ksup': ksup, 'run': run}[sys.argv[1]]()
