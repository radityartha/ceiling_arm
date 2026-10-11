"""G55 (docs/p1_g55_par_sched.md): schedules for the PARALLEL runner. The G49 scheduler (ok49, G46 E1-E3, mirror,
henti-eps 3 %) with the G25 cost model replaced by P1'' (G53 B3, measured on the 5 G51 HW runs):
    task      a_t                                   (was c_task 43.65)
    traverse  l_bar + g * (c_rovh + max(r T_lin, ROT_CMD T_rot))     -> t_fold_first = l_bar + g c_rovh,
              rail_cmd = g r, rot_cmd = g ROT_CMD;  + a_r before every non-first move -> t_fold = a_r + t_fold_first
Predicates, caches (COPIES here) and solver untouched: feasibility does not depend on cost. The runner's exclusive
traverses are NOT in this model; every schedule is then scored by g54_sim (which has them) -- g55_eval.py.

    python3 g55_make.py run R90 S..   /   python3 g55_make.py run R35 S..
"""
import json
import os
import shutil
import sys
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g52'))
import g52_make as G52  # noqa: E402

M, K, G41, G4, W = G52.M, G52.K, G52.G41, G52.G4, G52.W
S = G4.S
C25 = json.load(open(os.path.join(R, 'p1_g25/g25_constants.json')))
P2 = json.load(open(os.path.join(R, 'p1_g53/g53_tests.json')))['calib_all']
ROT_CMD0 = S.ROT_CMD
M.RSETS['R35'] = list(M.RSETS['R35'])


def p1pp(inst, serial=True):
    tf1 = P2['l_bar'] + P2['g'] * C25['c_rovh']
    return replace(inst, dwell=P2['a_t'], t_fold=P2['a_r'] + tf1, t_fold_first=tf1, rail_cmd=P2['g'] * C25['r'],
                   serial_arms=serial)


def setup55(rset, tag):
    K.FIX_CHOICES = True
    K.VAR = 'G49'
    G4.RSET = M.RSETS[rset]

    def cp(src, name):
        dst = os.path.join(HERE, name)
        if not os.path.exists(dst):
            shutil.copy(src, dst)
        return dst
    K.STOP_CACHE = cp(os.path.join(R, 'p1_g52/g52_stop49_cache.jsonl'), 'g55_stop49_cache.jsonl')
    W.TUPLE_CACHE = G41.TUPLE_CACHE = cp(os.path.join(R, 'p1_g52/g52_tuple49_cache.jsonl'), 'g55_tuple49_cache.jsonl')
    return os.path.join(HERE, f'{tag}_candidates.json')


def run(rset, seeds):
    S.RS.p1prime = p1pp
    S.ROT_CMD = P2['g'] * ROT_CMD0
    M.setup, M.Feas43 = setup55, W.Feas49
    M.run(rset, f'g55_{rset}', seeds=seeds)




def k():
    """No cost patch: R90 s12 must equal G52 s12."""
    M.setup, M.Feas43 = setup55, W.Feas49
    M.run('R90', 'g55_k', seeds=[12])
    a = {r['seed']: r['R90'] for r in json.load(open(os.path.join(HERE, 'g55_k_candidates.json')))}[12]
    b = {r['seed']: r['R90'] for r in json.load(open(os.path.join(R, 'p1_g52/g52_ok49_candidates.json')))}[12]
    key = lambda v: [(t['rail_m'], t['rot_deg'], t['tasks'], t['goal']) for g in ('1', '2') for t in v['schedule'][g]]  # noqa: E731
    print(f"K: {a['makespan']:.3f} vs {b['makespan']:.3f}, identik {key(a) == key(b)}")


if __name__ == '__main__':
    if sys.argv[1] == 'k':
        k()
    else:
        run(sys.argv[2], [int(x) for x in sys.argv[3:]])
