"""G52 (docs/p1_g52_r90.md): the 5 HW schedules that leave +-90 deg (s0 s2 s11 s12 s17) rescheduled with RSET R90 =
every grid rotation with |rot| <= 90 deg. OFFLINE, no ROS. Predicates, solver and stopping rule = the run that made
each R180 schedule: G43 (Feas43, ok41) for s0 s2 s11 s17, G49 (Feas49, ok49) for s12; G46 E1+E2+E3, mirror on.
Only the module global g40_make.RSET is swapped; every cache is a COPY in this directory (G43/G49 files never written).

    python3 g52_make.py k       -> K-R180: R180 through this wrapper == G43 s0 / G49 s12 (gap 0 there)
    python3 g52_make.py k90     -> K-R90: s5 (G49, |rot| <= 25) at R90 == its R180 schedule
    python3 g52_make.py run     -> g52_ok41_candidates.json (s0 s2 s11 s17) + g52_ok49_candidates.json (s12)
    python3 g52_make.py ok49 S..-> seeds S.. with the G49 predicate (fallback for an ok41 failure, B says so)
    python3 g52_make.py iv | chain -> (iv') / (iv'') of every R90 schedule that passes (i)-(iii)
"""
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g49'))
import g49_make as W  # noqa: E402

M, K, G41, G4 = W.M, W.K, W.G41, W.G4
M.RSETS['R90'] = [i for i, r in enumerate(M.RDEG) if abs(r) <= 90.0 + 1e-6]
M.G46 = True
M.MIRROR = True
SEEDS41, SEEDS49 = [0, 2, 11, 17], [12]


def _copy(src, name):
    dst = os.path.join(HERE, name)
    if not os.path.exists(dst):
        shutil.copy(src, dst)
    return dst


def setup41(rset, tag):
    """g43_make.setup with copies here (tuple + stop; G46 EdgeMatrix never reads/writes the edge cache for verdicts)."""
    K.FIX_CHOICES = True
    K.VAR = 'G43'
    G4.RSET = M.RSETS[rset]
    K.TUPLE_CACHE = G41.TUPLE_CACHE = _copy(os.path.join(R, 'p1_g43/g43_tuple_cache.jsonl'), 'g52_tuple41_cache.jsonl')
    K.EDGE_CACHE = G4.EDGE_CACHE = _copy(os.path.join(R, 'p1_g43/g43_edge_cache.jsonl'), 'g52_edge41_cache.jsonl')
    K.STOP_CACHE = _copy(os.path.join(R, 'p1_g43/g43_stop_cache.jsonl'), 'g52_stop41_cache.jsonl')
    return os.path.join(HERE, f'{tag}_candidates.json')


def setup49(rset, tag):
    """g49_make.setup with copies here."""
    K.FIX_CHOICES = True
    K.VAR = 'G49'
    G4.RSET = M.RSETS[rset]
    K.STOP_CACHE = _copy(os.path.join(R, 'p1_g49/g49_stop_cache.jsonl'), 'g52_stop49_cache.jsonl')
    W.TUPLE_CACHE = G41.TUPLE_CACHE = _copy(os.path.join(R, 'p1_g49/g49_tuple_cache.jsonl'), 'g52_tuple49_cache.jsonl')
    return os.path.join(HERE, f'{tag}_candidates.json')


def run41(rset, tag, seeds):
    M.setup, M.Feas43 = setup41, M.__dict__.get('_Feas43_orig', M.Feas43)
    M.run(rset, tag, seeds=seeds)


def run49(rset, tag, seeds):
    M._Feas43_orig = M.__dict__.get('_Feas43_orig', M.Feas43)
    M.setup, M.Feas43 = setup49, W.Feas49
    M.run(rset, tag, seeds=seeds)


def _same(path, rset, ref_path, seeds):
    a = {r['seed']: r[rset] for r in json.load(open(path))}
    b = {r['seed']: r['R180'] for r in json.load(open(ref_path))}
    for s in seeds:
        x, y = a[s], b[s]
        key = lambda v: [(t['rail_m'], t['rot_deg'], t['tasks'], t['goal']) for g in ('1', '2') for t in v['schedule'][g]]  # noqa: E731
        print(f"  s{s}: makespan {x['makespan']:.3f} vs {y['makespan']:.3f}, jadwal identik {key(x) == key(y)}")


def k():
    run41('R180', 'g52_k41', [0])
    run49('R180', 'g52_k49', [12])
    print('K-R180:')
    _same(os.path.join(HERE, 'g52_k41_candidates.json'), 'R180', os.path.join(R, 'p1_g43/g43_candidates.json'), [0])
    _same(os.path.join(HERE, 'g52_k49_candidates.json'), 'R180', os.path.join(R, 'p1_g49/g49_candidates.json'), [12])


def k90():
    run49('R90', 'g52_k90', [5])
    print('K-R90:')
    _same(os.path.join(HERE, 'g52_k90_candidates.json'), 'R90', os.path.join(R, 'p1_g49/g49_candidates.json'), [5])


def run():
    run41('R90', 'g52_ok41', SEEDS41)
    run49('R90', 'g52_ok49', SEEDS49)


def _cands():
    """One file with every R90 schedule (ok41 first, ok49 for seeds ok41 did not pass)."""
    rows = {}
    for tag in ('g52_ok49', 'g52_ok41'):
        p = os.path.join(HERE, f'{tag}_candidates.json')
        if os.path.exists(p):
            for r in json.load(open(p)):
                v = r['R90']
                if v.get('ok_i') and v['ok_ii'] and v['ok_iii']:
                    r['R90']['predicate'] = 'ok49' if tag == 'g52_ok49' else 'ok41'
                    rows[r['seed']] = r
    out = os.path.join(HERE, 'g52_candidates.json')
    json.dump([rows[s] for s in sorted(rows)], open(out, 'w'), indent=1, default=float)
    return out


def iv():
    K.VAR = 'R90'
    K.iv(_cands())


def chain():
    G41.chain('R90', _cands())


if __name__ == '__main__':
    if sys.argv[1] == 'ok49':
        run49('R90', 'g52_ok49', [int(x) for x in sys.argv[2:]])
    else:
        {'k': k, 'k90': k90, 'run': run, 'iv': iv, 'chain': chain}[sys.argv[1]]()
