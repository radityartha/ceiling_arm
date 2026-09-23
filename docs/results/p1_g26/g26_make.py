"""G26 A1: G24 instances with p0 = rail READ at bring-up, scheduled under P1' (g25 A2).
OFFLINE. make_instance_g24.py / run_sched45.py / sched.py are NOT changed: p0 is injected
by wrapping make_instance.restrict_rot0 (the only place instance() fixes it).

    python3 g26_make.py kg1                     -> control KG1 (p0 0.55/0.00 reproduces g24 + g25)
    python3 g26_make.py run --p0 0.00 0.00      -> g26_candidates.json
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g25'))
sys.path.insert(0, os.path.join(R, 'p1_g24'))
import run_sched45 as RS  # noqa: E402
import g0_gate as G  # noqa: E402

M, MI, sched = RS.M, RS.MI, RS.sched
from make_instance import sched_coll  # noqa: E402

_ORIG = MI.restrict_rot0
CAND24 = json.load(open(os.path.join(R, 'p1_g24/g24_candidates.json')))


def instance(row, cache, p0):
    """make_instance_g24.instance verbatim, R1 = p0 (dict g -> rail m)."""
    MI.restrict_rot0 = lambda base, _ignored: _ORIG(base, p0)
    try:
        inst = M.instance(row['seed'], row['nodes'], row['rejects'], cache)[0]
        assert all(inst.poses[g][inst.p0[g], 0] == p0[g] for g in (1, 2)), 'p0 tidak tersuntik'
        return inst
    finally:
        MI.restrict_rot0 = _ORIG


def js(x):
    return json.loads(json.dumps(x, default=float))


def kg1():
    cache = M.load_cache()
    s45 = {r['seed']: r["P1'"]['opt'] for r in json.load(open(os.path.join(R, 'p1_g25/g25_sched45.json')))}
    bad, n = [], 0
    for row in CAND24:
        if not row.get('ok_ii'):
            continue
        n += 1
        inst = instance(row, cache, {1: 0.55, 2: 0.00})
        f = sched.solve_exact(MI.with_tfold(inst, MI.T_FOLD['FISIK']))
        ok = (f.makespan == row['makespan']['FISIK'] and
              js({str(g): v for g, v in MI.sched_json(inst, f).items()}) == row['schedule_FISIK'])
        p = sched.solve_exact(RS.p1prime(inst)).makespan
        ok &= p == s45[row['seed']]
        if not ok:
            bad.append(row['seed'])
    print(f'KG1: {n - len(bad)}/{n} bit-identik (FISIK makespan+jadwal g24, P1\' opt g25); beda {bad}')
    return not bad


def run(p0):
    cache = M.load_cache()
    rows = []
    for r24 in CAND24:
        seed = r24['seed']
        inst = instance(r24, cache, p0)
        row = dict(seed=seed, nodes=r24['nodes'], rejects=r24['rejects'], p0=p0,
                   schedule_key_note="schedule_FISIK/decomp.FISIK = jadwal P1' (A1 G26; nama lama untuk alat G22)")
        x = RS.p1prime(inst)
        try:
            new = sched.solve_exact(x)
            fis = sched.solve_exact(MI.with_tfold(inst, MI.T_FOLD['FISIK']))
        except ValueError as e:
            row.update(ok_i=False, why=str(e))
            rows.append(row)
            print(f'seed {seed:2d}: (i) GAGAL -- {e}')
            continue
        rep = G.eval_schedule(x, new.stops)                     # KG2
        assert all(abs(rep[g] - new.finish[g]) < 1e-9 for g in x.gantries), seed
        dec = MI.decompose(inst, new, C_TF)
        mv = {g: dec[g]['moves'] for g in (1, 2)}
        dec_f = MI.decompose(inst, fis, MI.T_FOLD['FISIK'])
        mv_f = {g: dec_f[g]['moves'] for g in (1, 2)}
        ntask = {g: bin(new.assign[g]).count('1') for g in (1, 2)}
        conf = sched_coll.schedule_conflict(inst, new.stops)
        sh = RS.shape(inst, new.stops)
        same = RS.sig(new) == RS.sig(fis)
        ok_ii = all(mv[g] >= 1 for g in (1, 2))
        row.update(ok_i=True, ok_ii=bool(ok_ii) and conf is None, ok_ii_FISIK=all(mv_f[g] >= 1 for g in (1, 2)),
                   conflict=repr(conf), moves=mv, moves_FISIK_true=mv_f, ntask=ntask,
                   retracts={g: sh[g]['retracts'] for g in (1, 2)}, same_as_FISIK=same,
                   makespan={"P1'": new.makespan, "P1'_serial": sum(new.finish.values()),
                             'FISIK': fis.makespan},
                   decomp={'FISIK': {str(g): v for g, v in dec.items()}},
                   schedule_FISIK={str(g): v for g, v in MI.sched_json(x, new).items()},
                   schedule_FISIK_true={str(g): v for g, v in MI.sched_json(inst, fis).items()})
        rows.append(row)
        print(f"seed {seed:2d}: pindah g1/g2 {mv[1]}/{mv[2]} (FISIK {mv_f[1]}/{mv_f[2]}), retract "
              f"{sh[1]['retracts']}/{sh[2]['retracts']}, tugas {ntask[1]}/{ntask[2]}, P1' {new.makespan:7.2f} "
              f"serial {sum(new.finish.values()):7.2f}, =FISIK {same}, konflik {conf!r}  "
              f"(ii) {'LOLOS' if row['ok_ii'] else '-'}", flush=True)
    out = os.path.join(HERE, 'g26_candidates.json')
    json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r.get('ok_ii')]
    print(f"\n(i) {sum(r['ok_i'] for r in rows)}/50; lolos (i)-(iii): {len(ok)} seed: {ok}\n"
          f"(ii) FISIK-sejati akan lolos: {sum(bool(r.get('ok_ii_FISIK')) for r in rows)}; "
          f"(ii) P1' != (ii) FISIK pada seed {[r['seed'] for r in rows if r.get('ok_i') and bool(r['ok_ii']) != bool(r['ok_ii_FISIK'] and r['conflict'] == 'None')]}"
          f"\n  -> {out}")


C_TF = RS.C['t_fold']

if __name__ == '__main__':
    if sys.argv[1:] == ['kg1']:
        sys.exit(0 if kg1() else 1)
    elif sys.argv[1:2] == ['run']:
        i = sys.argv.index('--p0')
        run({1: float(sys.argv[i + 1]), 2: float(sys.argv[i + 2])})
