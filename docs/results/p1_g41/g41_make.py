"""G41 (docs/p1_g41_joint_goal.md A1-A3): G40e scheduler with tuple predicate (c41) and JOINT task goals. OFFLINE.

(c41) = L1 AND pose (a) AND exact torq2 AND there is an oracle IK branch q (all rolls, tried in order of max |q - REST|
ascending) with
  (e1) end point: EnvChecker >= 50 mm AND box-free (= G40 env2)
  (e2) straight return line return_rest._rest_line(q) (+ q itself), partner REST, other gantry left out: EnvChecker
       screen CLEAR AND InterArmChecker(gantry g) screen CLEAR AND box-free at every point
  (e3) every line point: O2.gravity + OFFSET + MARGIN <= LIM
The FIRST such branch is the task goal (joints). (a), (b), DP, lazy loop = g40_make (G40e: exact torque, no table).

    python3 g41_make.py k     -> K1 (e2 vs plan_retract on saved task end states), K2 (e3 = gravity), K3 (4 tuples)
    python3 g41_make.py run   -> g41_candidates.json (+ g41_tuple_cache.jsonl, g41_edge_cache.jsonl)
    python3 g41_make.py iv    -> (iv') serial cross-gantry traverse check of the candidates
"""
import json
import math
import os
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g40'))
import g40_make as G4  # noqa: E402

G, S, MI, sched = G4.G, G4.S, G4.MI, G4.sched
O2, REF, NR, SLOT_ARM, MARGIN, PREFIX = G4.O2, G4.REF, G4.NR, G4.SLOT_ARM, G4.MARGIN, G4.PREFIX
X, ENV_MIN, P0_LIN, gp = G4.X, G4.ENV_MIN, G4.P0_LIN, G4.gp
VAR = 'G41'
TUPLE_CACHE = G41_CACHE = os.path.join(HERE, 'g41_tuple_cache.jsonl')
EDGE_CACHE = os.path.join(HERE, 'g41_edge_cache.jsonl')
REST = np.array(O2.O.REST)
LIM, OFFSET = O2.O.LIM, O2.O.OFFSET
BOX_MIN = 0.0                     # G41: box-free = d > 0; G41b (B4): d >= 10 mm everywhere a box is checked

_W = {}


def same_checker(g):
    if g not in _W:
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            from interarm_collision import InterArmChecker
            _W[g] = InterArmChecker(gantry=f'gantry_{g}')
    return _W[g]


def line_of(q6):
    from return_rest import _rest_line
    return [list(map(float, q6))] + _rest_line([float(v) for v in q6])


def screen_line(g, arm, lin, rot, q6, ec, bx, need_box=True):
    """(e2) + (e3) for one branch. Returns dict (all minima in m) with ok = all pass; stops at the first failing test
    (order torque -> env -> box -> same-gantry), so `why` names that first failure."""
    base = G.base_joints(g, lin, rot)
    p = PREFIX[arm]
    names = [f'{p}joint_{i}' for i in range(1, 7)]
    pts = line_of(q6)
    tau = max(float(np.max(O2.gravity(arm, np.array(q), lin, rot) + OFFSET + MARGIN - LIM)) for q in pts)
    out = dict(tau_excess=round(tau, 5))
    if tau > 0.0:
        return dict(out, ok=False, why='torsi-garis')
    placed = dict(base)
    placed.update(zip(names, pts[0]))
    v, d, wg, k = ec.screen_trajectory(names, pts, placed)
    out.update(env=v, env_d=round(float(d), 5), env_geom=wg)
    if v != 'CLEAR':
        return dict(out, ok=False, why='garis-env')
    if need_box:
        bmin = math.inf
        for q in pts:
            j = dict(base)
            j.update(zip(names, q))
            b, _ = bx.dist(ec, ec.q_from(j), gp(g))
            bmin = min(bmin, b)
            if b <= 0.0 or b < BOX_MIN:
                break
        out['box_line_d'] = round(float(bmin), 5)
        if bmin <= 0.0 or bmin < BOX_MIN:
            return dict(out, ok=False, why='garis-kotak')
    # slowest last (G41 B: 1.1 s / line); the verdict is a conjunction, so the order changes cost, not the result
    held = {n: x for n, x in placed.items() if n not in names}
    v2, d2, pair, k2 = same_checker(g).screen_trajectory(names, pts, held)
    out.update(same=v2, same_d=round(float(d2), 5), same_pair=list(pair) if not isinstance(pair, str) else pair)
    if v2 != 'CLEAR':
        return dict(out, ok=False, why='garis-se-gantry')
    return dict(out, ok=True, why='')


def _tuple_job(key):
    t0 = time.process_time()
    n, g, li, ri, s = key
    arm, lin, rot = SLOT_ARM[(g, s)], float(REF.lin[li]), float(REF.rot[ri])
    r = O2.solve(X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    ok3 = bool(O2.torq2(r, MARGIN))
    goal, n_e1, n_chk, whys = None, 0, 0, {}
    if ok3:
        ec, bx = G4.checkers()
        base, p = G.base_joints(g, lin, rot), PREFIX[arm]
        order = sorted(range(len(r['solutions'])),
                       key=lambda i: (float(np.max(np.abs(r['solutions'][i][1] - REST))), i))
        for i in order:
            roll, q6 = r['solutions'][i]
            n_chk += 1
            j = dict(base)
            j.update({f'{p}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
            q = ec.q_from(j)
            d, _ = ec.check(q, only=(gp(g),))
            if d < ENV_MIN:
                whys['e1-env'] = whys.get('e1-env', 0) + 1
                continue
            b, _ = bx.dist(ec, q, gp(g))
            if b <= 0.0 or b < BOX_MIN:
                whys['e1-kotak'] = whys.get('e1-kotak', 0) + 1
                continue
            n_e1 += 1
            sl = screen_line(g, arm, lin, rot, q6, ec, bx)
            if not sl['ok']:
                whys[sl['why']] = whys.get(sl['why'], 0) + 1
                continue
            goal = dict(roll_k=int(roll), q6=[float(v) for v in q6], dist_rest_deg=round(math.degrees(
                float(np.max(np.abs(q6 - REST)))), 2), line=sl)
            break
    return dict(key=[int(v) for v in key], n_sol=r['n_sol'], saturated=bool(r['saturated']),
                taumax=r['taumax'].tolist(), ok3=ok3, n_checked=n_chk, n_e1=n_e1, whys=whys,
                ok41=goal is not None, goal=goal, cpu_s=round(time.process_time() - t0, 2))


class Feas41(G4.Feas2):
    """G40e Feas2 (exact torque) with (c41) in place of env2 and its own cache."""

    def __init__(self, pose_ok):
        super().__init__(pose_ok, 'exact')
        self.mine = {}
        if os.path.exists(TUPLE_CACHE):
            for ln in open(TUPLE_CACHE):
                d = json.loads(ln)
                self.mine[tuple(d['key'])] = d
        self.g40 = {}
        if os.path.exists(G4.TUPLE_CACHE):
            for ln in open(G4.TUPLE_CACHE):
                d = json.loads(ln)
                self.g40[tuple(d['key'])] = d
        self.g41 = {}                              # G41b: a G41 refusal stays a refusal (B4 is stricter)
        if VAR != 'G41' and os.path.exists(G41_CACHE):
            for ln in open(G41_CACHE):
                d = json.loads(ln)
                self.g41[tuple(d['key'])] = d

    def get(self, key, tab):
        if not self.pose_ok[key[1]][key[2], key[3]]:
            return False
        if key in self.mine:
            d = self.mine[key]
            return bool(d['ok3'] and d['ok41'])
        d = self.g40.get(key)                      # (c41) implies G40 env2: a G40 refusal stays a refusal
        if d is not None and not (d['ok3'] and d['env2']):
            return False
        d = self.g41.get(key)
        if d is not None and not (d['ok3'] and d['ok41']):
            return False
        if self.exact_torque(key) is False:
            return False
        return None

    def compute(self, keys, pool, full=False):
        keys = sorted({k for k in keys if k not in self.mine})
        if not keys:
            return
        with open(TUPLE_CACHE, 'a') as f:
            for d in pool.imap_unordered(_tuple_job, keys, chunksize=1):
                self.mine[tuple(d['key'])] = d
                f.write(json.dumps(d) + '\n')
                f.flush()


def goals(inst, sol, F):
    """Per stop: {task: {joint: pos}} from the chosen branch of each used tuple."""
    out = {}
    for g, stops in sol.stops.items():
        out[g] = []
        for st in stops:
            li, ri = G4.li_ri(inst, st['pose'])
            gl = {}
            for i, who in st['assign'].items():
                s = sched.GANTRY_ARMS[g].index(who)
                key = (int(inst.meta['nodes'][i]), g, li, ri, s)
                d = F.mine[key]
                p = PREFIX[SLOT_ARM[(g, s)]]
                gl[int(i)] = {f'{p}joint_{k}': v for k, v in enumerate(d['goal']['q6'], 1)}
            out[g].append(gl)
    return out


def pose_ok_b():
    """(a) with the B4 box margin, from the G40 pose table's stored distances (no recomputation)."""
    rows = json.load(open(G4.POSE_TABLE))
    for d in rows:
        d['ok'] = bool(d['ok'] and (d['d_box'] is None or d['d_box'] >= BOX_MIN))   # None = no box near (inf)
    return G4.load_pose_ok(rows)


def set_b():
    global VAR, TUPLE_CACHE, BOX_MIN
    VAR, TUPLE_CACHE, BOX_MIN = 'G41b', os.path.join(HERE, 'g41b_tuple_cache.jsonl'), 0.010


def run():
    out = os.path.join(HERE, f'{VAR.lower()}_candidates.json')
    if not os.path.exists(EDGE_CACHE):                 # A1: G40 edges copied, new ones appended to the copy
        shutil.copy(G4.EDGE_CACHE, EDGE_CACHE)
    G4.EDGE_CACHE = EDGE_CACHE
    pose_ok = G4.load_pose_ok() if VAR == 'G41' else pose_ok_b()
    if VAR != 'G41':
        p40 = G4.load_pose_ok()
        for g in (1, 2):
            print(f'pose (a) G41b g{g}: R35 layak {int(pose_ok[g][:, G4.RSET].sum())} (G40 {int(p40[g][:, G4.RSET].sum())})')
    F, E = Feas41(pose_ok), G4.Edges()
    g40e = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g40/g40e_candidates.json')))}
    rows = []
    log = lambda s: print(s, flush=True)                    # noqa: E731
    with Pool(G4.NPROC) as pool:
        for seed in sorted(g40e):
            r40 = g40e[seed]
            nodes = r40['nodes']
            rec = {k: r40[k] for k in ('seed', 'drawn', 'nodes', 'kind', 'xyz', 'd_env', 'p0', 'p0_rot_deg', 'rot_cmd',
                                       'g38_makespan')}
            rec['g40e_makespan'] = r40['G40e'].get('makespan')
            log(f'seed {seed} {VAR}  [{time.strftime("%T")}]')
            t0 = time.time()
            try:
                inst, sol, it = G4.solve_lazy(nodes, F, E, pool, log)
            except ValueError as e:
                rec[VAR] = dict(ok_i=False, why=str(e), wall_s=time.time() - t0)
                log(f'  seed {seed}: (i) GAGAL -- {e}')
                rows.append(rec)
                json.dump(rows, open(out, 'w'), indent=1, default=float)
                continue
            dec = S.describe(inst, sol)
            conf = S.sched_coll.schedule_conflict(inst, sol.stops)
            ok_ii = all(v['moves'] >= 1 for v in dec.values())
            sj = {str(g): v for g, v in MI.sched_json(inst, sol).items()}
            for g, gl in goals(inst, sol, F).items():
                for st, gg in zip(sj[str(g)], gl):
                    st['goal'] = gg
            rec[VAR] = dict(ok_i=True, ok_ii=ok_ii, ok_iii=conf is None, makespan=sol.makespan,
                            serial=sum(sol.finish.values()), iters=it, conflict=repr(conf), wall_s=time.time() - t0,
                            moves={str(g): v['moves'] for g, v in dec.items()},
                            rot_deg={str(g): v['rot_deg'] for g, v in dec.items()},
                            ntask={str(g): v['ntask'] for g, v in dec.items()},
                            served={str(k): v for k, v in G.served(inst, sol, nodes).items()},
                            edges=[dict(key=list(k), **{x: E.c[k][x] for x in ('verdict', 'd', 'geom')})
                                   for k in G4.used_edges(inst, sol)],
                            schedule=sj)
            log(f"  seed {seed}: makespan {sol.makespan:.2f} (G40e {rec['g40e_makespan']}), pindah "
                f"{rec[VAR]['moves']}, rot {rec[VAR]['rot_deg']}, tugas {rec[VAR]['ntask']}, konflik {conf!r}, "
                f"(ii) {ok_ii}, iterasi {it}  [{time.strftime('%T')}]")
            rows.append(rec)
            json.dump(rows, open(out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r[VAR].get('ok_i') and r[VAR]['ok_ii'] and r[VAR]['ok_iii']]
    print(f"\nlolos (i)-(iii) {VAR}: {len(ok)}/{len(rows)} seed {ok}; tuple {len(F.mine)}, tepi {len(E.c)}", flush=True)
    if VAR == 'G41':
        d311()


def d311():
    """D311: of the tuples G40e verified feasible (ok3 AND env2) and recomputed here, the fraction (c41) accepts."""
    g40 = {}
    for ln in open(G4.TUPLE_CACHE):
        d = json.loads(ln)
        g40[tuple(d['key'])] = d
    mine = [json.loads(x) for x in open(TUPLE_CACHE)]
    both = [d for d in mine if tuple(d['key']) in g40 and g40[tuple(d['key'])]['ok3'] and g40[tuple(d['key'])]['env2']]
    acc = sum(d['ok41'] for d in both)
    why = {}
    for d in both:
        if not d['ok41']:
            for k, v in d['whys'].items():
                why[k] = why.get(k, 0) + v
    cpu = np.array([d['cpu_s'] for d in mine])
    print(f'D311: (c41) menerima {acc}/{len(both)} tuple layak-G40e ({100 * acc / max(1, len(both)):.1f} %); '
          f'alasan cabang ditolak (jumlah cabang, tuple ditolak): {why}')
    print(f'D317: CPU per tuple median {np.median(cpu):.2f} s, p90 {np.percentile(cpu, 90):.2f}, maks {cpu.max():.2f} '
          f'(n {len(cpu)})')
    acc_all = [d for d in mine if d['ok41']]
    if acc_all:
        dr = np.array([d['goal']['dist_rest_deg'] for d in acc_all])
        print(f'    cabang terpilih: jarak dari REST median {np.median(dr):.1f} deg; tuple dihitung {len(mine)}, '
              f'ok41 {len(acc_all)}')


# ------------------------------------------------------------------------------------------------------------ checks
def k_checks():
    import warnings
    sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
    from return_rest import plan_retract
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        cross = CrossGantryChecker()
    ec, bx = G4.checkers()
    bad = 0
    states = []
    for f in ('g40_planonly_plans.jsonl', 'g40e_planonly_plans.jsonl'):
        for ln in open(os.path.join(R, 'p1_g40', f)):
            d = json.loads(ln)
            if d.get('task') is None or not d.get('traj'):
                continue
            # K1 needs the PARTNER at REST (the predicate's assumption) -> only those end states
            st = dict(d['start'])
            st.update(zip(d['traj']['joint_names'], d['traj']['pos'][-1]))
            partner = [a for a in PREFIX if a != d['arm'] and G4.O2.O.GANTRY[a] == G4.O2.O.GANTRY[d['arm']]][0]
            if max(abs(st[f'{PREFIX[partner]}joint_{i}'] - REST[i - 1]) for i in range(1, 7)) > math.radians(0.5):
                continue
            states.append((d, st))
    print(f'K1: {len(states)} keadaan ujung tugas tersimpan dengan pasangan REST')
    for d, st in states[:40]:
        arm = d['arm']
        g = G4.O2.O.GANTRY[arm]
        lin, rot = st[f't{g}_linear_joint'], st[f't{g}_rotation_joint']
        q6 = np.array([st[f'{PREFIX[arm]}joint_{i}'] for i in range(1, 7)])
        lines = []
        v, _ = plan_retract(g, [arm], st, same_checker(g), cross, ec, None, log=lines.append)
        # parse the two minima plan_retract logs (se-gantry, lingkungan); the line itself = _rest_line from q
        mins = {}
        for s_ in lines:
            for lab in ('se-gantry', 'antar-gantry', 'lingkungan'):
                if f'penyaring {lab}:' in s_:
                    mins[lab] = (s_.split(': ')[1].split(',')[0], float(s_.split('minimum ')[1].split(' mm')[0]))
        # G41 screen with base = the SAVED state (other gantry as saved; the predicate uses base_joints, but K1 tests
        # the line screen, so hold everything as plan_retract held it)
        names = [f'{PREFIX[arm]}joint_{i}' for i in range(1, 7)]
        from return_rest import _rest_line
        pts = _rest_line(list(q6))
        ve, de, _, _ = ec.screen_trajectory(names, pts, st)
        vs, ds, _, _ = same_checker(g).screen_trajectory(names, pts, {n: x for n, x in st.items() if n not in names})
        ok = (mins['lingkungan'][0] == ve and abs(mins['lingkungan'][1] - de * 1000) <= 0.1
              and mins['se-gantry'][0] == vs and abs(mins['se-gantry'][1] - ds * 1000) <= 0.1)
        bad += not ok
        print(f"  s{d['seed']} t{d['task']} {arm}: plan_retract {v} (env {mins['lingkungan']}, se {mins['se-gantry']}, "
              f"antar {mins['antar-gantry']}) | G41 env {ve} {de * 1000:.1f}, se {vs} {ds * 1000:.1f} "
              f"{'ok' if ok else 'BEDA'}")
        # K2: (e3) at q == gravity(q) called directly
        t1 = O2.gravity(arm, q6, lin, rot)
        t2 = O2.gravity(arm, np.array(line_of(q6)[0]), lin, rot)
        if not np.array_equal(t1, t2):
            bad += 1
            print('    K2 BEDA')
    print('K2: torsi titik q bit-identik' if bad == 0 else f'K1/K2 BEDA {bad}')
    # K3: the 4 non-cross failing tuples of G40/G40e
    tup = {}
    for var, fc in (('G40', 'g40_candidates.json'), ('G40e', 'g40e_candidates.json')):
        for r in json.load(open(os.path.join(R, 'p1_g40', fc))):
            if r[var].get('ok_i'):
                for i, n in enumerate(r['nodes']):
                    v = r[var]['served'][str(n)]
                    li, ri = G4.idx_of(v['rail'], v['rot_deg'])
                    tup[(var, r['seed'], i)] = (int(n), v['gantry'], li, ri,
                                                sched.GANTRY_ARMS[v['gantry']].index(v['arm']))
    keys = {f'{v} s{s} t{t}': tup[(v, s, t)] for v, s, t in
            (('G40e', 0, 3), ('G40e', 10, 5), ('G40e', 19, 3), ('G40', 19, 3))}
    with Pool(4) as pool:
        res = pool.map(_tuple_job, list(keys.values()))
    for (lab, k), d in zip(keys.items(), res):
        g = d['goal']
        print(f'K3 {lab} {k}: ok3 {d["ok3"]} ok41 {d["ok41"]} (cabang dicek {d["n_checked"]}/{d["n_sol"]}, e1 '
              f'{d["n_e1"]}, alasan {d["whys"]}, cpu {d["cpu_s"]} s)'
              + (f' -> cabang roll {g["roll_k"]}, jarak REST {g["dist_rest_deg"]} deg, garis {g["line"]}' if g else ''))
    print('K: LULUS' if not bad else f'K: GAGAL ({bad})')
    return not bad


def iv():
    """(iv') serial cross-gantry traverse check (A3.3): g1 traverses vs g2 REST at p0, g2 traverses vs g1 REST at its
    final stop pose (the walk order with --final-retract)."""
    import warnings
    sys.path.insert(0, os.path.join(R, 'p1_g29'))
    sys.path.insert(0, os.path.join(R, 'p1_g31'))
    import g31_screen as G31S
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        import g29_rot_screen as RS
        chk = RS.RotCrossChecker()
    rows = json.load(open(os.path.join(HERE, f'{VAR.lower()}_candidates.json')))
    res = {}
    for r in rows:
        v = r[VAR]
        if not (v.get('ok_i') and v['ok_ii'] and v['ok_iii']):
            continue
        p0 = {g: r['p0'][str(g)] for g in (1, 2)}
        st = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2], 't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
        for p in PREFIX.values():
            st.update({f'{p}joint_{i}': float(x) for i, x in enumerate(REST, 1)})
        out = []
        for e in G31S.events(v['schedule'], p0, final_retract=True):
            if e['kind'] != 'traverse':
                continue
            g = e['gantry']
            sw = RS.sweep_rot(chk, g, e['frm'], e['to'], dict(st))
            out.append(sw['verdict'])
            print(f"  s{r['seed']} g{g} ({e['frm'][0]:.2f},{math.degrees(e['frm'][1]):+.0f})->({e['to'][0]:.2f},"
                  f"{math.degrees(e['to'][1]):+.0f}): {sw['verdict']} {sw['d'] * 1000:.1f} mm {sw['pair']}")
            st[f't{g}_linear_joint'], st[f't{g}_rotation_joint'] = e['to']
        res[r['seed']] = all(x == 'CLEAR' for x in out)
    print(f"(iv') lolos {sum(res.values())}/{len(res)} {res}")
    json.dump(res, open(os.path.join(HERE, f'{VAR.lower()}_iv.json'), 'w'))


def chain(var, cand):
    """(iv'', B4) offline serial walk with --final-retract and the JOINT goals: every task transit = straight joint line
    from the arm's state in the walk (REST after a retract, else the previous goal) to its goal, everything else held
    as the walk holds it (partner possibly extended, other gantry where it is): env, box >= BOX_MIN, same-gantry,
    cross-gantry (only=t{g}_a, as plan_retract), torque. Retract lines are screened the same way."""
    import warnings
    sys.path.insert(0, os.path.join(R, 'p1_g31'))
    import g31_screen as G31S
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker
        cross = CrossGantryChecker()
    from return_rest import _rest_line
    ec, bx = G4.checkers()
    res = {}
    for r in json.load(open(os.path.join(HERE, cand))):
        v = r[var]
        if not (v.get('ok_i') and v['ok_ii'] and v['ok_iii']):
            continue
        p0 = {g: r['p0'][str(g)] for g in (1, 2)}
        st = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2], 't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
        for p in PREFIX.values():
            st.update({f'{p}joint_{i}': float(x) for i, x in enumerate(REST, 1)})
        bad = []

        def screen(g, names, pts, lab):
            held = {n: x for n, x in st.items() if n not in names}
            ve = ec.screen_trajectory(names, pts, st)
            vs = same_checker(g).screen_trajectory(names, pts, held)
            vc = cross.screen_trajectory(names, pts, held, only=f't{g}_a')
            bmin = math.inf
            for q in pts:
                j = dict(st)
                j.update(zip(names, q))
                bmin = min(bmin, bx.dist(ec, ec.q_from(j), gp(g))[0])
            arms = sorted({n[:6] for n in names})
            tau = -math.inf
            for a in arms:
                arm = [k for k, p in PREFIX.items() if p == a][0]
                ix = [names.index(f'{a}joint_{i}') for i in range(1, 7)]
                for q in pts:
                    tau = max(tau, float(np.max(O2.gravity(arm, np.array([q[i] for i in ix]), st[f't{g}_linear_joint'],
                                                           st[f't{g}_rotation_joint']) + OFFSET + MARGIN - LIM)))
            ok = (ve[0] == vs[0] == vc[0] == 'CLEAR' and bmin >= max(BOX_MIN, 1e-12) and tau <= 0.0)
            print(f"  s{r['seed']} {lab}: env {ve[0]} {ve[1] * 1000:.1f}, se {vs[0]} {vs[1] * 1000:.1f}, antar {vc[0]} "
                  f"{vc[1] * 1000:.1f}, kotak {bmin * 1000:.2f} mm, torsi lebih {tau:+.2f} -> {'ok' if ok else 'TOLAK'}")
            if not ok:
                bad.append(lab)

        for k, e in enumerate(G31S.events(v['schedule'], p0, final_retract=True)):
            g = e['gantry']
            if e['kind'] == 'traverse':
                st[f't{g}_linear_joint'], st[f't{g}_rotation_joint'] = e['to']
            elif e['kind'] == 'retract':
                arms = [a for a in PREFIX if G4.O2.O.GANTRY[a] == g]
                names = [f'{PREFIX[a]}joint_{i}' for a in arms for i in range(1, 7)]
                cur = [st[n] for n in names]
                if max(abs(c - x) for c, x in zip(cur, list(REST) * len(arms))) >= math.radians(0.5):
                    screen(g, names, _rest_line(cur), f'[{k}] retract g{g}')
                    st.update(zip(names, list(map(float, REST)) * len(arms)))
            else:
                names = list(e['goal'])
                s0 = np.array([st[n] for n in names])
                q1 = np.array([e['goal'][n] for n in names])
                f = [0.5 * (1.0 - math.cos(math.pi * i / 60)) for i in range(0, 61)]
                pts = [list(s0 + (q1 - s0) * x) for x in f]
                rest0 = np.max(np.abs(s0 - REST)) < math.radians(0.5)
                screen(g, names, pts, f"[{k}] t{e['task']} {e['arm']} dari {'REST' if rest0 else 'ujung tugas sebelumnya'}")
                st.update(zip(names, map(float, q1)))
        res[r['seed']] = bad
    print(f"(iv'') {var}: lolos {sum(not b for b in res.values())}/{len(res)} " + str({s: b for s, b in res.items()}))
    return res


if __name__ == '__main__':
    a = sys.argv[1]
    if a.endswith('_b'):
        set_b()
    {'k': lambda: sys.exit(0 if k_checks() else 1), 'run': run, 'run_b': run, 'iv': iv, 'iv_b': iv, 'd311': d311,
     'chain': lambda: chain('G41', 'g41_candidates.json'),
     'chain_b': lambda: chain('G41b', 'g41b_candidates.json')}[a]()
