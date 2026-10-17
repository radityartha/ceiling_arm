"""G64 task 0 (docs/p1_g64_joint_plan.md): OFFLINE diagnosis -- split failures into
  (i)  DECOUPLED PLANNING: a static whole-body configuration exists (IK + static torque + env-free task arm), but the
       current system cannot use it (both arms REST at the stop / REST traverse / straight return line), and
  (ii) PHYSICS: no static configuration of any arm at any (rail, rot) passes IK + torque + env.
No ROS, no hardware.

Per tuple (target, arm, rail, rot) = oracle''' solve (oracle2: tool-down, roll free, tilt envelope, keep_solutions):
  torque   per-branch static envelope + OFFSET + MARGIN <= LIM            (= G49 _branch_ok, the system rule)
  STATIC   torque AND task arm + own gantry structure: EnvChecker >= 50 mm AND box-free (MoveIt scene boxes)
  E1       torque AND the whole own gantry incl. the partner arm at REST: env >= 50 mm AND box-free (= G49 e1)
  POSE_A   both arms of the gantry at REST at the stop: env >= 50 mm AND box-free (= G40 pose (a), START-JUMP)
  LINE     G41 screen_line (straight joint line to REST, partner REST: torque, env, box, same-gantry)   (= G49 e2/e3)
  TRAV     REST rect sweep (G33 / pose_to_g) from the parking pose p0 to the stop, env >= 50 mm
  CURRENT  POSE_A AND TRAV AND some branch with E1 AND LINE           (= the G49/P1'' tuple predicate + traverse)
           LINE is tried on <= LINE_CAP of the E1 branches (G49 tries all) -> CURRENT can only be UNDER-counted
Raw static j2 of the best (lowest-j2) torque-ok STATIC branch is stored, so other torque bars can be re-applied later.

    python3 g64_diag.py targets              -> g64_targets.json (fixed lists + seeded capability sample)
        grid res 1 = rail 0.05 m / rot 5 deg (TAS-RAK, DEMO-OFF), 2 = 0.10 / 10 (G42), 3 = 0.10 / 15 (CAP)
    python3 g64_diag.py run [--only NAME..]  -> g64_diag.jsonl (append, restartable; one line per tuple)
    python3 g64_diag.py report               -> per-target class + reasons, g64_diag_report.json
    python3 g64_diag.py fold                 -> partial-fold (lift tool 0.10..0.40 m above REST) static torque
"""
import json
import math
import os
import sys
import time
import warnings
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path[:0] = [os.path.join(R, 'p1_g41'), os.path.join(R, 'p1_g29'), os.path.join(R, 'p1_g48'),
                os.path.join(HERE, '..', '..', '..', 'scripts')]
warnings.simplefilter('ignore')
import g41_make as G41  # noqa: E402

G4, O2, PREFIX, MARGIN = G41.G4, G41.O2, G41.PREFIX, G41.MARGIN
REST, LIM, OFFSET = G41.REST, G41.LIM, G41.OFFSET
ENV_MIN = G4.ENV_MIN
TARGETS = os.path.join(HERE, 'g64_targets.json')
OUT = os.path.join(HERE, 'g64_diag.jsonl')
REPORT = os.path.join(HERE, 'g64_diag_report.json')
P0 = {1: (1.50, 0.0), 2: (0.0, 0.0)}       # parking poses (gantry1-park-far-end, G36)
G_OTHER_LIN = {1: 0.0, 2: 1.40}            # other gantry rail (only own-gantry geometry is screened anyway)
REACH_XY = 0.88                            # horizontal prefilter, > sqrt(1.005^2 - 0.55^2) = 0.84 (z <= 1.40)
ARMS = {1: ('arm_1', 'arm_2'), 2: ('arm_3', 'arm_4')}
NPROC = int(os.environ.get('G64_NPROC', 14))
ENV_CAP = 40                               # torque-ok branches env-checked per tuple (first 10 in G41 order + 30 spread)
LINE_CAP = 12                              # straight-line checks per tuple (G49 is uncapped): first 4 in G41 order + 8 spread


_W = {}


def _env():
    if 'ec' not in _W:
        _W['ec'], _W['bx'] = G4.checkers()
        import rot_envelope as RE
        _W['RE'] = RE.load()
    return _W['ec'], _W['bx']


def base_joints(g, lin, rot):
    j = G4.G.base_joints(g, lin, rot)
    j[f't{3 - g}_linear_joint'] = G_OTHER_LIN[g]
    return j


def geom_d(ec, bx, q, idx):
    """{geometry index: (env distance, box distance)} for the given indices, ONE placement."""
    import coal
    oMg = ec.place(q)
    out = {}
    req = coal.DistanceRequest()
    for i in idx:
        g = ec.geom.geometryObjects[i]
        Mi = oMg[i]
        Rm, t = np.array(Mi.rotation), np.array(Mi.translation)
        Tg = coal.Transform3s(Rm, t)
        res = coal.DistanceResult()
        de = coal.distance(g.geometry, Tg, ec.octree, ec._I, ec._req, res)
        lo, hi = ec.world_aabb(i, Mi)
        cand = np.flatnonzero(((bx.lo <= hi + 1e-3) & (bx.hi >= lo - 1e-3)).all(1))
        db = math.inf
        for k in cand:
            res = coal.DistanceResult()
            db = min(db, coal.distance(g.geometry, Tg, coal.Box(*bx.s[k]),
                                       coal.Transform3s(np.eye(3), bx.c[k]), req, res))
        out[i] = (float(de), float(db))
    return out


def groups(ec, g, arm):
    names = [x.name for x in ec.geom.geometryObjects]
    p = PREFIX[arm]
    partner = [a for a in ARMS[g] if a != arm][0]
    own = [i for i, n in enumerate(names) if n.startswith(p)]
    struct = [i for i, n in enumerate(names) if n.startswith(f't{g}_') and not n.startswith(f't{g}_a')]
    part = [i for i, n in enumerate(names) if n.startswith(PREFIX[partner])]
    return own, struct, part


def _ok(dd):
    return all(de >= ENV_MIN and db > 0.0 for de, db in dd)


def _mins(dd):
    dd = list(dd)
    return round(min(d[0] for d in dd), 4), round(min(d[1] for d in dd), 4) if dd else None


def base_xy(ec, arm, g, lin, rot):
    import pinocchio as pin
    q = ec.q_from(base_joints(g, lin, rot))
    pin.framesForwardKinematics(ec.model, ec.data, q)
    return np.array(ec.data.oMf[ec.model.getFrameId(f'{PREFIX[arm]}base_link')].translation)


def trav_ok(g, lin, rot):
    """TRAV: REST rect sweep p0 -> stop (G33 as pose_to_g runs it). Cached per stop in the worker."""
    key = ('trav', g, round(lin, 3), round(rot, 5))
    if key not in _W:
        import g29_rot_screen as RS
        ec, _ = _env()
        v, d, wg, k = ec.screen_trajectory([f't{g}_linear_joint', f't{g}_rotation_joint'],
                                           RS.rect_points(P0[g], (lin, rot)), base_joints(g, *P0[g]))
        _W[key] = (v, round(float(d), 4), wg)
    return _W[key]


def _job(key):
    name, xyz, arm, lin, rot_deg = key
    t0 = time.process_time()
    ec, bx = _env()
    g = O2.O.GANTRY[arm]
    rot = math.radians(rot_deg)
    own, struct, part = groups(ec, g, arm)
    base = base_joints(g, lin, rot)
    q_rest = ec.q_from(base)
    d_rest = geom_d(ec, bx, q_rest, own + struct + part)
    pose_a = _ok(d_rest.values())
    d_fixed = {i: d_rest[i] for i in struct + part}
    r = O2.solve(np.asarray(xyz, float), lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
    sols, taus = r['solutions'], r.get('solutions_tau', [])
    order = sorted(range(len(sols)), key=lambda i: (float(np.max(np.abs(sols[i][1] - REST))), i))
    p = PREFIX[arm]
    n_static = n_e1 = n_line = 0
    best_static, cur, why, e1_list, n_line_chk = None, None, {}, [], 0
    j2_min_any = min((float(t[1]) for t in taus), default=None)
    tq = [i for i in order if G41_branch_ok(taus[i])]
    n_tq = len(tq)
    if n_tq > ENV_CAP:                   # first 10 in G41 order + spread over the rest (roll continuum, near-duplicates)
        tq = sorted(set(tq[:10] + [tq[int(k * (n_tq - 1) / (ENV_CAP - 11))] for k in range(ENV_CAP - 10)]), key=tq.index)
    for i in tq:
        roll, q6 = sols[i]
        j = dict(base)
        j.update({f'{p}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
        q = ec.q_from(j)
        da = geom_d(ec, bx, q, own)
        static = _ok(list(da.values()) + [d_fixed[k] for k in struct])
        if not static:
            why['static-env'] = why.get('static-env', 0) + 1
            continue
        n_static += 1
        j2 = float(taus[i][1])
        if best_static is None or j2 < best_static['j2_raw']:
            best_static = dict(j2_raw=round(j2, 4), q6=[round(float(v), 5) for v in q6],
                               env_arm=_mins(da.values()))
        e1 = _ok(list(da.values()) + list(d_fixed.values()))
        if not e1:
            why['e1-partner'] = why.get('e1-partner', 0) + 1
            continue
        n_e1 += 1
        e1_list.append(i)
    if pose_a and e1_list:               # CURRENT impossible when POSE_A fails: skip the ~0.35 s lines
        n = len(e1_list)
        pick = sorted(set(e1_list[:4] + [e1_list[int(k * (n - 1) / max(1, LINE_CAP - 5))] for k in range(LINE_CAP - 4)]))
        for i in sorted(pick, key=e1_list.index):
            q6 = sols[i][1]
            sl = G41.screen_line(g, arm, lin, rot, q6, ec, bx)
            n_line_chk += 1
            if not sl['ok']:
                why[sl['why']] = why.get(sl['why'], 0) + 1
                continue
            n_line += 1
            cur = dict(q6=[round(float(v), 5) for v in q6], j2_raw=round(float(taus[i][1]), 4))
            break
    trav = trav_ok(g, lin, rot) if (cur is not None or (best_static is not None and pose_a)) else None
    return dict(name=name, arm=arm, lin=lin, rot=rot_deg, n_sol=r['n_sol'], saturated=bool(r['saturated']),
                j2_min_any=None if j2_min_any is None else round(j2_min_any, 4),
                n_torque_ok=n_tq, n_env_checked=len(tq), n_static=n_static, n_e1=n_e1, n_line=n_line, n_line_checked=n_line_chk,
                pose_a=pose_a, rest_env=_mins(d_rest.values()), trav=trav,
                static=best_static is not None, best_static=best_static,
                current=bool(cur is not None and trav is not None and trav[0] == 'CLEAR'), cur=cur, why=why,
                cpu_s=round(time.process_time() - t0, 2))


def G41_branch_ok(tau):
    return bool(np.all(np.asarray(tau) + OFFSET + MARGIN <= LIM))


# ------------------------------------------------------------------------------------------------------- targets
def targets():
    T = [dict(name='TASRAK_z134', set='TASRAK', xyz=[0.978, -0.831, 1.34], res=1),
         dict(name='TASRAK_z140', set='TASRAK', xyz=[0.978, -0.831, 1.40], res=1),
         dict(name='DEMOOFF_P2', set='DEMOOFF', xyz=[2.30, 0.85, 1.20], res=1),
         dict(name='DEMOOFF_P3rak', set='DEMOOFF', xyz=[2.15, -0.15, 1.10], res=1),
         dict(name='DEMOOFF_P4', set='DEMOOFF', xyz=[2.05, -0.10, 1.15], res=1)]
    for d in json.load(open(os.path.join(R, 'p1_g42', 'g42f_diag.json'))):
        T.append(dict(name=f'G42_s{d["seed"]}t{d["task"]}', set='G42', kind=d['kind'], xyz=d['xyz'], res=2))
    rng = np.random.default_rng(64)
    X = G4.X
    for n in sorted(rng.choice(len(X), 20, replace=False)):
        T.append(dict(name=f'CAP_n{int(n)}', set='CAP', xyz=[round(float(v), 4) for v in X[n]], res=3))
    json.dump(T, open(TARGETS, 'w'), indent=1)
    print(f'{len(T)} target -> {TARGETS}')


def tuples(T):
    ec, _ = _env()
    env = _W['RE']
    keys = []
    for t in T:
        lins = G4.REF.lin if t['res'] == 1 else G4.REF.lin[::2]
        step = {1: 5, 2: 10, 3: 15}[t['res']]
        for g in (1, 2):
            lo, hi = env[str(g)]
            rots = [r for r in range(int(math.ceil(lo / step) * step), int(hi) + 1, step)]
            for arm in ARMS[g]:
                for lin in lins:
                    for rd in rots:
                        b = base_xy(ec, arm, g, float(lin), math.radians(rd))
                        if np.hypot(*(b[:2] - np.asarray(t['xyz'][:2]))) <= REACH_XY:
                            keys.append((t['name'], tuple(t['xyz']), arm, round(float(lin), 3), rd))
    return keys


def run(only=None):
    T = json.load(open(TARGETS))
    if only:
        T = [t for t in T if t['name'] in only or t['set'] in only]
    done = set()
    if os.path.exists(OUT):
        for ln in open(OUT):
            d = json.loads(ln)
            done.add((d['name'], d['arm'], d['lin'], d['rot']))
    keys = [k for k in tuples(T) if (k[0], k[2], k[3], k[4]) not in done]
    print(f'{len(T)} target, {len(keys)} tuple baru (sudah {len(done)})', flush=True)
    t = time.time()
    with Pool(NPROC) as pool, open(OUT, 'a') as f:
        for i, d in enumerate(pool.imap_unordered(_job, keys, chunksize=1), 1):
            f.write(json.dumps(d) + '\n')
            f.flush()
            if i % 200 == 0:
                print(f'{i}/{len(keys)} {time.time() - t:.0f} s', flush=True)
    print(f'selesai {len(keys)} dalam {time.time() - t:.0f} s', flush=True)


def report():
    T = {t['name']: t for t in json.load(open(TARGETS))}
    rows = {}
    for ln in open(OUT):
        d = json.loads(ln)
        rows.setdefault(d['name'], []).append(d)
    out, cls_n = [], {}
    for name, t in T.items():
        rr = rows.get(name, [])
        if not rr:
            continue
        st = [r for r in rr if r['static']]
        cur = [r for r in rr if r['current']]
        if cur:
            c = 'OK-NOW'
        elif st:
            c = 'SPLIT'
        elif any(r['n_sol'] > 0 for r in rr):
            c = 'PHYS-TORQUE' if not any(r['n_torque_ok'] for r in rr) else 'PHYS-ENV'
        else:
            c = 'PHYS-NOIK'
        reasons = {}
        for r in st:                       # why each STATIC tuple is not CURRENT (first failing gate)
            if r['current']:
                continue
            if not r['pose_a']:
                k = 'REST-di-stop'
            elif r['n_e1'] == 0:
                k = 'partner-REST'
            elif r['n_line'] == 0:
                k = 'garis-pulang:' + ','.join(sorted(x for x in r['why'] if x.startswith('garis') or x.startswith('torsi')))
            elif r['trav'] and r['trav'][0] != 'CLEAR':
                k = 'traverse-REST'
            else:
                k = 'lain'
            reasons[k] = reasons.get(k, 0) + 1
        j2s = sorted(r['best_static']['j2_raw'] for r in st)
        out.append(dict(name=name, set=t['set'], xyz=t['xyz'], cls=c, n_tuple=len(rr), n_ik=sum(r['n_sol'] > 0 for r in rr),
                        n_torque_ok=sum(r['n_torque_ok'] > 0 for r in rr), n_static=len(st), n_current=len(cur),
                        reasons=reasons, j2_static_min=j2s[0] if j2s else None,
                        j2_any_min=min((r['j2_min_any'] for r in rr if r['j2_min_any'] is not None), default=None)))
        cls_n.setdefault(t['set'], {}).setdefault(c, 0)
        cls_n[t['set']][c] += 1
    json.dump(dict(targets=out, by_set=cls_n), open(REPORT, 'w'), indent=1)
    for o in out:
        print(f"{o['name']:16s} {o['cls']:11s} tuple {o['n_tuple']:4d} ik {o['n_ik']:4d} tq {o['n_torque_ok']:4d} "
              f"static {o['n_static']:4d} now {o['n_current']:3d}  j2 statis min {o['j2_static_min']} (any "
              f"{o['j2_any_min']})  {o['reasons']}")
    print(json.dumps(cls_n, indent=1))


# ------------------------------------------------------------------------------------------------------- fold
def fold():
    """Static torque of partial folds: tool lifted h above its REST position, tool-down, roll free (oracle''');
    per h the branch with the lowest j2. Lowest point = min z of the arm's hull AABBs (pessimistic, the safe side)."""
    import pinocchio as pin
    ec, _ = _env()
    out = []
    for arm, g, lin in (('arm_3', 2, 0.0), ('arm_1', 1, 1.40)):
        base = base_joints(g, lin, 0.0)
        q = ec.q_from(base)
        pin.framesForwardKinematics(ec.model, ec.data, q)
        tool = np.array(ec.data.oMf[ec.model.getFrameId(f'{PREFIX[arm]}tool_frame')].translation)
        own, _, _ = groups(ec, g, arm)
        oMg = ec.place(q)
        low0 = min(ec.world_aabb(i, oMg[i])[0][2] for i in own)
        out.append(dict(arm=arm, h=0.0, j2_raw=round(float(O2.gravity(arm, REST, lin)[1]), 3), tool_z=round(float(tool[2]), 3),
                        lowest_z=round(float(low0), 3)))
        for h in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40):
            r = O2.solve(tool + [0, 0, h], lin, arm, tilt=True, envelope=True, keep_solutions=True)
            if not r['n_sol']:
                out.append(dict(arm=arm, h=h, j2_raw=None))
                continue
            i = int(np.argmin([t[1] for t in r['solutions_tau']]))
            q6 = r['solutions'][i][1]
            j = dict(base)
            j.update({f'{PREFIX[arm]}joint_{k}': float(v) for k, v in enumerate(q6, 1)})
            oMg = ec.place(ec.q_from(j))
            low = min(ec.world_aabb(k, oMg[k])[0][2] for k in own)
            tau = np.asarray(r['solutions_tau'][i])
            out.append(dict(arm=arm, h=h, j2_raw=round(float(tau[1]), 3), sys_j2=round(float(tau[1] + OFFSET[1] + MARGIN[1]), 3),
                            sys_ok=G41_branch_ok(tau), tool_z=round(float(tool[2] + h), 3), lowest_z=round(float(low), 3),
                            q6=[round(float(v), 4) for v in q6]))
    for o in out:
        print(o)
    json.dump(out, open(os.path.join(HERE, 'g64_fold.json'), 'w'), indent=1)


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'targets':
        targets()
    elif cmd == 'run':
        run(sys.argv[3:] if len(sys.argv) > 2 and sys.argv[2] == '--only' else None)
    elif cmd == 'report':
        report()
    elif cmd == 'fold':
        fold()
