"""G31 A6 controls for C-1 / C-2. OFFLINE, no ROS graph (probe imported, never spun).

    python3 g31_controls.py pre    -> BEFORE the patch: production classes as they are -> g31_pre.json
    python3 g31_controls.py post   -> AFTER the patch: K-C1a-d, K-C2p, K-C2n vs g31_pre.json -> g31_post.json

K-C1c / K-C2n use the 183 SAVED V28 trajectories of G28 (p1_g28/v28_plans.jsonl.gz). The cross-gantry
S18 value is taken per trajectory with a FRESH GeometryData (= a new checker object per plan, G29 B1/N2:
the pseudo-hull depends on call history, so a warm sequence would not be reproducible).
"""
import gzip
import json
import math
import os
import sys
import warnings
from types import SimpleNamespace

import numpy as np

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../../scripts'))
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
import pinocchio as pin  # noqa: E402
import interarm_collision as IC  # noqa: E402
import reach_dwell_probe as RDP  # noqa: E402
import g29_rot_screen as R  # noqa: E402

URDF = R.URDF
V28 = os.path.join(HERE, '../p1_g28/v28_plans.jsonl.gz')
MM = 1000.0


class FakeNode:
    def __init__(self):
        self.errors = []
        lg = SimpleNamespace(error=self.errors.append, info=lambda *_: None, warn=lambda *_: None)
        self.get_logger = lambda: lg


def fake_traj(names, pos):
    return SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=list(names), points=[SimpleNamespace(positions=list(p)) for p in pos]))


def plans():
    with gzip.open(V28, 'rt') as f:
        for ln in f:
            d = json.loads(ln)
            if d.get('traj') and d['traj'].get('pos'):
                yield d


def s18_cross(chk, d):
    """Cross-gantry part of probe S18 for one saved plan, exactly as screen_interarm builds it."""
    arm = d['arm']
    others = [x for x in RDP.JOINT_PREFIX if x != arm]
    g = RDP.GANTRY_OF[arm]
    want = [RDP.RAIL_JOINT[g]]
    for o in others:
        want += [f'{RDP.JOINT_PREFIX[o]}joint_{i}' for i in range(1, 7)]
        if RDP.RAIL_JOINT[RDP.GANTRY_OF[o]] not in want:
            want.append(RDP.RAIL_JOINT[RDP.GANTRY_OF[o]])
    other = {n: d['start'][n] for n in want}
    chk.geom_data = chk.geom.createData()          # fresh per plan
    v, dist, pair, k = chk.screen_trajectory(d['traj']['joint_names'], d['traj']['pos'], other,
                                             only=RDP.JOINT_PREFIX[arm])
    return dict(v=v, d_mm=dist * MM, pair=list(pair), k=k)


def s18_probe(node, d):
    """The probe's own screen_interarm (C-2 lives there), checkers shared through node._interarm."""
    arm = d['arm']
    others = [x for x in RDP.JOINT_PREFIX if x != arm]
    for c in node._interarm.values():
        c.geom_data = c.geom.createData()
    r = RDP.screen_interarm(fake_traj(d['traj']['joint_names'], d['traj']['pos']), arm, node, others,
                            other_joints=dict(d['start']))
    return None if r is None else dict(v=r[0], d_mm=r[1] * MM, pair=list(r[2]), k=r[3])


def c2p_case():
    """G29 P3: g2 rot +60 deg at lin 0.8; arm_1 (g1 lin 0.8, rot 0) driven to g2's right mount plate."""
    chk = R.RotCrossChecker()
    held = R.rest_state(0.8, 0.0, 0.8, math.radians(60))
    q = chk.q_from(held)
    pin.framesForwardKinematics(chk.model, chk.data, q)
    w = chk.data.oMf[chk.model.getFrameId('t2_mount_plate_right')].translation.copy()
    qa, conv = IC._ik(chk.model, chk.data, 't1_a1_', w, q)
    names = [f't1_a1_joint_{i}' for i in range(1, 7)]
    q_end = [float(qa[chk.model.joints[chk.model.getJointId(n)].idx_q]) for n in names]
    pos = [[r + (e - r) * t for r, e in zip(R.REST, q_end)] for t in np.linspace(0, 1, 21)]
    return held, names, pos, conv


def run_c2p(node_factory):
    held, names, pos, conv = c2p_case()
    out = dict(ik=conv)
    node = node_factory()
    r = RDP.screen_interarm(fake_traj(names, pos), 'arm_1', node, ['arm_2', 'arm_3', 'arm_4'], other_joints=held)
    out['rot60'] = None if r is None else dict(v=r[0], d_mm=r[1] * MM, pair=list(r[2]), k=r[3])
    node = node_factory()
    r = RDP.screen_interarm(fake_traj(names, pos), 'arm_1', node, ['arm_2', 'arm_3', 'arm_4'],
                            other_joints={k: v for k, v in held.items() if k != 't2_rotation_joint'})
    out['no_rot'] = None if r is None else dict(v=r[0], d_mm=r[1] * MM)
    out['no_rot_errors'] = node.errors[-1:] if node.errors else []
    node = node_factory()
    bad = ['t1_a1_joint_1', 't1_a1_joint_2', 't1_a1_joint_3', 't1_a1_joint_4', 't1_a1_joint_5', 't1_a1_jiont_6']
    r = RDP.screen_interarm(fake_traj(bad, pos), 'arm_1', node, ['arm_2', 'arm_3', 'arm_4'], other_joints=held)
    out['bad_name'] = None if r is None else dict(v=r[0], d_mm=r[1] * MM)
    out['bad_name_errors'] = node.errors[-1:] if node.errors else []
    return out


_W = {}


def _worker_state():
    if not _W:
        _W['chk'] = IC.CrossGantryChecker()
        _W['node'] = FakeNode()
        _W['node']._interarm = {}
    return _W['chk'], _W['node']


def _plan_job(k):
    """One saved plan: cross S18 (fresh), the probe's screen_interarm, and (post) the pre-C-2 recipe."""
    d = PLANS[k]
    chk, node = _worker_state()
    out = dict(i=d['i'], sample=d['sample'], arm=d['arm'], z=d['z'], verdict=d['verdict'],
               c1c=s18_cross(chk, d), c2n=s18_probe(node, d))
    if MODE == 'post':
        g = RDP.GANTRY_OF[d['arm']]
        same = node._interarm[g]
        same.geom_data = same.geom.createData()
        cr = s18_cross(node._interarm['cross'], d)
        others = [x for x in RDP.JOINT_PREFIX if x != d['arm']]
        want = [RDP.RAIL_JOINT[g]] + [f'{RDP.JOINT_PREFIX[o]}joint_{i}' for o in others for i in range(1, 7)] + \
            [r for r in RDP.RAIL_JOINT.values() if r != RDP.RAIL_JOINT[g]]
        sm = same.screen_trajectory(d['traj']['joint_names'], d['traj']['pos'], {n: d['start'][n] for n in want})
        ref = min([(sm[0], sm[1] * MM), (cr['v'], cr['d_mm'])], key=lambda r: r[1])
        p = out['c2n']
        out['c2n_ident'] = p is not None and p['v'] == ref[0] and p['d_mm'] == ref[1]
    return out


def run_plans():
    from multiprocessing import Pool
    with Pool(15) as pool:
        return pool.map(_plan_job, range(len(PLANS)), chunksize=1)


def pre():
    rows = run_plans()
    res = dict(c1c=[dict(i=r['i'], sample=r['sample'], arm=r['arm'], verdict=r['verdict'], **r['c1c']) for r in rows],
               c2n=[r['c2n'] for r in rows])
    res['c2p'] = run_c2p(FakeNode)
    json.dump(res, open(os.path.join(HERE, 'g31_pre.json'), 'w'), indent=1, default=float)
    print(f"pre: {len(res['c1c'])} lintasan V28; C-2p rot60 {res['c2p']['rot60']}; tanpa rot {res['c2p']['no_rot']}")


def k_c1a_d(rng):
    """N1 states (G29 rng 29 recipe): patched (warm) vs OldFresh; RotCrossChecker arm part vs patched."""
    pat, old, rot = IC.CrossGantryChecker(), R.OldFresh(), R.RotCrossChecker()
    names = R.arm_joint_names()
    iq = {n: pat.model.joints[pat.model.getJointId(n)].idx_q for n in names}
    da, dd, vbad = [], [], 0
    for k in range(2000):
        s = {'t1_linear_joint': rng.uniform(0, 1.6), 't2_linear_joint': rng.uniform(0, 1.6)}
        rot0 = k < 500
        s['t1_rotation_joint'] = 0.0 if rot0 else rng.uniform(-np.pi, np.pi)
        s['t2_rotation_joint'] = 0.0 if rot0 else rng.uniform(-np.pi, np.pi)
        for n in names:
            s[n] = rng.uniform(pat.model.lowerPositionLimit[iq[n]], pat.model.upperPositionLimit[iq[n]])
        dp, _ = pat.check(pat.q_from(s))
        do, _ = old.check(old.q_from(s))
        dr = rot.check_split(rot.q_from(s))[0]
        da.append(dp - do)
        dd.append(dr - dp)
        vbad += R._verdict(dp, 0.05) != R._verdict(do, 0.05)
    da, dd = np.abs(da) * MM, np.abs(dd) * MM
    return dict(c1a_max_mm=float(da.max()), c1a_n_gt_0p001=int((da > 1e-3).sum()), c1a_verdict_diff=vbad,
                c1d_max_mm=float(dd.max()))


def k_c1b():
    import g22_plan as P
    pat, old = IC.CrossGantryChecker(), R.OldFresh()
    rows = []
    for rj, csvp in {'p1_g24/g24b_run.json': '/tmp/g24b_js.csv', 'p1_g26/g26_s1_run.json': '/tmp/g26_js.csv',
                     'p1_g26/g26_s2_run.json': '/tmp/g26_js.csv', 'p1_g26/g26_s13_run.json': '/tmp/g26_js.csv'}.items():
        need = R.arm_joint_names() + ['t1_linear_joint', 't2_linear_joint', 't1_rotation_joint', 't2_rotation_joint']
        for e in json.load(open(os.path.join(HERE, '..', rj)))['events']:
            if e['kind'] != 'traverse':
                continue
            g, Rr = e['gantry'], e['rail']
            st = R.js_state_at(csvp, Rr['t_send'], need)
            st = {k: st[k] for k in need}
            s = st[f't{g}_linear_joint']
            vp = P.sweep_screen(pat, g, s, Rr['goal'], st)
            vo = P.sweep_screen(old, g, s, Rr['goal'], st)
            rows.append(dict(run=rj, g=g, logged_mm=Rr['sweep_min_mm'], new_mm=vp[1] * MM, oldfresh_mm=vo[1] * MM,
                             v_new=vp[0], v_old=vo[0]))
            print(f"  K-C1b {rj.split('/')[1][:12]} g{g}: arsip {Rr['sweep_min_mm']:.1f}, terpatch {vp[1]*MM:.3f}, "
                  f"OldFresh {vo[1]*MM:.3f}, {vp[0]}/{vo[0]}", flush=True)
    return rows


def post():
    pre_ = json.load(open(os.path.join(HERE, 'g31_pre.json')))
    res = {}
    rows = run_plans()
    c1c, c2n = [], []
    for k, r in enumerate(rows):
        a, b = pre_['c1c'][k], r['c1c']
        assert (a['i'], a['sample']) == (r['i'], r['sample'])
        c1c.append(dict(i=r['i'], sample=r['sample'], arm=r['arm'], z=r['z'], verdict=r['verdict'],
                        old_mm=a['d_mm'], new_mm=b['d_mm'], delta_mm=a['d_mm'] - b['d_mm'],
                        v_old=a['v'], v_new=b['v'], pair_old=a['pair'], pair_new=b['pair']))
        c2n.append(r['c2n'])
    res['c1c'] = c1c
    dl = np.array([r['delta_mm'] for r in c1c])
    flips = [r for r in c1c if r['v_old'] != r['v_new']]
    planned = [r for r in c1c if r['verdict'] == 'PLANNED']
    print(f"K-C1c {len(c1c)} lintasan: lama − baru min {dl.min():+.3f} / median {np.median(dl):+.3f} / maks {dl.max():+.3f} mm; "
          f"satu arah (≥ −0.01) {int((dl >= -0.01).sum())}/{len(dl)}; > 0.1 mm {int((dl > 0.1).sum())}; verdict berubah "
          f"{len(flips)} (PLANNED {sum(r['verdict'] == 'PLANNED' for r in flips)}/{len(planned)}); "
          f"min baru PLANNED {min(r['new_mm'] for r in planned):.1f} mm, semua {min(r['new_mm'] for r in c1c):.1f}")
    for r in flips:
        print(f"   berubah: i {r['i']} s{r['sample']} {r['arm']} z {r['z']} {r['verdict']}: {r['v_old']} {r['old_mm']:.1f} -> "
              f"{r['v_new']} {r['new_mm']:.1f} mm")
    ident = sum(bool(r['c2n_ident']) for r in rows)
    res['c2n_identical'] = ident
    print(f"K-C2n sesudah C-2 == resep pra-C-2 (rotasi tidak diteruskan), checker sama: {ident}/{len(rows)}")
    res['c2n_post'] = c2n
    res['c2n_pre'] = pre_['c2n']
    rng = np.random.default_rng(29)
    res.update(k_c1a_d(rng))
    print(f"K-C1a 2000 keadaan terpatch(hangat) − OldFresh: maks |Δ| {res['c1a_max_mm']:.6f} mm, > 0.001 mm "
          f"{res['c1a_n_gt_0p001']}, verdict beda {res['c1a_verdict_diff']}")
    print(f"K-C1d RotCrossChecker (hull-dari-hull) − terpatch: maks |Δ| {res['c1d_max_mm']:.6f} mm")
    res['c1b'] = k_c1b()
    res['c2p'] = run_c2p(FakeNode)
    res['c2p_pre'] = pre_['c2p']
    print(f"K-C2p sebelum: rot60 {pre_['c2p']['rot60']}  tanpa-rot {pre_['c2p']['no_rot']}")
    print(f"K-C2p sesudah: rot60 {res['c2p']['rot60']}  tanpa-rot {res['c2p']['no_rot']} {res['c2p']['no_rot_errors']}")
    json.dump(res, open(os.path.join(HERE, 'g31_post.json'), 'w'), indent=1, default=float)


MODE = sys.argv[1] if __name__ == '__main__' else None
PLANS = list(plans())

if __name__ == '__main__':
    {'pre': pre, 'post': post}[MODE]()
