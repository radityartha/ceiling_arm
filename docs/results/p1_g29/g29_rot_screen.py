"""G29 A1: S18/S24 with gantry ROTATION -- new module; interarm_collision.py is NOT changed.

CrossGantryChecker drops the 16 structure-structure (SS) t1 x t2 pairs on the premise that the
rails are parallel and the gantries only translate past each other (docstring there). That premise
is void at rot != 0. RotCrossChecker keeps the parent's 660 arm-bearing pairs at the SAME indices
and appends the 16 SS pairs, so its arm part is the parent's computation, not a second one.

Joint names are STRICT: the parent's q_from silently ignores a name it does not know, and the probe's
S18 silently evaluates rot = 0 because it never passes the rotation joints (docs/p1_g29_rot.md A0.1).
Here an unknown name raises, and a sweep refuses a state without both rotation joints.

sweep_rot(mode='rect') screens the full [lin0, lin1] x [rot0, rot1] grid (<= 10 mm, <= 1 deg), which
covers ANY path monotone per axis -- the bridge dispatches absolute targets and both axes run
concurrently at their own speeds, so the real path is not a straight line in (lin, rot). Rotation is
NOT wrapped: the motor goes to an absolute encoder target in [-180, 180] deg (A0.3).

TRUE convex hulls (post-lock, B1b): coal on this machine is built WITHOUT qhull, so the parent's
buildConvexRepresentation keeps ALL mesh vertices with the (non-convex) mesh adjacency; GJK's
hill-climbing support then stops at local maxima and over-reads distances (195.9 vs mesh 54.4 mm).
Every arm geometry is replaced by the scipy ConvexHull of those points, handed to coal.Convex.

FRESH GeometryData on every evaluation as a second belt: with the pseudo-hull, distances also depended
on call history (+129 / -119 mm on one pair); with true hulls warm vs fresh differ <= 0.003 mm (B1).

    python3 g29_rot_screen.py controls     -> N1 N2 N3 P1 P2 P3 P4 (A2), g29_controls.json
"""
import csv
import glob
import json
import math
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../../scripts'))
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
sys.path.insert(0, os.path.join(HERE, '../../../ros2_ws/src/reachability_gng'))
import pinocchio as pin  # noqa: E402
from interarm_collision import CrossGantryChecker, ARM_PREFIX, _ik  # noqa: E402

URDF = os.path.join(HERE, '../p1_g23/reach_dwell_live.urdf')     # sha f02e7c53 = /tmp copy G22-G28
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
MARGIN = 0.05
LIN_STEP = 0.010
ROT_STEP = math.radians(1.0)
ARM = tuple(ARM_PREFIX.values())
V_LIN = 3000 / 95.4930 / 1000.0          # m/s  (p1_state 5.6)
V_ROT = math.radians(10.0)               # rad/s
T0_LIN, T0_ROT = 0.29, 0.26


def true_hull(g):
    """coal.Convex of the scipy ConvexHull of g's points (g = the parent's pseudo-hull)."""
    import coal
    from scipy.spatial import ConvexHull
    X = np.array([g.points(t) for t in range(g.num_points)])
    H = ConvexHull(X)
    idx = {v: i for i, v in enumerate(H.vertices)}
    pts, tris = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
    for v in H.vertices:
        pts.append(X[v])
    for a, b, c in H.simplices:
        tris.append(coal.Triangle(idx[a], idx[b], idx[c]))
    return coal.Convex(pts, tris)


class RotCrossChecker(CrossGantryChecker):
    def __init__(self, urdf=URDF, margin=MARGIN, fix_hull=True):
        super().__init__(urdf=urdf, margin=margin)
        if fix_hull:
            for go in self.geom.geometryObjects:
                if hasattr(go.geometry, 'num_points'):
                    go.geometry = true_hull(go.geometry)
        self.n_arm = len(self.geom.collisionPairs)
        ia = [i for i, n in enumerate(self.names) if n.startswith('t1_') and not n.startswith(ARM)]
        ib = [i for i, n in enumerate(self.names) if n.startswith('t2_') and not n.startswith(ARM)]
        for i in ia:
            for j in ib:
                self.geom.addCollisionPair(pin.CollisionPair(i, j))
        self.geom_data = self.geom.createData()
        self.n_ss = len(self.geom.collisionPairs) - self.n_arm
        assert self.n_ss == 16, self.n_ss

    def q_from(self, joint_positions, q=None):
        for name in joint_positions:
            if not self.model.existJointName(name):
                raise KeyError(f'sendi tidak ada di model: {name!r}')
        return super().q_from(joint_positions, q)

    def check_split(self, q, only=None):
        self.geom_data = self.geom.createData()
        pin.updateGeometryPlacements(self.model, self.data, self.geom, self.geom_data, q)
        pin.computeDistances(self.model, self.data, self.geom, self.geom_data, q)
        da, pa, ds, ps = float('inf'), None, float('inf'), None
        for k, res in enumerate(self.geom_data.distanceResults):
            cp = self.geom.collisionPairs[k]
            a, b = self.names[cp.first], self.names[cp.second]
            if k < self.n_arm:
                if only and not (a.startswith(only) or b.startswith(only)):
                    continue
                if res.min_distance < da:
                    da, pa = res.min_distance, (a, b)
            elif not only and res.min_distance < ds:
                ds, ps = res.min_distance, (a, b)
        return da, pa, ds, ps

    def check(self, q, only=None):
        da, pa, ds, ps = self.check_split(q, only)
        return (da, pa) if da <= ds else (ds, ps)

    def by_class(self, q):
        """{LL, LS, SS: (d, pair)} over all pairs."""
        self.geom_data = self.geom.createData()
        pin.updateGeometryPlacements(self.model, self.data, self.geom, self.geom_data, q)
        pin.computeDistances(self.model, self.data, self.geom, self.geom_data, q)
        out = {}
        for k, res in enumerate(self.geom_data.distanceResults):
            cp = self.geom.collisionPairs[k]
            a, b = self.names[cp.first], self.names[cp.second]
            sa, sb = a.startswith(ARM), b.startswith(ARM)
            c = 'LL' if sa and sb else ('SS' if not (sa or sb) else 'LS')
            if c not in out or res.min_distance < out[c][0]:
                out[c] = (res.min_distance, (a, b))
        return out


def _verdict(d, margin):
    return 'COLLIDE' if d <= 0.0 else ('MARGIN' if d < margin else 'CLEAR')


def rect_points(frm, to):
    (l0, r0), (l1, r1) = frm, to
    nl = max(2, int(math.ceil(abs(l1 - l0) / LIN_STEP)) + 1)
    nr = 1 if r1 == r0 else max(2, int(math.ceil(abs(r1 - r0) / ROT_STEP)) + 1)
    L = np.linspace(l0, l1, nl)
    R = np.array([r0]) if nr == 1 else np.linspace(r0, r1, nr)
    return [(l, r) for l in L for r in R]


def path_points(frm, to):
    """G9 A2.3: dead time, then constant speed per axis, concurrent; sampled <= 10 mm and <= 1 deg."""
    (l0, r0), (l1, r1) = frm, to
    tl = (T0_LIN + abs(l1 - l0) / V_LIN) if l1 != l0 else 0.0
    tr = (T0_ROT + abs(r1 - r0) / V_ROT) if r1 != r0 else 0.0
    T = max(tl, tr)
    dt = min(LIN_STEP / V_LIN, ROT_STEP / V_ROT)
    n = max(2, int(math.ceil(T / dt)) + 1)
    pts = []
    for t in np.linspace(0.0, T, n):
        l = l0 + math.copysign(min(max(t - T0_LIN, 0.0) * V_LIN, abs(l1 - l0)), l1 - l0) if l1 != l0 else l0
        r = r0 + math.copysign(min(max(t - T0_ROT, 0.0) * V_ROT, abs(r1 - r0)), r1 - r0) if r1 != r0 else r0
        pts.append((l, r))
    return pts


def sweep_rot(chk, g, frm, to, state, mode='rect'):
    """Gantry g swept (lin, rot) frm -> to, everything else HELD at `state` (must hold BOTH rotations)."""
    lj, rj = f't{g}_linear_joint', f't{g}_rotation_joint'
    for need in ('t1_rotation_joint', 't2_rotation_joint', 't1_linear_joint', 't2_linear_joint'):
        if need not in state:
            raise KeyError(f'keadaan tanpa {need} -- menolak (rotasi tidak punya default)')
    pts = rect_points(frm, to) if mode == 'rect' else path_points(frm, to)
    base = chk.q_from({k: v for k, v in state.items() if k not in (lj, rj)})
    ia, ir = (chk.model.joints[chk.model.getJointId(j)].idx_q for j in (lj, rj))
    best = (float('inf'), None, -1)
    da_min, ds_min = float('inf'), float('inf')
    for k, (l, r) in enumerate(pts):
        q = base.copy()
        q[ia], q[ir] = l, r
        da, pa, ds, ps = chk.check_split(q)
        da_min, ds_min = min(da_min, da), min(ds_min, ds)
        d, p = (da, pa) if da <= ds else (ds, ps)
        if d < best[0]:
            best = (d, p, k)
    return dict(verdict=_verdict(best[0], chk.margin), d=best[0], pair=best[1], k=best[2],
                at=pts[best[2]], d_arm=da_min, d_ss=ds_min, n=len(pts))


# ------------------------------------------------------------------------------------ controls
def rest_state(l1, r1, l2, r2):
    s = {'t1_linear_joint': l1, 't2_linear_joint': l2, 't1_rotation_joint': r1, 't2_rotation_joint': r2}
    for p in ARM:
        s.update({f'{p}joint_{i}': v for i, v in enumerate(REST, 1)})
    return s


def arm_joint_names():
    return [f'{p}joint_{i}' for p in ARM for i in range(1, 7)]


class OldFresh(CrossGantryChecker):
    """The parent, unchanged, except TRUE hulls (B1b) and a fresh GeometryData per check (B1) -- the
    bit-identity reference for the arm-bearing part."""

    def __init__(self, urdf=URDF, margin=MARGIN):
        super().__init__(urdf=urdf, margin=margin)
        for go in self.geom.geometryObjects:
            if hasattr(go.geometry, 'num_points'):
                go.geometry = true_hull(go.geometry)
        self.geom_data = self.geom.createData()

    def check(self, q, only=None):
        self.geom_data = self.geom.createData()
        return super().check(q, only)


def n1(new, old, rng):
    names = arm_joint_names()
    iq = {n: new.model.joints[new.model.getJointId(n)].idx_q for n in names}
    oldw = CrossGantryChecker(urdf=URDF)          # as used G22-G28: one GeometryData, reused
    bad, dss0, werr = 0, float('inf'), []
    for k in range(2000):
        s = {'t1_linear_joint': rng.uniform(0, 1.6), 't2_linear_joint': rng.uniform(0, 1.6)}
        rot0 = k < 500
        s['t1_rotation_joint'] = 0.0 if rot0 else rng.uniform(-np.pi, np.pi)
        s['t2_rotation_joint'] = 0.0 if rot0 else rng.uniform(-np.pi, np.pi)
        for n in names:
            s[n] = rng.uniform(new.model.lowerPositionLimit[iq[n]], new.model.upperPositionLimit[iq[n]])
        q = new.q_from(s)
        da, pa, ds, _ = new.check_split(q)
        do, po = old.check(old.q_from(s))
        if not (da == do and pa == po):
            bad += 1
        werr.append(oldw.check(oldw.q_from(s))[0] - da)
        if rot0:
            dss0 = min(dss0, ds)
    werr = np.array(werr)
    return dict(n=2000, mismatch=bad, d_ss_min_rot0=dss0, warm_err_max=float(werr.max()),
                warm_err_min=float(werr.min()), warm_err_n_gt_0p1mm=int((np.abs(werr) > 1e-4).sum()))


def js_state_at(path, t_query, need):
    """Last sample of every joint at or before t_query (streamed; files are 100s of MB)."""
    st = {}
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            t = float(row[0])
            if t > t_query:
                if all(n in st for n in need):
                    break
                continue
            st[row[1]] = float(row[2])
    return st


def n2(new, old):
    import g22_plan as P
    runs = {'p1_g24/g24b_run.json': '/tmp/g24b_js.csv', 'p1_g26/g26_s1_run.json': '/tmp/g26_js.csv',
            'p1_g26/g26_s2_run.json': '/tmp/g26_js.csv', 'p1_g26/g26_s13_run.json': '/tmp/g26_js.csv'}
    need = arm_joint_names() + ['t1_linear_joint', 't2_linear_joint', 't1_rotation_joint', 't2_rotation_joint']
    rows = []
    for rj, csvp in runs.items():
        d = json.load(open(os.path.join(HERE, '..', rj)))
        for e in d['events']:
            if e['kind'] != 'traverse':
                continue
            g, R = e['gantry'], e['rail']
            st = js_state_at(csvp, R['t_send'], need)
            st = {k: st[k] for k in need}
            s = st[f't{g}_linear_joint']
            v_arc = P.sweep_screen(CrossGantryChecker(urdf=URDF), g, s, R['goal'], st)   # as rail_to_g ran
            v_old = P.sweep_screen(old, g, s, R['goal'], st)                              # OldFresh
            rot = st[f't{g}_rotation_joint']
            v_new = sweep_rot(new, g, (s, rot), (R['goal'], rot), st)
            rows.append(dict(run=rj, gantry=g, frm=s, to=R['goal'], rot=(st['t1_rotation_joint'], st['t2_rotation_joint']),
                             logged_mm=R['sweep_min_mm'], arc_mm=v_arc[1] * 1000, old_mm=v_old[1] * 1000, new_arm_mm=v_new['d_arm'] * 1000,
                             new_ss_mm=v_new['d_ss'] * 1000, old_verdict=v_old[0], new_verdict=v_new['verdict'],
                             bit_identical=v_new['d_arm'] == v_old[1],
                             recon_ok=abs(v_arc[1] * 1000 - R['sweep_min_mm']) <= 1.0))
            print(f"  N2 {rj.split('/')[1][:12]} g{g} {s:.4f}->{R['goal']:.2f}: arsip {R['sweep_min_mm']:.1f}, "
                  f"lama-hangat {v_arc[1]*1000:.3f}, lama-segar {v_old[1]*1000:.3f}, baru-lengan {v_new['d_arm']*1000:.3f} (SS {v_new['d_ss']*1000:.1f}), "
                  f"{v_old[0]}/{v_new['verdict']}, bit {rows[-1]['bit_identical']}", flush=True)
    return rows


def p1(new):
    out = {}
    for dx in (0.0, 0.09, 0.10):
        q = new.q_from(rest_state(0.8 + dx, -np.pi / 2, 0.8, np.pi / 2))
        da, pa, ds, ps = new.check_split(q)
        out[dx] = dict(d_ss=ds, pair_ss=ps, d_arm=da, pair_arm=pa)
    ok = out[0.0]['d_ss'] <= 0 and out[0.09]['d_ss'] <= 0 and out[0.10]['d_ss'] > 0
    return dict(cases=out, ok=ok)


def p2(new):
    """g2 held at rot +90 deg; g1 at the same lin rotates a -> b through -90 deg, both ends >= margin."""
    lin = 0.8
    held = rest_state(lin, 0.0, lin, np.pi / 2)
    ends, side = {}, {}
    for a in range(-90, 1, 1):
        q = new.q_from({**held, 't1_rotation_joint': math.radians(a)})
        ends[a] = new.check(q)[0]
    for a in range(-180, -89, 1):
        q = new.q_from({**held, 't1_rotation_joint': math.radians(a)})
        side[a] = new.check(q)[0]
    a_end = max(a for a in side if side[a] >= MARGIN)          # closest to -90 from below that is clear
    b_end = min(a for a in ends if ends[a] >= MARGIN and a > -90)
    frm, to = (lin, math.radians(a_end)), (lin, math.radians(b_end))
    sw = sweep_rot(new, 1, frm, to, held)
    e1 = sweep_rot(new, 1, frm, frm, held)
    e2 = sweep_rot(new, 1, to, to, held)
    ok = sw['verdict'] == 'COLLIDE' and e1['verdict'] == 'CLEAR' and e2['verdict'] == 'CLEAR'
    return dict(a_deg=a_end, b_deg=b_end, sweep=sw, end_a=e1, end_b=e2, ok=ok)


def p3(new):
    held = rest_state(0.8, 0.0, 0.8, math.radians(60))
    q = new.q_from(held)
    pin.framesForwardKinematics(new.model, new.data, q)
    out = {}
    for tgt_frame in ('t2_mount_plate_left', 't2_mount_plate_right'):
        M = new.data.oMf[new.model.getFrameId(tgt_frame)]
        w = M.translation.copy()
        w[2] -= 0.0          # plate centre (inside the cylinder)
        qa, conv = _ik(new.model, new.data, 't1_a1_', w, q)
        b = new.by_class(qa)
        out[tgt_frame] = dict(target=w.tolist(), ik=conv, LS=b['LS'], LL=b['LL'], SS=b['SS'])
    best = min(out.values(), key=lambda r: r['LS'][0])
    ok = any(r['ik'] and r['LS'][0] <= 0 and r['LS'][1][0].startswith('t1_a1_')
             and r['LS'][1][1].startswith(('t2_rotation_link', 't2_mount_plate')) for r in out.values())
    return dict(cases=out, best=best, ok=ok)


def p4(new, rng):
    from reachability_gng import irm_sweep as IS
    labels = {'arm1': 't1_a1_', 'arm2': 't1_a2_', 'arm3': 't2_a1_', 'arm4': 't2_a2_'}   # irm_sweep.ARMS keys
    worst = 0.0
    for _ in range(200):
        l1, l2 = rng.uniform(0, 1.6, 2)
        r1, r2 = rng.uniform(-np.pi, np.pi, 2)
        q = new.q_from(rest_state(l1, r1, l2, r2))
        pin.framesForwardKinematics(new.model, new.data, q)
        for arm, pfx in labels.items():
            g = 1 if pfx.startswith('t1') else 2
            _, p = IS.base_pose(arm, l1 if g == 1 else l2, r1 if g == 1 else r2)
            worst = max(worst, float(np.abs(new.data.oMf[new.model.getFrameId(f'{pfx}base_link')].translation - p).max()))
    try:
        new.q_from({'t1_rot_joint': 0.3})
        strict = False
    except KeyError:
        strict = True
    try:
        sweep_rot(new, 1, (0.5, 0.0), (0.6, 0.0), {'t1_linear_joint': 0.5, 't2_linear_joint': 0.0})
        refuse = False
    except KeyError:
        refuse = True
    return dict(max_dev_m=worst, strict_name=strict, refuse_no_rot=refuse,
                ok=worst <= 1e-9 and strict and refuse)


def controls():
    new, old = RotCrossChecker(), OldFresh(urdf=URDF)
    print(f'pasangan: induk {old.n_pairs}, baru ber-lengan {new.n_arm} + SS {new.n_ss}')
    rng = np.random.default_rng(29)
    res = {}
    res['P4'] = p4(new, rng)
    print(f"P4 FK base vs irm_sweep.base_pose: maks {res['P4']['max_dev_m']:.2e} m; nama ketat "
          f"{res['P4']['strict_name']}; tolak tanpa rotasi {res['P4']['refuse_no_rot']} -> "
          f"{'LULUS' if res['P4']['ok'] else 'GAGAL'}", flush=True)
    res['N1'] = n1(new, old, rng)
    res['N3_rot0_min_ss'] = res['N1']['d_ss_min_rot0']
    print(f"N1 lama APA ADANYA (hull palsu, data hangat; G22-G28) - baru: maks {res['N1']['warm_err_max']*1000:+.2f} / min "
          f"{res['N1']['warm_err_min']*1000:+.2f} mm, |err| > 0.1 mm pada {res['N1']['warm_err_n_gt_0p1mm']}/2000")
    print(f"N1 {res['N1']['n']} keadaan: beda (vs lama-segar) {res['N1']['mismatch']} -> "
          f"{'LULUS' if res['N1']['mismatch'] == 0 else 'GAGAL'}; N3 SS min rot0 "
          f"{res['N1']['d_ss_min_rot0']*1000:.3f} mm", flush=True)
    res['P1'] = p1(new)
    for dx, c in res['P1']['cases'].items():
        print(f"P1 dx {dx:.2f}: SS {c['d_ss']*1000:7.2f} mm {c['pair_ss']}; lengan {c['d_arm']*1000:7.2f} {c['pair_arm']}")
    print(f"P1 -> {'LULUS' if res['P1']['ok'] else 'GAGAL'}", flush=True)
    res['P2'] = p2(new)
    s = res['P2']
    print(f"P2 g1 rot {s['a_deg']} -> {s['b_deg']} deg (g2 +90, lin sama): sapuan {s['sweep']['verdict']} "
          f"{s['sweep']['d']*1000:.1f} mm {s['sweep']['pair']} di {np.degrees(s['sweep']['at'][1]):.0f} deg "
          f"(lengan {s['sweep']['d_arm']*1000:.1f}, SS {s['sweep']['d_ss']*1000:.1f}); ujung {s['end_a']['verdict']} "
          f"{s['end_a']['d']*1000:.1f} / {s['end_b']['verdict']} {s['end_b']['d']*1000:.1f} -> "
          f"{'LULUS' if s['ok'] else 'GAGAL'}", flush=True)
    res['P3'] = p3(new)
    for f, c in res['P3']['cases'].items():
        print(f"P3 arm_1 -> {f} {np.round(c['target'], 3)} IK {c['ik']}: LS {c['LS'][0]*1000:.1f} {c['LS'][1]}, "
              f"LL {c['LL'][0]*1000:.1f}")
    print(f"P3 -> {'LULUS' if res['P3']['ok'] else 'GAGAL'}", flush=True)
    res['N2'] = n2(new, old)
    ok2 = all(r['bit_identical'] and r['old_verdict'] == r['new_verdict'] for r in res['N2'])
    rec = sum(r['recon_ok'] for r in res['N2'])
    ss_min = min(r['new_ss_mm'] for r in res['N2'])
    print(f"N2 {len(res['N2'])} traverse: rekonstruksi |d| <= 1 mm {rec}/{len(res['N2'])}; bit-identik+verdict "
          f"{'LULUS' if ok2 else 'GAGAL'}; SS min {ss_min:.1f} mm", flush=True)
    res['N3_ok'] = res['N1']['d_ss_min_rot0'] >= 0.380 - 1e-6 and ss_min >= 380.0 - 1e-3
    print(f"N3 -> {'LULUS' if res['N3_ok'] else 'GAGAL'}")
    json.dump(res, open(os.path.join(HERE, 'g29_controls.json'), 'w'), indent=1, default=str)


if __name__ == '__main__':
    if sys.argv[1:] == ['controls']:
        controls()
