"""G28 A1: V28 plan-only screen with SAVED trajectories. Zero motion. docs/p1_g28_hw.md.

Per tuple of g27_oracle4.json['V28'] (61, file order): ALWAYS 3 samples, each planned from
the same PLACED start -- tuple gantry rail at the tuple rail, the other gantry at R1 (read
rail), rotations 0, all four arms REST -- through reach_dwell_probe._plan_and_screen with the
exact arguments of g22 sched_screen.walk. _plan_and_screen is not touched: _violates_tuck and
predict_peak_torque are wrapped pass-through in the probe module namespace, so the trajectory
of EVERY plan MoveIt returns (including the ones then refused) is kept. Per waypoint: full
RNEA and static gravity on the probe's own model (neutral + arm joints).

    python3 v28_screen.py --dry                       # KD1-KD3 + K-RNEA off, no ROS
    python3 v28_screen.py --r1 0.00 0.00              # V28 on the running stack
    python3 v28_screen.py --set smoke --r1 0 0 --out smoke_mock_plans.jsonl   # A5, dev tuples
"""
import argparse
import hashlib
import json
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g22'))
import g22_plan as P  # noqa: E402

V28_JSON = os.path.join(HERE, '..', 'p1_g27', 'g27_oracle4.json')
URDF_CACHE = '/tmp/reach_dwell_live.urdf'
URDF_SHA = 'f02e7c532cdbee70ecc3b972a2343e20b60f81622f6a8bd78e9211f8030e7711'
TAU_MAX = 12.0
GANTRY = {'arm_1': 1, 'arm_2': 1, 'arm_3': 2, 'arm_4': 2}
# A5: development tuples only (g27_table.json), never V28.
SMOKE = [dict(key=[2033, 1, 15, 0], arm='arm_1', xyz=[1.2857, 0.3882, 1.4], rail=0.75, ok4=False),
         dict(key=[83, 1, 11, 1], arm='arm_2', xyz=[0.0, 0.3176, 1.4], rail=0.55, ok4=False),
         dict(key=[1600, 1, 11, 0], arm='arm_1', xyz=[1.0, 0.3882, 1.32], rail=0.55, ok4=True)]


def load_set(name):
    if name == 'smoke':
        return SMOKE
    v = json.load(open(V28_JSON))['V28']
    z = [round(t['xyz'][2], 2) for t in v]
    ok = (len(v) == 61 and z.count(1.4) == 41 and z.count(1.32) == 20
          and all(t['ok4'] == (round(t['xyz'][2], 2) == 1.32) for t in v)
          and all(0.0 <= t['rail'] <= P.RAIL_MAX for t in v))
    if not ok:
        raise SystemExit('KD1 GAGAL: V28 bukan 61 = 41 z1.40 (tolak) + 20 z1.32 (terima), rel dalam [0, 1.6]')
    print('KD1 lulus: 61 tuple = 41 z1.40 tolak + 20 z1.32 terima; rel dalam [0, 1.6]')
    return v


def start_joints(t, r1):
    g = GANTRY[t['arm']]
    rail = {1: r1[0], 2: r1[1]}
    rail[g] = t['rail']
    s = {'t1_linear_joint': rail[1], 't2_linear_joint': rail[2],
         't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
    for p in P.PREFIX.values():
        s.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
    return s


def rnea_points(m, d, jt, arm, prefix):
    """Per waypoint |tau| (full RNEA) and static |gravity|, the probe's predict_peak_torque
    model step for step: pin.neutral + this arm's joints, v/a from the point or zero."""
    import pinocchio as pin
    names = [n for n in jt.joint_names if n.startswith(prefix[arm])]
    idx = {n: (m.joints[m.getJointId(n)].idx_q, m.joints[m.getJointId(n)].idx_v) for n in names}
    col = {n: jt.joint_names.index(n) for n in names}
    tau, stat = [], []
    for pt in jt.points:
        q = pin.neutral(m)
        v = np.zeros(m.nv)
        a = np.zeros(m.nv)
        for n in names:
            iq, iv = idx[n]
            k = col[n]
            q[iq] = pt.positions[k]
            if pt.velocities:
                v[iv] = pt.velocities[k]
            if pt.accelerations:
                a[iv] = pt.accelerations[k]
        t = pin.rnea(m, d, q, v, a)
        g = pin.rnea(m, d, q, np.zeros(m.nv), np.zeros(m.nv))
        tau.append([abs(t[idx[n][1]]) for n in names])
        stat.append([abs(g[idx[n][1]]) for n in names])
    return names, np.array(tau), np.array(stat)


def kd2():
    if not os.path.exists(URDF_CACHE):
        return 'tidak ada'
    return hashlib.sha256(open(URDF_CACHE, 'rb').read()).hexdigest()


def krnea_off(RDP, n=200):
    """K-RNEA off: per-point max == predict_peak_torque, bit-identical, on random trajectories."""
    import pinocchio as pin
    m = pin.buildModelFromUrdf(URDF_CACHE)
    d = m.createData()
    rng = np.random.default_rng(28)
    node = SimpleNamespace()
    worst = 0.0
    for k in range(n):
        arm = list(P.PREFIX)[k % 4]
        names = [f'{RDP.JOINT_PREFIX[arm]}joint_{i}' for i in range(1, 7)]
        other = 't9_dummy' if k % 2 else f'{RDP.JOINT_PREFIX[list(P.PREFIX)[(k + 1) % 4]]}joint_1'
        jn = names + [other]                         # a foreign joint in the list, as MoveIt may send
        lo = [m.lowerPositionLimit[m.joints[m.getJointId(x)].idx_q] for x in names]
        hi = [m.upperPositionLimit[m.joints[m.getJointId(x)].idx_q] for x in names]
        pts = []
        for j in range(int(rng.integers(2, 60))):
            q = list(rng.uniform(np.maximum(lo, -3.0), np.minimum(hi, 3.0))) + [0.1]
            empty = (k % 7 == 0)
            pts.append(SimpleNamespace(positions=q,
                                       velocities=[] if empty else list(rng.normal(0, 0.5, 7)),
                                       accelerations=[] if empty else list(rng.normal(0, 1.0, 7))))
        jt = SimpleNamespace(joint_names=jn, points=pts)
        peak = RDP.predict_peak_torque(SimpleNamespace(joint_trajectory=jt), arm, node)
        _, tau, _ = rnea_points(m, d, jt, arm, RDP.JOINT_PREFIX)
        for c, x in enumerate(names):
            if tau[:, c].max() != peak[x]:
                worst = max(worst, abs(tau[:, c].max() - peak[x]))
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--set', choices=('V28', 'smoke'), default='V28')
    ap.add_argument('--r1', type=float, nargs=2, help='R1: rel terbaca g1 g2, grid 0.05 m')
    ap.add_argument('--samples', type=int, default=3)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = os.path.join(HERE, a.out or ('v28_plans.jsonl' if a.set == 'V28' else 'smoke_mock_plans.jsonl'))

    tuples = load_set(a.set)
    sha = kd2()
    if sha != URDF_SHA:
        raise SystemExit(f'KD2 GAGAL: {URDF_CACHE} sha {sha[:12]} != {URDF_SHA[:12]} (cache basi?)')
    print(f'KD2 lulus: {URDF_CACHE} sha {sha[:12]}')
    import reach_dwell_probe as RDP
    if RDP.JOINT_TORQUE_OFFSET_NM[2] != 7.7:
        raise SystemExit('KD3 GAGAL: offset joint_2 != 7.7')
    print('KD3 lulus: offset joint_2 = 7.7')

    if a.dry:
        w = krnea_off(RDP)
        print(f'K-RNEA off: 200 lintasan sintetis, maks |beda| = {w!r} -> {"LULUS (bit-identik)" if w == 0.0 else "GAGAL"}')
        r1 = a.r1 or (0.0, 0.0)
        print(f'DRY (R1 {"diberikan" if a.r1 else "belum diberikan -> contoh 0/0"} = {r1}): '
              f'{len(tuples)} tuple x {a.samples} sampel -> {out}')
        for i, t in enumerate(tuples):
            s = start_joints(t, r1)
            print(f'  [{i:2d}] {t["arm"]} -> ({t["xyz"][0]:.4f}, {t["xyz"][1]:.4f}, {t["xyz"][2]:.2f}) '
                  f'rel t1 {s["t1_linear_joint"]:.3f} t2 {s["t2_linear_joint"]:.3f}  '
                  f'oracle4 {"terima" if t["ok4"] else "tolak"}  others={[x for x in P.PREFIX if x != t["arm"]]}')
        sys.exit(0 if w == 0.0 else 1)

    if a.r1 is None:
        raise SystemExit('--r1 wajib (rel terbaca bring-up, g26 A1)')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient

    cap = {}
    _tuck, _ppt = RDP._violates_tuck, RDP.predict_peak_torque

    def tuck_hook(traj, arm, *args, **kw):
        cap['traj'] = traj
        return _tuck(traj, arm, *args, **kw)

    def ppt_hook(traj, arm, node):
        r = _ppt(traj, arm, node)
        cap['peak'] = r
        return r
    RDP._violates_tuck, RDP.predict_peak_torque = tuck_hook, ppt_hook

    def pose(xyz):
        p = PoseStamped()
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in xyz)
        p.pose.orientation.x, p.pose.orientation.w = 1.0, 0.0
        return p

    done = set()
    if os.path.exists(out):
        for ln in open(out):
            r = json.loads(ln)
            done.add((r['i'], r['sample']))
    rclpy.init()
    node = RDP.Probe('arm_1', 0.0, TAU_MAX, False, arms=list(P.PREFIX))
    node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
    if not node._plan_ac.wait_for_server(timeout_sec=15.0):
        raise RuntimeError('move_group tidak ada')
    import pinocchio as pin
    path = RDP._urdf_path(node)
    if hashlib.sha256(open(path, 'rb').read()).hexdigest() != URDF_SHA:
        raise SystemExit('KD2 GAGAL sesudah _urdf_path')
    m = pin.buildModelFromUrdf(path)
    d = m.createData()
    t_all = time.time()
    try:
        with open(out, 'a') as f:
            for i, t in enumerate(tuples):
                for s in range(a.samples):
                    if (i, s) in done:
                        continue
                    sj = start_joints(t, a.r1)
                    others = [x for x in P.PREFIX if x != t['arm']]
                    cap.clear()
                    t0 = time.time()
                    v, _ = RDP._plan_and_screen(t['arm'], pose(t['xyz']), node, 0.002, 2.0, 0.15,
                                                15.0, TAU_MAX, others, start_joints=dict(sj))
                    wall = time.time() - t0
                    rec = dict(i=i, sample=s, key=t['key'], arm=t['arm'], xyz=t['xyz'], rail=t['rail'],
                               z=round(t['xyz'][2], 2), ok4=t['ok4'], verdict=v, wall=wall,
                               r1=a.r1, start=sj, traj=None)
                    tr = cap.get('traj')
                    if tr is not None:
                        jt = tr.joint_trajectory
                        names, tau, stat = rnea_points(m, d, jt, t['arm'], RDP.JOINT_PREFIX)
                        k2 = int(np.argmax(tau[:, 1]))
                        rec.update(traj=dict(
                            joint_names=list(jt.joint_names),
                            t=[p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points],
                            pos=[list(p.positions) for p in jt.points],
                            vel=[list(p.velocities) for p in jt.points],
                            acc=[list(p.accelerations) for p in jt.points]),
                            arm_joints=names, tau=tau.tolist(), stat=stat.tolist(),
                            N=len(jt.points), k_peak2=k2, rnea2=float(tau[k2, 1]),
                            stat2_at_peak=float(stat[k2, 1]), stat2_end=float(stat[-1, 1]))
                        pk = cap.get('peak')
                        if pk is not None:
                            err = max(abs(tau[:, c].max() - pk[n]) for c, n in enumerate(names))
                            rec.update(peak_probe=pk, krnea_err=err)
                            if err > 1e-9:
                                f.write(json.dumps(rec) + '\n')
                                raise SystemExit(f'K-RNEA on GAGAL: [{i}] s{s} beda {err}')
                    f.write(json.dumps(rec) + '\n')
                    f.flush()
                    extra = (f' RNEA j2 {rec["rnea2"]:.2f} @ {rec["k_peak2"]}/{rec["N"] - 1}'
                             f' statis-akhir {rec["stat2_end"]:.2f}') if rec['traj'] else ''
                    print(f'[{i:2d}] s{s + 1} {t["arm"]} z{rec["z"]:.2f} @{t["rail"]:.2f} '
                          f'oracle4 {"T" if t["ok4"] else "F"}: {v} ({wall:.1f} s){extra}', flush=True)
    finally:
        print(f'wall total sesi ini {time.time() - t_all:.0f} s', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
