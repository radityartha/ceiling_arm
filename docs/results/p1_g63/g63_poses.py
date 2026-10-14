#!/usr/bin/env python3
"""G63 pose generator (§A3 / §A5 T4): candidate board poses per carrying arm, screened OFFLINE, in a fixed order.

Per arm (arm_2 = board A on gantry 1 at rail1, arm_3 = board B on gantry 2 at rail2; every other arm at REST):
  1. sample joint configurations uniformly within the URDF limits (seeded -> reproducible);
  2. board (payload URDF, mount JSON) must be fully visible in >= 1 camera with the INCUMBENT extrinsic:
     all 12 corners >= 40 px inside the 1280x720 frame, viewing angle <= 55 deg, 0.6 <= distance <= DMAX;
     board centre z in [0.80, 1.40] m;
  3. static gravity torque (pinocchio, board mass included): (|tau| + probe offset) / limit <= 0.97 for every joint
     (necessary only -- the probe screens the PLANNED trajectory with full RNEA later);
  4. collision with the shared screens (payload URDF via g63_screens): same-gantry arm >= 50 mm, cross-gantry
     >= 50 mm, frozen env map >= 50 mm; + board vs its OWN arm links (not gripper/fingers) >= 20 mm;
  5. camera rays to the 12 corners not blocked by any robot geometry (5 mm capsule) -- else that camera is dropped.
Selection: farthest-point order over (board centre, 0.5 x board normal), cycling categories both/rgbd/rgbd2 so every
prefix mixes them. Stage cuts = prefixes 3 / 9 / 15 / 21 (§A6.2: start small). HOLDOUT = every 4th of the order.
R0 = the pose seen by BOTH cameras (fallback: most corners) closest to REST in joint space.

Offline, NO ROS. The order is frozen into the output JSON, which is locked with §A; a stage only cuts it.

    python3 g63_poses.py --urdf live.urdf --mount mount.json --out poses.json [--samples 40000]
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time

import numpy as np

from g63_common import (ARM_PREFIX, BOARD_ARM, CAMERAS, IMG_H, IMG_W, K_NOMINAL, board_points, incumbent_T, inv)
import g63_screens
import g63_urdf

REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
EFFORT = {1: 10.0, 2: 14.0, 3: 10.0, 4: 7.0, 5: 7.0, 6: 7.0}          # = reach_dwell_probe JOINT_EFFORT_LIMIT
OFFSET = {1: 6.6, 2: 7.7, 3: 6.6, 4: 1.98, 5: 1.98, 6: 1.98}         # = JOINT_TORQUE_OFFSET_NM
MARGIN_FRAC = 0.97
DMAX = 2.8
Z_RANGE = (0.80, 1.40)
ANGLE_MAX = 55.0
EDGE_PX = 40
CLEAR_ARM = 0.05
CLEAR_ENV = 0.05
CLEAR_SELF = 0.02
STAGES = (3, 9, 15, 21)
# upper_wrist_link is the link right before joint_6: its distance to a board taped below it depends on the MOUNT
# (rotationally ~constant about j6), not on the pose -> checked once per mount (mount_check), not per pose.
OWN_LINKS = ('base_link', 'shoulder_link', 'arm_link', 'forearm_link', 'lower_wrist_link')
MOUNT_LINKS = ('upper_wrist_link', 'gripper_base_link')


def gantry(arm):
    return 't1' if ARM_PREFIX[arm].startswith('t1') else 't2'


class Cell:
    def __init__(self, urdf, rail1, rail2):
        import pinocchio as pin
        self.pin = pin
        self.m = pin.buildModelFromUrdf(urdf)
        self.d = self.m.createData()
        self.base = {'t1_linear_joint': rail1, 't2_linear_joint': rail2,
                     't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
        for a, p in ARM_PREFIX.items():
            for i, v in enumerate(REST):
                self.base[f'{p}joint_{i + 1}'] = v
        self.same = {g: g63_screens.interarm_collision.InterArmChecker(gantry=f'gantry_{g[-1]}') for g in ('t1', 't2')}
        self.cross = g63_screens.interarm_collision.CrossGantryChecker()
        self.env = g63_screens.env_collision.EnvChecker()
        g63_screens.verify()
        self.K = K_NOMINAL
        self.Twc = {c: incumbent_T(c) for c in CAMERAS}

    def q(self, joints):
        q = self.pin.neutral(self.m)
        for n, v in joints.items():
            q[self.m.joints[self.m.getJointId(n)].idx_q] = v
        return q

    def joints(self, arm, q6):
        j = dict(self.base)
        for i, v in enumerate(q6):
            j[f'{ARM_PREFIX[arm]}joint_{i + 1}'] = float(v)
        return j


def board_world(cell, arm, q):
    pin = cell.pin
    pin.framesForwardKinematics(cell.m, cell.d, q)
    M = cell.d.oMf[cell.m.getFrameId(ARM_PREFIX[arm] + 'g63_board')]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = M.rotation, M.translation
    return T


def visibility(cell, T_wb, P):
    """{cam: dict(dist, angle, uv)} for cameras that see all corners."""
    out = {}
    Pw = P @ T_wb[:3, :3].T + T_wb[:3, 3]
    ctr = Pw.mean(0)
    nrm = -T_wb[:3, 2]                                    # printed face normal (board z points INTO the board)
    for c in CAMERAS:
        Tcw = inv(cell.Twc[c])
        pc = Pw @ Tcw[:3, :3].T + Tcw[:3, 3]
        if (pc[:, 2] < 0.3).any():
            continue
        to_cam = cell.Twc[c][:3, 3] - ctr
        dist = float(np.linalg.norm(to_cam))
        ang = math.degrees(math.acos(np.clip(np.dot(nrm, to_cam / dist), -1, 1)))
        if ang > ANGLE_MAX or dist > DMAX or dist < 0.6:
            continue
        uv = pc[:, :2] / pc[:, 2:3] @ cell.K[c][:2, :2].T + cell.K[c][:2, 2]
        if (uv[:, 0] < EDGE_PX).any() or (uv[:, 0] > IMG_W - EDGE_PX).any() or (uv[:, 1] < EDGE_PX).any() or \
                (uv[:, 1] > IMG_H - EDGE_PX).any():
            continue
        out[c] = dict(dist=dist, angle=ang)
    return out, ctr, nrm, Pw


def torque_ok(cell, arm, q):
    tau = cell.pin.computeGeneralizedGravity(cell.m, cell.d, q)
    worst = 0.0
    for k in range(1, 7):
        iv = cell.m.joints[cell.m.getJointId(f'{ARM_PREFIX[arm]}joint_{k}')].idx_v
        worst = max(worst, (abs(tau[iv]) + OFFSET[k]) / EFFORT[k])
    return worst <= MARGIN_FRAC, worst


def mount_check(cell, arm):
    """Board box vs the links next to it, over joint_6 in [-pi, pi] at REST: min distance (box already +10 mm)."""
    import coal
    pre = ARM_PREFIX[arm]
    names = cell.env.names
    ib = names.index(pre + 'g63_board_0')
    res = {}
    for ln in MOUNT_LINKS:
        i = names.index(f'{pre}{ln}_0')
        dmin = float('inf')
        for j6 in np.linspace(-np.pi, np.pi, 25):
            q6 = list(REST[:5]) + [j6]
            oMg = cell.env.place(cell.env.q_from(cell.joints(arm, q6)))
            r = coal.DistanceResult()
            d = coal.distance(cell.env.geom.geometryObjects[ib].geometry,
                              coal.Transform3s(oMg[ib].rotation, oMg[ib].translation),
                              cell.env.geom.geometryObjects[i].geometry,
                              coal.Transform3s(oMg[i].rotation, oMg[i].translation), coal.DistanceRequest(), r)
            dmin = min(dmin, float(d))
        res[ln] = round(dmin, 4)
    return res


def collisions(cell, arm, joints):
    g = gantry(arm)
    pre = ARM_PREFIX[arm]
    q = cell.q(joints)
    d_same, p_same = cell.same[g].check(q, only=pre)
    d_cross, p_cross = cell.cross.check(q, only=pre)
    d_env, p_env = cell.env.check(cell.env.q_from(joints), only=[pre])
    # board vs own arm links (MoveIt also checks this with the attached object; done here so the list is clean)
    oMg = cell.env.place(cell.env.q_from(joints))
    names = cell.env.names
    import coal
    ib = names.index(pre + 'g63_board_0')
    gb = cell.env.geom.geometryObjects[ib]
    Mb = oMg[ib]
    d_self, p_self = float('inf'), None
    for ln in OWN_LINKS:
        i = names.index(f'{pre}{ln}_0')
        Mi = oMg[i]
        r = coal.DistanceResult()
        d = coal.distance(gb.geometry, coal.Transform3s(Mb.rotation, Mb.translation),
                          cell.env.geom.geometryObjects[i].geometry, coal.Transform3s(Mi.rotation, Mi.translation),
                          coal.DistanceRequest(), r)
        if d < d_self:
            d_self, p_self = d, ln
    ok = d_same >= CLEAR_ARM and d_cross >= CLEAR_ARM and d_env >= CLEAR_ENV and d_self >= CLEAR_SELF
    return ok, dict(same_m=float(d_same), cross_m=float(d_cross), env_m=float(d_env), self_m=float(d_self),
                    worst=dict(same=p_same, cross=p_cross, env=p_env, self=p_self))


def occluded(cell, arm, joints, cam, Pw):
    """True if any corner's camera ray passes within 5 mm of robot geometry (the board itself excluded)."""
    import coal
    oMg = cell.env.place(cell.env.q_from(joints))
    own_board = ARM_PREFIX[arm] + 'g63_board_0'
    o = cell.Twc[cam][:3, 3]
    for X in Pw:
        v = X - o
        L = float(np.linalg.norm(v)) - 0.01                   # stop 1 cm before the corner (paper surface)
        z = v / np.linalg.norm(v)
        x = np.cross(z, [0, 0, 1.0]) if abs(z[2]) < 0.9 else np.cross(z, [1.0, 0, 0])
        x /= np.linalg.norm(x)
        R = np.stack([x, np.cross(z, x), z], 1)
        cap = coal.Capsule(0.005, L)
        Tc = coal.Transform3s(R, o + z * L / 2)
        for i, g in enumerate(cell.env.geom.geometryObjects):
            if g.name == own_board:
                continue
            M = oMg[i]
            r = coal.CollisionResult()
            if coal.collide(cap, Tc, g.geometry, coal.Transform3s(M.rotation, M.translation),
                            coal.CollisionRequest(), r):
                return g.name
    return None


def generate(cell, arm, board, n_samples, rng, log):
    P = np.array([board_points(board)[i] for i in range(12)])
    lo = np.array([cell.m.lowerPositionLimit[cell.m.joints[cell.m.getJointId(f'{ARM_PREFIX[arm]}joint_{k}')].idx_q]
                   for k in range(1, 7)])
    hi = np.array([cell.m.upperPositionLimit[cell.m.joints[cell.m.getJointId(f'{ARM_PREFIX[arm]}joint_{k}')].idx_q]
                   for k in range(1, 7)])
    cand = []
    cnt = dict(sampled=0, visible=0, z=0, torque=0, collision=0, occlusion=0)
    t0 = time.time()
    for _ in range(n_samples):
        q6 = rng.uniform(lo, hi)
        cnt['sampled'] += 1
        jn = cell.joints(arm, q6)
        q = cell.q(jn)
        T_wb = board_world(cell, arm, q)
        vis, ctr, nrm, Pw = visibility(cell, T_wb, P)
        if not vis:
            continue
        cnt['visible'] += 1
        if not Z_RANGE[0] <= ctr[2] <= Z_RANGE[1]:
            continue
        cnt['z'] += 1
        tok, tfrac = torque_ok(cell, arm, q)
        if not tok:
            continue
        cnt['torque'] += 1
        cand.append(dict(q6=q6, vis=vis, ctr=ctr, nrm=nrm, Pw=Pw, tfrac=tfrac, T_wb=T_wb))
    log(f'{arm}: {cnt} ({time.time() - t0:.0f} s) -> tabrakan/oklusi pada kandidat')
    good = []
    for c in cand:
        jn = cell.joints(arm, c['q6'])
        ok, col = collisions(cell, arm, jn)
        if not ok:
            continue
        cnt['collision'] += 1
        for cam in list(c['vis']):
            hit = occluded(cell, arm, jn, cam, c['Pw'])
            if hit:
                c['vis'].pop(cam)
                c.setdefault('occluded_by', {})[cam] = hit
        if not c['vis']:
            continue
        cnt['occlusion'] += 1
        c['col'] = col
        good.append(c)
    log(f'{arm}: lolos semua {len(good)}; hitungan {cnt}')
    return good, cnt


def category(c):
    return 'both' if len(c['vis']) == 2 else next(iter(c['vis']))


def order_fps(cands, n, seed_idx):
    feats = np.array([np.r_[c['ctr'], 0.5 * c['nrm']] for c in cands])
    chosen = [seed_idx]
    cats = ['both', 'rgbd', 'rgbd2']
    k = 0
    while len(chosen) < min(n, len(cands)):
        dmin = np.min(np.linalg.norm(feats[:, None, :] - feats[chosen][None], axis=2), axis=1)
        dmin[chosen] = -1
        pick = None
        for t in range(3):
            want = cats[(k + t) % 3]
            idx = [i for i in range(len(cands)) if category(cands[i]) == want and dmin[i] > 0]
            if idx:
                pick = max(idx, key=lambda i: dmin[i])
                break
        if pick is None:
            break
        chosen.append(pick)
        k += 1
    return chosen


def tool_pose(cell, arm, q6):
    from scipy.spatial.transform import Rotation
    q = cell.q(cell.joints(arm, q6))
    cell.pin.framesForwardKinematics(cell.m, cell.d, q)
    M = cell.d.oMf[cell.m.getFrameId(ARM_PREFIX[arm] + 'tool_frame')]
    return np.array(M.translation).tolist(), Rotation.from_matrix(M.rotation).as_quat().tolist()   # x y z w


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True, help='live URDF (tanpa papan)')
    ap.add_argument('--mount', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--samples', type=int, default=40000)
    ap.add_argument('--rail1', type=float, default=1.50)
    ap.add_argument('--rail2', type=float, default=0.00)
    ap.add_argument('--seed', type=int, default=63)
    a = ap.parse_args()
    mount = json.load(open(a.mount))
    payload = os.path.join(tempfile.mkdtemp(prefix='g63_'), 'g63_payload.urdf')
    g63_urdf.build(a.urdf, mount, payload)
    g63_screens.install(payload)
    cell = Cell(payload, a.rail1, a.rail2)
    logs = []

    def log(s):
        print(s, flush=True)
        logs.append(s)
    rng = np.random.default_rng(a.seed)
    out = dict(created='g63_poses.py', urdf=os.path.abspath(a.urdf), mount=mount,
               mount_guess=any(v.get('guess') for k, v in mount.items() if not k.startswith('_')),
               rail1=a.rail1, rail2=a.rail2, seed=a.seed, samples=a.samples, stages=STAGES,
               incumbent_used_for_visibility=True, arms={})
    for board, arm in sorted(BOARD_ARM.items()):
        mc = mount_check(cell, arm)
        log(f'{arm}: cek dudukan (kotak papan +10 mm vs link tetangga, j6 penuh): {mc} m'
            f'{"  <-- upper_wrist NEGATIF = papan menembus pergelangan: dudukan mustahil" if mc["upper_wrist_link"] < 0 else ""}'
            ' (gripper_base = touch link, boleh bersentuhan)')
        good, cnt = generate(cell, arm, board, a.samples, rng, log)
        if not good:
            log(f'!!! {arm}: TIDAK ADA kandidat lolos')
            out['arms'][arm] = dict(board=board, counts=cnt, mount_check=mc, poses=[])
            continue
        qrest = np.array(REST)
        both = [i for i, c in enumerate(good) if len(c['vis']) == 2]
        pool = both or list(range(len(good)))
        r0 = min(pool, key=lambda i: (np.linalg.norm(good[i]['q6'] - qrest), good[i]['tfrac']))
        rest_idx = [i for i in range(len(good)) if i != r0]
        sub = [good[i] for i in rest_idx]
        seed = max(range(len(sub)), key=lambda i: np.linalg.norm(sub[i]['ctr'] - good[r0]['ctr']))
        order = order_fps(sub, STAGES[-1], seed)

        def rec(c, role, stage, idx):
            pos, quat = tool_pose(cell, arm, c['q6'])
            return dict(id=f'{board}-{idx:02d}' if role != 'R0' else f'{board}-R0', board=board, arm=arm, role=role,
                        stage=stage, goal_joints={f'{ARM_PREFIX[arm]}joint_{k + 1}': round(float(v), 6)
                                                  for k, v in enumerate(c['q6'])},
                        tool_xyz=[round(v, 5) for v in pos], tool_quat_xyzw=[round(v, 6) for v in quat],
                        board_centre=[round(float(v), 4) for v in c['ctr']],
                        board_normal=[round(float(v), 4) for v in c['nrm']],
                        cams={k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in c['vis'].items()},
                        occluded_by=c.get('occluded_by', {}), static_torque_frac=round(c['tfrac'], 4),
                        clearance=c['col'])
        poses = [rec(good[r0], 'R0', 'all', 0)]
        for n, i in enumerate(order, start=1):
            stage = next(k for k, cut in enumerate(STAGES) if n <= cut)
            role = 'HOLDOUT' if n % 4 == 0 else 'FIT'
            poses.append(rec(sub[i], role, ['P0', 'T1', 'T2', 'T3'][stage], n))
        out['arms'][arm] = dict(board=board, counts=cnt, mount_check=mc, n_candidates=len(good), poses=poses)
        summ = {}
        for p in poses[1:]:
            k = '+'.join(sorted(p['cams']))
            summ[(p['role'], k)] = summ.get((p['role'], k), 0) + 1
        log(f'{arm}: R0 kamera {sorted(poses[0]["cams"])}; urutan {len(poses) - 1}; {summ}')
    out['log'] = logs
    json.dump(out, open(a.out, 'w'), indent=1)
    print(f'tulis {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
