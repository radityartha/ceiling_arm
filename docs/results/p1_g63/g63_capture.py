#!/usr/bin/env python3
"""G63 capture (§A5 T5): one board pose, both cameras, NO motion commanded by this tool.

  capture: wait until the carrying arm is STILL (all 6 joints std < 2e-4 rad over 1 s), then take N fresh colour
           frames per camera (unique stamps; duplicates are not counted), detect boards (prior = live camera TF +
           live finger-link TF + mount JSON), median per corner (g63_detect.median_over_frames). Records camera_info,
           the TF world->finger link (median over the frames), all joints by name, one raw PNG per camera.
           Appends ONE line to <session>/poses.jsonl. Prints a short verdict per camera.
  merge:   poses.jsonl + MEASURED square sizes -> solver dataset JSON (g63_solve.py format).

Run alongside (separate terminals): rec_health.py for both cameras, the torque monitor and the joint recorder.

    g63_capture.py capture --session DIR --pose-id A-03 --board A --role FIT --stage P0 --mount mount.json [--frames 30]
    g63_capture.py merge --session DIR --square-mm A=50.0 B=50.0 --out dataset.json
"""
import argparse
import json
import os
import sys
import time

import numpy as np

from g63_common import ARM_PREFIX, BOARD_ARM, CAMERAS, T_from_xyzrpy
import g63_detect

STILL_STD_RAD = 2e-4
# G63b §A6.1 (operator 2026-10-15): joints listed here (name suffix -> max peak-to-peak rad in the window) are judged by
# peak-to-peak instead of std, and left out of the drift check. EMPTY = original G63 behaviour (every joint std).
STILL_EXEMPT_PP = {}


def still_check(win, names):
    """-> (ok, std of strict joints, {exempt joint: peak-to-peak}). win = list of joint dicts over the window."""
    strict = [n for n in names if not any(n.endswith(s) for s in STILL_EXEMPT_PP)]
    std = max(float(np.std([j[n] for j in win])) for n in strict)
    pp = {n: float(np.ptp([j[n] for j in win])) for n in names if n not in strict}
    ok = std < STILL_STD_RAD and all(v <= STILL_EXEMPT_PP[next(s for s in STILL_EXEMPT_PP if n.endswith(s))]
                                     for n, v in pp.items())
    return ok, std, pp


def _img_to_gray(msg):
    import cv2
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
        msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY), a[:, :, ::-1].copy()
    if msg.encoding == 'bgr8':
        return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), a.copy()
    raise ValueError(f'encoding {msg.encoding} tidak didukung')


def _T(tf):
    from scipy.spatial.transform import Rotation
    t, q = tf.transform.translation, tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def capture(a):
    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from tf2_ros import Buffer, TransformListener

    mount = json.load(open(a.mount))
    boards = a.board
    rclpy.init()
    node = rclpy.create_node('g63_capture')
    buf = Buffer()
    TransformListener(buf, node)
    info, frames, joints, jhist = {}, {c: [] for c in CAMERAS}, {}, []
    seen = {c: set() for c in CAMERAS}

    def on_img(cam):
        def cb(m):
            st = (m.header.stamp.sec, m.header.stamp.nanosec)
            if st in seen[cam] or len(frames[cam]) >= a.frames or not state['go']:
                return
            seen[cam].add(st)
            frames[cam].append(m)
        return cb

    def on_js(m):
        for n, p in zip(m.name, m.position):
            joints[n] = p
        jhist.append((time.time(), dict(joints)))

    state = {'go': False}
    for c in CAMERAS:
        node.create_subscription(Image, f'/{c}/{c}/color/image_raw', on_img(c), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{c}/{c}/color/camera_info', lambda m, c=c: info.__setitem__(c, m),
                                 qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', on_js, 10)

    arms = sorted({BOARD_ARM[b] for b in boards})
    names = [f'{ARM_PREFIX[arm]}joint_{k}' for arm in arms for k in range(1, 7)]
    # 1. stillness: 1 s window, every carrying-arm joint std < STILL_STD_RAD
    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        win = [j for t, j in jhist if now - t <= 1.0 and all(n in j for n in names)]
        if now - t0 > 1.2 and len(win) >= 10:
            ok_still, std, pp_ex = still_check(win, names)
            if ok_still:
                break
        if now - t0 > 20:
            print('GAGAL: lengan tidak diam / joint_states tidak lengkap dalam 20 s -- tidak menangkap', flush=True)
            return 2
    still_std = std
    j_before = {n: float(np.median([j[n] for j in win])) for n in names}
    # 2. frames
    state['go'] = True
    t1 = time.time()
    while any(len(frames[c]) < a.frames for c in CAMERAS) and time.time() - t1 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    state['go'] = False
    j_after = {n: joints.get(n, float('nan')) for n in names}
    strict = [n for n in names if not any(n.endswith(s) for s in STILL_EXEMPT_PP)]
    moved = max(abs(j_after[n] - j_before[n]) for n in strict)
    rec = dict(id=a.pose_id, boards=boards, role=a.role, stage=a.stage, t_wall=time.time(), still_std_rad=still_std,
               joint_drift_during_capture_rad=moved, still_exempt_pp_rad=pp_ex, joints={n: float(v) for n, v in joints.items()}, cams={})
    os.makedirs(os.path.join(a.session, 'raw'), exist_ok=True)
    for c in CAMERAS:
        if c not in info or not frames[c]:
            rec['cams'][c] = dict(status='TIDAK ADA FRAME/INFO', n_frames=len(frames[c]))
            continue
        ci = info[c]
        K = np.array(ci.k).reshape(3, 3)
        dist = list(ci.d)
        cam_frame = ci.header.frame_id or f'{c}_camera_optical'
        # priors: live camera TF (= extrinsic in use) x live finger TF x mount guess
        priors, T_wl = {}, {}
        try:
            Twc = _T(buf.lookup_transform('world', f'{c}_camera_optical', Time(), timeout=rclpy.duration.Duration(
                seconds=2.0)))
        except Exception as e:                                    # noqa: BLE001
            rec['cams'][c] = dict(status=f'TF kamera tidak ada: {e}')
            continue
        for b in boards:
            link = ARM_PREFIX[BOARD_ARM[b]] + mount[b]['link']
            Ts = []
            for m in frames[c]:
                try:
                    Ts.append(_T(buf.lookup_transform('world', link, Time.from_msg(m.header.stamp),
                                                      timeout=rclpy.duration.Duration(seconds=0.5))))
                except Exception:                                 # noqa: BLE001
                    pass
            if not Ts:
                continue
            Tm = Ts[len(Ts) // 2]
            spread = max(float(np.linalg.norm(T[:3, 3] - Tm[:3, 3])) for T in Ts)
            T_wl[b] = dict(T=Tm.tolist(), n_tf=len(Ts), spread_m=spread)
            T_lb = T_from_xyzrpy(*mount[b]['xyz'], *mount[b]['rpy'])
            priors[b] = g63_detect.predict_uv(b, Tm, T_lb, Twc, K, dist, mount[b].get('square_m'))
        dets = {b: [] for b in boards}
        for m in frames[c]:
            gray, _ = _img_to_gray(m)
            r = g63_detect.detect(gray, priors)
            for b in boards:
                if r[b]['ids']:
                    dets[b].append(r[b])
                else:
                    dets[b].append(dict(ids=[], uv=[], method='none', note=r[b]['note']))
        mid = frames[c][len(frames[c]) // 2]
        _, bgr = _img_to_gray(mid)
        png = os.path.join(a.session, 'raw', f'{a.pose_id}_{c}.png')
        cv2.imwrite(png, bgr)
        if a.save_frames:   # HW 10-15: all frames, for offline re-detection (detector amendment needs the same 30 frames)
            fdir = os.path.join(a.session, 'frames', a.pose_id, c)
            os.makedirs(fdir, exist_ok=True)
            for i, m in enumerate(frames[c]):
                cv2.imwrite(os.path.join(fdir, f'{i:02d}_{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}.png'),
                            _img_to_gray(m)[1])
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in frames[c]]
        rec['cams'][c] = dict(status='OK', n_frames=len(frames[c]), stamp_span_s=max(stamps) - min(stamps),
                              K=K.tolist(), dist=dist, frame_id=cam_frame, width=ci.width, height=ci.height,
                              T_wc_live=Twc.tolist(), raw_png=os.path.relpath(png, a.session),
                              T_wl=T_wl, boards={b: dict(g63_detect.median_over_frames(dets[b]),
                                                         notes=sorted({d.get('note', '') for d in dets[b]})[:3])
                                                 for b in boards})
    with open(os.path.join(a.session, 'poses.jsonl'), 'a') as f:
        f.write(json.dumps(rec) + '\n')
    print(f'{a.pose_id}: diam std {still_std:.2e} rad, geser selama tangkap {moved:.2e} rad', flush=True)
    for c, v in rec['cams'].items():
        if v.get('status') != 'OK':
            print(f'  {c}: {v["status"]}')
            continue
        for b, d in v['boards'].items():
            print(f'  {c} papan {b}: {len(d["ids"])} sudut, frame {d["n_frames"]}, metode {d["methods"]}, '
                  f'std maks {max(d["std_px"]) if d["std_px"] else float("nan"):.2f} px')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if moved < 1e-3 else 3


def merge(a):
    sq = {kv.split('=')[0]: float(kv.split('=')[1]) / 1000 for kv in a.square_mm}
    zone = {}
    if a.poses:
        for arm in json.load(open(a.poses))['arms'].values():
            zone.update({p['id']: p['zone'] for p in arm['poses']})
    cams, poses = {}, []
    for line in open(os.path.join(a.session, 'poses.jsonl')):
        r = json.loads(line)
        if r['joint_drift_during_capture_rad'] >= 1e-3:
            print(f'{r["id"]}: lengan bergeser selama tangkap ({r["joint_drift_during_capture_rad"]:.1e}) -> DILEWATI')
            continue
        for b in r['boards']:
            obs, T_wl = {}, None
            for c, v in r['cams'].items():
                if v.get('status') != 'OK':
                    continue
                cams.setdefault(c, dict(K=v['K'], dist=v['dist']))
                if max(abs(np.array(v['K']) - np.array(cams[c]['K'])).ravel()) > 1e-6:
                    raise SystemExit(f'K {c} berubah antar pose -- intrinsik tidak konsisten, TOLAK')
                d = v['boards'].get(b)
                if b in v['T_wl']:
                    T_wl = v['T_wl'][b]['T']
                if d and d['ids']:
                    obs[c] = dict(ids=d['ids'], uv=d['uv'])
            if T_wl is None:
                continue
            poses.append(dict(id=r['id'] if len(r['boards']) == 1 else f'{r["id"]}:{b}', board=b, role=r['role'],
                              stage=r['stage'], zone=zone.get(r['id'], '?'), T_wl=T_wl, obs=obs))
    ds = dict(square_m=sq, cameras=cams, poses=poses)
    json.dump(ds, open(a.out, 'w'), indent=1)
    print(f'{len(poses)} pose -> {a.out}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    c = sp.add_parser('capture')
    c.add_argument('--session', required=True)
    c.add_argument('--pose-id', required=True)
    c.add_argument('--board', nargs='+', required=True, choices=sorted(BOARD_ARM))
    c.add_argument('--role', required=True, choices=['FIT', 'HOLDOUT', 'R0'])
    c.add_argument('--stage', required=True)
    c.add_argument('--mount', required=True)
    c.add_argument('--frames', type=int, default=30)
    c.add_argument('--timeout', type=float, default=10.0)
    c.add_argument('--save-frames', action='store_true', help='also write every frame (PNG) under <session>/frames/')
    m = sp.add_parser('merge')
    m.add_argument('--session', required=True)
    m.add_argument('--square-mm', nargs='+', required=True)
    m.add_argument('--out', required=True)
    m.add_argument('--poses', help='g63_poses.py output (zone per pose id)')
    a = ap.parse_args()
    return capture(a) if a.cmd == 'capture' else merge(a)


if __name__ == '__main__':
    sys.exit(main())
