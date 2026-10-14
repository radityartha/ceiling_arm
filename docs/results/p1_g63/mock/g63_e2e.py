#!/usr/bin/env python3
"""G63 end-to-end MOCK test (domain 77 ONLY; fake HW, NOT data): fakecam -> probe_g63 --move -> g63_capture ->
merge -> g63_solve, compared with the fakecam's hidden truth. Exercises every frame convention the HW run depends on
(optical frame, K, TF world->finger link, prior projection, detection, median, merge, solver).

Poses = those PLANNED in the T6 plan-check (pc_<arm>.json), in locked order, first N per arm.
Stack prerequisite: mock up (g63_mock_up.sh), monitor (g63_mon_up.sh), arms REST, boards attached, fakecam running.

    ROS_DOMAIN_ID=77 g63_e2e.py --poses poses.json --pc pc_arm_2.json pc_arm_3.json --payload P.urdf \
        --mount ../mount_guess.json --truth truth.json --session e2e_session --n 12
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G63 = os.path.dirname(HERE)
sys.path.insert(0, G63)
FILTER = ('UserWarning', 'buildGeom', 'self.geom')


def run(cmd, log):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=1200)
    out = '\n'.join(ln for ln in (r.stdout + r.stderr).splitlines() if not any(f in ln for f in FILTER))
    with open(log, 'a') as f:
        f.write(f'$ {" ".join(cmd)[:300]}\n{out}\n')
    return r.returncode, out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--poses', required=True)
    ap.add_argument('--pc', nargs='+', required=True)
    ap.add_argument('--payload', required=True)
    ap.add_argument('--mount', required=True)
    ap.add_argument('--truth', required=True)
    ap.add_argument('--session', required=True)
    ap.add_argument('--n', type=int, default=12)
    a = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        raise SystemExit('e2e hanya untuk mock domain 77')
    os.makedirs(a.session, exist_ok=True)
    log = os.path.join(a.session, 'e2e.log')
    P = json.load(open(a.poses))
    for pc_file in a.pc:
        pc = json.load(open(pc_file))
        arm = pc['arm']
        ok_ids = [r['id'] for r in pc['results'] if r['verdict'] == 'PLANNED' and r['id'] != 'REST']
        seen, order = set(), []
        for i in ok_ids:
            if i not in seen:
                seen.add(i)
                order.append(i)
        byid = {p['id']: p for p in P['arms'][arm]['poses']}
        for pid in order[:a.n]:
            p = byid[pid]
            rc, out = run([sys.executable, '-u', os.path.join(G63, 'probe_g63.py'), '--g63-payload-urdf', a.payload,
                           '--g63-tool-quat', ','.join(map(str, p['tool_quat_xyzw'])), '--arms', arm,
                           '--target', ','.join(map(str, p['tool_xyz'])), '--goal-joints', json.dumps(p['goal_joints']),
                           '--approach', '0', '--tau-max', '14', '--plan-attempts', '8', '--trials', '1', '--move',
                           '--out-json', os.path.join(a.session, f'{pid}_probe.json')], log)
            moved = 'MOVED' in out
            print(f'{arm} {pid}: gerak mock {"MOVED" if moved else "GAGAL"}', flush=True)
            if not moved:
                continue
            rc, out = run([sys.executable, '-u', os.path.join(G63, 'g63_capture.py'), 'capture', '--session', a.session,
                           '--pose-id', pid, '--board', p['board'], '--role', p['role'], '--stage', p['stage'],
                           '--mount', a.mount], log)
            print('   ' + ' | '.join(out.splitlines()[-3:]), flush=True)
        rc, out = run([sys.executable, '-u', os.path.join(G63, '..', '..', '..', 'scripts', 'return_rest.py'),
                       '--arms', arm, '--move'], log)
        print(f'{arm}: pulang REST rc {rc}', flush=True)
    ds = os.path.join(a.session, 'dataset.json')
    run([sys.executable, os.path.join(G63, 'g63_capture.py'), 'merge', '--session', a.session, '--square-mm', 'A=50.0',
         'B=50.0', '--poses', a.poses, '--out', ds], log)
    res = os.path.join(a.session, 'solve.json')
    rc, out = run([sys.executable, os.path.join(G63, 'g63_solve.py'), '--data', ds, '--out', res], log)
    print(out[-1500:])
    from g63_common import rot_deg
    truth = json.load(open(a.truth))
    r = json.load(open(res))
    for c, T in r['Twc'].items():
        T, Tt = np.array(T), np.array(truth['Twc'][c])
        print(f'{c}: vs KEBENARAN rot {rot_deg(Tt[:3, :3], T[:3, :3]):.4f} deg, pos {np.linalg.norm(T[:3, 3] - Tt[:3, 3]) * 1000:.2f} mm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
