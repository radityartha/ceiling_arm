#!/usr/bin/env python3
"""G63b e2e MOCK (domain 77 saja; fake HW, BUKAN data): probe_g63 --move -> g63b_capture -> merge -> g63b_solve vs kebenaran
g63b_fakecam. Menguji jalur ROS yang tidak bisa diuji luring: topik/TF/joint_states, prior dua muka, detektor polos via
capture, penamaan papan per fase. Urutan = P0 arm_2 dari poses_g63b_v3 (R0, A-01..A-03, R0)."""
import json, os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); G63B = os.path.dirname(HERE); G63 = os.path.dirname(G63B)
sys.path.insert(0, G63)
from g63_common import rot_deg  # noqa: E402
if os.environ.get('ROS_DOMAIN_ID') != '77':
    sys.exit('hanya domain 77')
S = os.path.join(HERE, 'e2e_session'); os.makedirs(S, exist_ok=True)
LOG = os.path.join(HERE, 'e2e_mock.log')
P = json.load(open(os.path.join(G63B, 'poses_g63b_v3.json')))['arms']['arm_2']['poses']
seq = [P[0], P[1], P[2], P[3], dict(P[0], id='A-R0b')]
def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=1200)
    out = '\n'.join(l for l in (r.stdout + r.stderr).splitlines() if 'Warn' not in l and 'buildGeom' not in l and 'self.geom' not in l)
    open(LOG, 'a').write('$ ' + ' '.join(cmd)[:250] + '\n' + out + '\n'); return r.returncode, out
for p in seq:
    rc, out = run([sys.executable, '-u', os.path.join(G63, 'probe_g63.py'), '--g63-boards', 'A', '--g63-payload-urdf',
                   os.path.join(HERE, 'payload_arm_2.urdf'), '--g63-extra-env', os.path.join(G63, 's0c_extra_env.npz'),
                   '--g63-tool-quat', ','.join(map(str, p['tool_quat_xyzw'])), '--arms', 'arm_2',
                   '--target=' + ','.join(map(str, p['tool_xyz'])), '--goal-joints', json.dumps(p['goal_joints']),
                   '--approach', '0', '--tau-max', '14', '--plan-attempts', '8', '--trials', '1', '--move',
                   '--out-json', os.path.join(S, p['id'] + '_probe.json')])
    moved = 'MOVED' in out
    print(p['id'], 'probe rc', rc, 'MOVED' if moved else 'TIDAK BERGERAK', flush=True)
    if not moved:
        sys.exit('berhenti: gerak mock gagal (lihat ' + LOG + ')')
    rc, out = run([sys.executable, '-u', os.path.join(G63B, 'g63b_capture.py'), 'capture', '--arm', 'arm_2', '--session', S,
                   '--pose-id', p['id'], '--board', 'A', 'B', '--role', p['role'], '--stage', p['stage'] if p['stage'] != 'all' else 'P0',
                   '--mount', os.path.join(G63B, 'mount_g63b.json')])
    print('  capture rc', rc, '|', ' / '.join(l.strip() for l in out.splitlines() if 'papan' in l or 'diam' in l), flush=True)
ds = os.path.join(HERE, 'e2e_ds_arm_2.json')
rc, out = run([sys.executable, os.path.join(G63B, 'g63b_capture.py'), 'merge', '--arm', 'arm_2', '--session', S,
               '--square-mm', 'A=42.0', 'B=42.0', '--out', ds, '--poses', os.path.join(G63B, 'poses_g63b_v3.json')])
print('merge rc', rc, out.splitlines()[-1] if out else '')
res = os.path.join(HERE, 'e2e_result.json')
rc, out = run([sys.executable, os.path.join(G63B, 'g63b_solve.py'), '--data', ds, '--out', res])
print('solve rc', rc, '(2 = ada parameter tak teramati -- WAJAR untuk 5 pose)')
R = json.load(open(res)); T = json.load(open(os.path.join(HERE, 'fakecam_truth.json')))
for c in R['Twc']:
    A, B = np.array(R['Twc'][c]), np.array(T['Twc'][c])
    print(f'  {c}: galat vs kebenaran {rot_deg(A[:3, :3], B[:3, :3]):.3f} deg / {np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1e3:.1f} mm; '
          f'std {R["std"][c]}')
print('  tak teramati:', R['unobservable'])
print("  G6' R0:", [(x['pose'], round(x.get('lateral_rms_m', float('nan')) * 1e3, 2)) for x in R['R0_drift']])
