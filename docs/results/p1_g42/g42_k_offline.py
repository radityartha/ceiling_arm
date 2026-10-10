"""G42 A2 offline checks. K-R: run_g32.task_cmd argv == the old inline argv on every goal-less schedule. K-P: probe CLI
refuses malformed --goal-joints / --start-joints with --move (rc 2, before rclpy.init)."""
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../p1_g32'))
import run_g32  # noqa: E402
G31S = run_g32.G31S


def old_cmd(e, mon):        # run_g32.py before G42, copied verbatim
    x, y, z = e['xyz']
    return ['timeout', '600', 'python3', '-u', 'reach_dwell_probe.py', '--arms', e['arm'],
            f'--target={x:.4f},{y:.4f},{z:.4f}', '--approach', '0', '--trials', '1',
            '--tau-max', '12.0', '--move', '--monitor-csv', mon]


n = bad = 0
files = []
for f in sorted(glob.glob(os.path.join(HERE, '../p1_g*/*candidates*.json'))):
    try:
        rows = json.load(open(f))
    except ValueError:
        continue
    if not isinstance(rows, list):
        continue
    nf = 0
    for r in rows:
        if not isinstance(r, dict) or 'p0' not in r:
            continue
        p0 = {g: r['p0'][str(g)] for g in (1, 2)}
        for var, v in r.items():
            if not (isinstance(v, dict) and 'schedule' in v):
                continue
            try:
                evs = G31S.events(v['schedule'], p0)
            except (KeyError, TypeError):
                continue
            for e in evs:
                if e['kind'] != 'task' or e.get('goal'):
                    continue
                n += 1
                nf += 1
                bad += run_g32.task_cmd(e, '/tmp/m.csv') != old_cmd(e, '/tmp/m.csv')
    if nf:
        files.append((os.path.relpath(f, HERE), nf))
print(f'K-R: {n} tugas tanpa tujuan dari {len(files)} berkas jadwal, argv beda {bad}')
for f, k in files:
    print(f'    {f}: {k}')

P = '/home/user1/Documents/ceiling_arm/scripts/reach_dwell_probe.py'
g6 = {f't1_a1_joint_{i}': 0.1 for i in range(1, 7)}
cases = {
    'kurang 1 sendi': ['--arms', 'arm_1', '--target=1,0,1', '--goal-joints=' + json.dumps(dict(list(g6.items())[:5]))],
    'lebih 1 sendi': ['--arms', 'arm_1', '--target=1,0,1', '--goal-joints=' + json.dumps(dict(g6, t1_a2_joint_1=0.0))],
    'lengan salah': ['--arms', 'arm_2', '--target=1,0,1', '--goal-joints=' + json.dumps(g6)],
    'tanpa --arms': ['--target=1,0,1', '--goal-joints=' + json.dumps(g6)],
    'bukan JSON': ['--arms', 'arm_1', '--target=1,0,1', '--goal-joints=abc'],
    '--start-joints + --move': ['--arms', 'arm_1', '--target=1,0,1', '--plan-check', '--move',
                                '--start-joints=' + json.dumps(g6)],
    '--start-joints tanpa --plan-check': ['--arms', 'arm_1', '--target=1,0,1', '--start-joints=' + json.dumps(g6)],
}
okp = 0
for lab, args in cases.items():
    pr = subprocess.run(['python3', P] + args, capture_output=True, text=True, timeout=60)
    msg = [ln for ln in pr.stderr.splitlines() if 'error' in ln]
    okp += pr.returncode == 2
    print(f'K-P {lab}: rc {pr.returncode} {msg[-1][:150] if msg else ""}')
print(f'K-P: {okp}/{len(cases)} ditolak rc 2')
print('K offline: LULUS' if bad == 0 and okp == len(cases) and n > 0 else 'K offline: GAGAL')
