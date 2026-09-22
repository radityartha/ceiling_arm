"""G22 stage 3 runner: the LOCKED schedule, end to end. docs/p1_g22_hw.md A3/A5.

Events come from g22_plan.events(row) -- the SAME list sched_screen.py walked.
Each event = one existing tool as a subprocess, every stdout line stamped with
wall time on arrival:
  retract  -> scripts/return_rest.py --arms <both arms of g> --move (self-skips at REST)
  traverse -> rail_to_g.py --gantry g --seed S <rail> --move
  task     -> scripts/reach_dwell_probe.py --arms <arm> --target=<xyz> --approach 0
              --trials 1 --tau-max 12.0 --move --monitor-csv <mon>
A background rclpy node records /joint_states (torque, rails, rotations) and
/reach_dwell/status (success events, receipt time). S26 auto-stop is checked
BEFORE every event. A HALTED task is recorded and the run continues (A3).

    python3 run_g22.py --seed S --launch-log /tmp/g22_t1.log --mon /tmp/g22_samples.csv [--dry]
Writes /tmp/g22/run.json (+ per-event logs) and copies them here.
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = '/home/user1/Documents/ceiling_arm/scripts'
sys.path.insert(0, HERE)
import g22_plan as P  # noqa: E402

FAULT_RE = re.compile(r'ARMSTATE_IN_FAULT|Kortex exception|KDetailedException|Failed to create Kortex|'
                      r'ros2_control_node-[0-9]+\]: process has died|\[ERROR\] \[ros2_control_node|'
                      r'bridge: table[12] (REJECTED|NOT armed)')
OUT = '/tmp/g22'


def pids(pat):
    out = []
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            c = open(f'/proc/{p}/cmdline', 'rb').read().replace(b'\0', b' ').decode()
        except OSError:
            continue
        if pat in c and 'bash' not in c and 'grep' not in c and str(os.getpid()) != p:
            out.append(int(p))
    return out


class Watch:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rclpy.init()
        self.rclpy = rclpy
        self.n = Node('g22_runner_watch')
        self.lock = threading.Lock()
        self.pos, self.t_pos, self.peak, self.status = {}, {}, {}, []
        self.n.create_subscription(JointState, '/joint_states', self._js, 200)
        self.n.create_subscription(String, '/reach_dwell/status', self._st, 50)
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        while self.rclpy.ok():
            self.rclpy.spin_once(self.n, timeout_sec=0.05)

    def _js(self, m):
        t = time.time()
        with self.lock:
            for i, k in enumerate(m.name):
                self.pos[k], self.t_pos[k] = m.position[i], t
                if '_a' in k and i < len(m.effort) and not math.isnan(m.effort[i]):
                    e = abs(m.effort[i])
                    if e > self.peak.get(k, (0.0, 0.0))[0]:
                        self.peak[k] = (e, t)

    def _st(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {'raw': m.data}
        d['_t_rx'] = time.time()
        with self.lock:
            self.status.append(d)

    def snap(self):
        with self.lock:
            return dict(self.pos), dict(self.t_pos), dict(self.peak), list(self.status)


def run_tool(tag, cmd, logf):
    """Subprocess with every stdout line stamped on arrival. (rc, lines, t0, t1)."""
    t0 = time.time()
    lines = []
    with open(logf, 'w') as f:
        f.write(f'# {t0:.4f} CMD {" ".join(cmd)}\n')
        pr = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1, cwd=SCRIPTS)
        for ln in pr.stdout:
            t = time.time()
            f.write(f'{t:.4f} {ln}')
            f.flush()
            lines.append((t, ln.rstrip('\n')))
        rc = pr.wait()
    t1 = time.time()
    return rc, lines, t0, t1


def main():
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--launch-log', required=True)
    ap.add_argument('--mon', required=True, help='<csv_log>_samples.csv of the monitor')
    ap.add_argument('--dry', action='store_true', help='print the event list + prechecks, send nothing')
    ap.add_argument('--plan', default=None, help='candidates json (default g22_candidates.json)')
    ap.add_argument('--out-dir', default=OUT, help='working dir for logs (default /tmp/g22)')
    ap.add_argument('--archive', default=HERE, help='where finish() copies the logs')
    ap.add_argument('--prefix', default='g22_', help='archive file-name prefix')
    a = ap.parse_args()
    OUT = a.out_dir
    os.makedirs(OUT, exist_ok=True)
    row = P.load_row(a.seed, a.plan)
    evs = P.events(row)
    rails = {g: v for g, v in P.p0_rails(row).items()}
    rec = dict(seed=a.seed, events=[], stop=None)

    def say(s):
        print(s, flush=True)
        with open(f'{OUT}/runner.log', 'a') as f:
            f.write(f'{time.time():.4f} {s}\n')

    def finish(stop=None):
        rec['stop'] = stop
        rec['status_all'] = W.snap()[3]
        json.dump(rec, open(f'{OUT}/run.json', 'w'), indent=1, default=str)
        for fn in os.listdir(OUT):
            shutil.copy(f'{OUT}/{fn}', os.path.join(
                a.archive, f'{a.prefix}{fn}' if not fn.startswith(a.prefix) else fn))
        if stop:
            say(f'AUTO-STOP: {stop}')
            sys.exit(3)

    pm = pids('lib/reachability_gng/reach_dwell_monitor')
    pr_ = pids('p1_g22/js_record.py')
    pc = pids('controller_manager/ros2_control_node')
    say(f'pids monitor {pm} recorder {pr_} ros2_control_node {pc}')
    if not (pm and pr_ and pc):
        say('REFUSE: monitor/recorder/ros2_control_node tidak hidup')
        return 2
    base_faults = len(FAULT_RE.findall(open(a.launch_log, errors='ignore').read()))
    W = Watch()
    time.sleep(2.0)
    for i, e in enumerate(evs):
        say(f'  event {i}: {e}')

    def s26(before):
        """Auto-stop checks BEFORE `before`. Returns a reason or None."""
        if not all(os.path.isdir(f'/proc/{p}') for p in (pm[0], pr_[0], pc[0])):
            return 'monitor/recorder/ros2_control_node mati'
        nf = len(FAULT_RE.findall(open(a.launch_log, errors='ignore').read()))
        if nf > base_faults:
            return f'launch log: {nf - base_faults} baris fault/exception baru'
        pos, tpos, peak, _ = W.snap()
        hi = {k: v[0] for k, v in peak.items() if v[0] > 14.0}
        if hi:
            return f'torsi terukur > 14: {hi}'
        for g in (1, 2):
            r = pos.get(f't{g}_linear_joint')
            if r is None or abs(r - rails[g]) > 0.002:
                return f'rel g{g} {r} != diharapkan {rails[g]:.3f} (> 2 mm)'
            if abs(math.degrees(pos.get(f't{g}_rotation_joint', 99))) > 0.5:
                return f'rotasi g{g} bukan 0'
        stale = [k for k in pos if '_a' in k and time.time() - tpos[k] > 1.0]
        if stale:
            return f'/joint_states basi: {stale[:3]}'
        return None

    # A3 awal: four arms at REST, rails at p0
    pos = W.snap()[0]
    dev = max(abs(pos[f'{p}joint_{j}'] - P.REST[j - 1]) for p in P.PREFIX.values() for j in range(1, 7))
    say(f'awal: maks |q - REST| keempat lengan {math.degrees(dev):.3f} deg; '
        f'rel t1 {pos["t1_linear_joint"]:.6f} t2 {pos["t2_linear_joint"]:.6f}')
    why = s26('awal') or (None if math.degrees(dev) < 0.5 else 'A3: lengan tidak di REST')
    if why:
        finish(why)
    if a.dry:
        say('DRY: tidak ada yang dikirim.')
        W.n.destroy_node()
        return 0

    rec['t0'] = t0 = time.time()
    say(f'===== t0 {t0:.4f}')
    for i, e in enumerate(evs):
        why = s26(f'event {i}')
        if why:
            finish(f'sebelum event {i} ({e["kind"]}): {why}')
        g = e['gantry']
        logf = f'{OUT}/ev{i:02d}_{e["kind"]}.log'
        if e['kind'] == 'retract':
            cmd = ['python3', '-u', 'return_rest.py', '--arms', *P.ARMS[g], '--move']
        elif e['kind'] == 'traverse':
            cmd = ['python3', '-u', os.path.join(HERE, 'rail_to_g.py'), '--gantry', str(g),
                   '--seed', str(a.seed), f'{e["to"]:.3f}', '--move'] + \
                (['--plan', a.plan] if a.plan else [])
        else:
            x, y, z = e['xyz']
            if os.path.exists('/tmp/g20_step5.json'):
                os.remove('/tmp/g20_step5.json')
            cmd = ['timeout', '600', 'python3', '-u', 'reach_dwell_probe.py', '--arms', e['arm'],
                   f'--target={x:.4f},{y:.4f},{z:.4f}', '--approach', '0', '--trials', '1',
                   '--tau-max', '12.0', '--move', '--monitor-csv', a.mon]
        say(f'  [{i}] {e["kind"]} g{g} {e.get("arm", "")} {e.get("to", e.get("xyz", ""))}')
        rc, lines, ts, te = run_tool(e['kind'], cmd, logf)
        er = dict(e, rc=rc, t_start=ts, t_end=te)
        txt = '\n'.join(ln for _, ln in lines)
        if e['kind'] == 'retract':
            m = re.search(r'torsi puncak ([0-9.]+)', txt)
            er['tau_peak'] = float(m.group(1)) if m else None
            er['skipped'] = 'sudah di rest' in txt
            if rc != 0:
                rec['events'].append(er)
                finish(f'return_rest rc={rc}')
            if er['tau_peak'] and er['tau_peak'] > 13.5:
                rec['events'].append(er)
                finish(f'return_rest torsi {er["tau_peak"]} > 13.5')
        elif e['kind'] == 'traverse':
            try:
                er['rail'] = json.loads(lines[-1][1])
            except (ValueError, IndexError):
                er['rail'] = None
            if rc != 0:
                rec['events'].append(er)
                finish(f'rail_to_g rc={rc}')
            rails[g] = e['to']
        else:
            if os.path.exists('/tmp/g20_step5.json'):
                shutil.copy('/tmp/g20_step5.json', f'{OUT}/ev{i:02d}_probe.json')
                pj = json.load(open('/tmp/g20_step5.json'))
                er['probe'] = pj[-1] if pj else None
            succ = [s for s in W.snap()[3] if s.get('event') == 'success'
                    and s.get('arm') == e['arm'] and s['_t_rx'] >= ts]
            er['t_success'] = succ[0]['_t_rx'] if succ else None
            er['success'] = succ[0] if succ else None
            v = (er.get('probe') or {}).get('verdict')
            er['verdict_probe'] = v
            if rc != 0:
                rec['events'].append(er)
                finish(f'probe rc={rc}')
            if 'UNSCREENED' in txt or 'TIDAK VALID (mesin):' in txt:
                rec['events'].append(er)
                finish('probe: UNSCREENED / TIDAK VALID (mesin)')
            if v == 'HALTED':
                say(f'  [{i}] HALTED -- dicatat, lanjut (A3); hasil TIDAK LENGKAP')
        rec['events'].append(er)
        say(f'  [{i}] rc={rc} {te - ts:.2f} s' +
            (f' success@{er["t_success"] - t0:.2f}' if er.get('t_success') else ''))
        json.dump(rec, open(f'{OUT}/run.json', 'w'), indent=1, default=str)
    tasks = [x for x in rec['events'] if x['kind'] == 'task']
    ok = [x for x in tasks if x.get('t_success')]
    rec['complete'] = len(ok) == 6
    rec['makespan'] = (max(x['t_success'] for x in ok) - t0) if rec['complete'] else None
    say(f'SELESAI: {len(ok)}/6 tugas sukses; makespan terukur '
        f'{rec["makespan"]:.2f} s' if rec['complete'] else f'SELESAI: TIDAK LENGKAP {len(ok)}/6')
    pos, _, peak, _ = W.snap()
    rec['peak_tau'] = {k: v[0] for k, v in peak.items()}
    finish(None)
    return 0


if __name__ == '__main__':
    sys.exit(main())
