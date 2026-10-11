"""G54 PARALLEL runner (docs/p1_g54_parallel.md A1): the two gantries of one locked schedule run AT THE SAME TIME.
Copy of p1_g32/run_g32.py's tools and guards; what changes is only WHEN an event may start.

Per gantry the events are g31_screen.events(schedule, p0, final_retract) of that gantry, in order (= run_g32). Gates:
  traverse  starts only when the OTHER gantry runs nothing; while it runs the other starts nothing (pose_to_g fails on any
            arm/rail motion, and its S28 sweep screens the other gantry as measured -- valid only if it holds still).
  retract   starts when the other gantry is not traversing AND the straight return_rest line (both arms, from the measured
            state) is CLEAR (50 mm) against every state the other gantry has RESERVED (reserve.py); then res_own = line,
            return_rest --no-plan (straight only, inside the reservation).
  task      starts when the other gantry is not traversing; the probe (--reserve-dir) screens its REAL plan against the
            other's reservation before it moves; RESERVE-BLOCKED = it did not move -> wait for the other gantry's next
            event boundary, retry.
  After every event res_own = the measured state of the gantry. Both blocked with nothing running = DEADLOCK -> AUTO-STOP.
Guards (run_g32 S26, now CONTINUOUS in a watchdog thread, 10 Hz): monitor/recorder/ros2_control_node alive, no new fault
lines, torque <= 14, rail within 2 mm / rotation within 1 deg of expected for every gantry not traversing, /joint_states
fresh. K0 + rotation envelope before t0. ANY stop -> AUTO-STOP of BOTH gantries: SIGINT to every running tool, cancel-all
on /execute_trajectory and both gantry_g_with_arm_controller actions; rc 3.

    python3 run_g54.py --seed 7 --plan ../p1_g43/g43_candidates.json --variant R180 --launch-log L --mon M [--dry]
"""
import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
SCRIPTS = '/home/user1/Documents/ceiling_arm/scripts'
sys.path.insert(0, os.path.join(R, 'p1_g32'))
sys.path.insert(0, SCRIPTS)
import run_g32 as R32  # noqa: E402
import reserve as RSV  # noqa: E402

G31S, P = R32.G31S, R32.P
ACTIONS = ['/execute_trajectory', '/gantry_1_with_arm_controller/follow_joint_trajectory',
           '/gantry_2_with_arm_controller/follow_joint_trajectory']
REST_TOL = math.radians(0.5)
# G56: run_g32's '\[ERROR\] \[ros2_control_node' never matches the real launch format
# '[ros2_control_node-4] [ERROR] [<stamp>] [controller_manager]: ...' -- generic ros2_control_node errors were invisible.
# G51 HW: the only such lines (7 per bring-up) are controller-spawner messages AT bring-up, inside base_faults.
FAULT_RE = re.compile(R32.FAULT_RE.pattern + r'|\[ros2_control_node-[0-9]+\] \[ERROR\]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--variant', required=True)
    ap.add_argument('--launch-log', required=True)
    ap.add_argument('--mon', required=True)
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--final-retract', action='store_true')
    ap.add_argument('--goal-tol-deg', type=float, default=None)
    ap.add_argument('--rot-max', type=float, default=None)
    ap.add_argument('--out-dir', default='/tmp/g54')
    ap.add_argument('--archive', default=HERE)
    ap.add_argument('--prefix', default='g54_')
    ap.add_argument('--max-blocked', type=int, default=30, help='RESERVE-BLOCKED retries per event')
    ap.add_argument('--inject-stop-on-motion', type=int, default=None, metavar='G',
                    help='NEGATIVE CONTROL (HW-safe, G56): AUTO-STOP the first time gantry G is in a RETRACT whose arms have '
                         'moved > 2 deg from the event start and are still > 10 deg from REST (mid-motion, timing-free)')
    ap.add_argument('--inject-stop', default=None, metavar='G:K',
                    help='NEGATIVE CONTROL: AUTO-STOP when gantry G is about to start its K-th own event (0-based, = g54_sim trace)')
    a = ap.parse_args()
    OUT = a.out_dir
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT)
    RES = os.path.join(OUT, 'res')
    row = {r['seed']: r for r in json.load(open(a.plan))}[a.seed]
    p0 = {g: row['p0'][str(g)] for g in (1, 2)}
    evs = G31S.events(row[a.variant]['schedule'], p0, a.final_retract)
    E = {g: [dict(e, idx=i) for i, e in enumerate(evs) if e['gantry'] == g] for g in (1, 2)}
    rails, rots = dict(p0), {1: 0.0, 2: 0.0}
    rec = dict(seed=a.seed, variant=a.variant, plan=a.plan, mode='parallel', events=[], stop=None)
    io = threading.Lock()

    def say(s):
        with io:
            print(s, flush=True)
            with open(f'{OUT}/runner.log', 'a') as f:
                f.write(f'{time.time():.4f} {s}\n')

    pm, pr_, pc = (R32.pids('lib/reachability_gng/reach_dwell_monitor'), R32.pids('p1_g22/js_record.py'),
                   R32.pids('controller_manager/ros2_control_node'))
    say(f'pids monitor {pm} recorder {pr_} ros2_control_node {pc}')
    if not (pm and pr_ and pc):
        say('REFUSE: monitor/recorder/ros2_control_node tidak hidup')
        return 2
    with open(a.launch_log, 'rb') as f_:
        base_faults = len(FAULT_RE.findall(f_.read().decode('utf-8', errors='ignore')))
        log_off = [f_.tell()]                       # G56: the watchdog reads only what was appended since (10 Hz)
    new_faults = [0]
    W = R32.Watch()
    time.sleep(2.0)
    for g in (1, 2):
        for e in E[g]:
            say(f'  g{g} event {e["idx"]}: {e}')

    # ---------------------------------------------------------------- shared state (cv guards running/epoch/stop)
    cv = threading.Condition()
    running = {1: None, 2: None}           # kind of the event in progress
    epoch = {1: 0, 2: 0}                   # +1 at every event boundary of g
    blocked = {1: False, 2: False}
    procs = {}
    st = dict(stop=None)

    def cancel_all():
        import rclpy
        from action_msgs.srv import CancelGoal
        n = W.n
        for act in ACTIONS:
            c = n.create_client(CancelGoal, act + '/_action/cancel_goal')
            if c.wait_for_service(timeout_sec=1.0):
                c.call_async(CancelGoal.Request())          # zero goal id + zero stamp = cancel ALL goals
        time.sleep(0.5)
        _ = rclpy

    def stop_all(why):
        with cv:
            if st['stop']:
                return
            st['stop'] = why
            st['t_stop'] = time.time()
            live = list(procs.items())
            cv.notify_all()
        say(f'AUTO-STOP KEDUA gantry: {why}')
        for g, p in live:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except OSError:
                pass
        cancel_all()

    def s26(g_trav):
        if not all(os.path.isdir(f'/proc/{p}') for p in (pm[0], pr_[0], pc[0])):
            return 'monitor/recorder/ros2_control_node mati'
        with open(a.launch_log, 'rb') as f_:
            f_.seek(log_off[0])
            chunk = f_.read()
        cut = chunk.rfind(b'\n') + 1                    # a partial last line waits for the next read
        new_faults[0] += len(FAULT_RE.findall(chunk[:cut].decode('utf-8', errors='ignore')))
        log_off[0] += cut
        if new_faults[0]:
            return f'launch log: {new_faults[0]} baris fault/exception baru'
        pos, tpos, peak, _ = W.snap()
        hi = {k: v[0] for k, v in peak.items() if v[0] > 14.0}
        if hi:
            return f'torsi terukur > 14: {hi}'
        for g in (1, 2):
            if g in g_trav:
                continue
            r = pos.get(f't{g}_linear_joint')
            if r is None or abs(r - rails[g]) > 0.002:
                return f'rel g{g} {r} != diharapkan {rails[g]:.3f} (> 2 mm)'
            q = pos.get(f't{g}_rotation_joint')
            if q is None or abs(math.degrees(q - rots[g])) > 1.0:
                return f'rotasi g{g} {q} != diharapkan {math.degrees(rots[g]):+.2f} deg (> 1.0 deg)'
        stale = [k for k in pos if '_a' in k and time.time() - tpos[k] > 1.0]
        if stale:
            return f'/joint_states basi: {stale[:3]}'
        return None

    def measured():
        pos = W.snap()[0]
        want = RSV.GANTRY_JOINTS[1] + RSV.GANTRY_JOINTS[2]
        missing = [k for k in want if k not in pos]
        if missing:
            raise RuntimeError(f'/joint_states tanpa {missing[:3]}')
        return {k: float(pos[k]) for k in want}

    # ---------------------------------------------------------------- prechecks (= run_g32)
    k0 = subprocess.run([sys.executable, f'{SCRIPTS}/k0_check.py', '--rail1', f'{p0[1]:.4f}', '--rail2', f'{p0[2]:.4f}',
                         '--rot1', '0', '--rot2', '0', '--seconds', '1.0', '--json', f'{OUT}/k0.json'],
                        capture_output=True, text=True)
    for ln in k0.stdout.splitlines():
        say(f'K0 | {ln}')
    sys.path.insert(0, os.path.join(R, 'p1_g48'))
    import rot_envelope as RE
    env = {'1': (-a.rot_max, a.rot_max), '2': (-a.rot_max, a.rot_max)} if a.rot_max is not None else RE.load()
    out_env = [(e['gantry'], round(math.degrees(e['to'][1]), 3)) for e in evs
               if e['kind'] == 'traverse' and not RE.allowed(e['gantry'], math.degrees(e['to'][1]), env)]
    say(f'amplop rotasi g1 {env["1"]} g2 {env["2"]}; traverse di luar: {out_env}')
    why = s26(()) or (None if k0.returncode == 0 else f'K0: {(k0.stdout.strip().splitlines() or ["?"])[-1]}') \
        or (None if not out_env else f'G48: rotasi jadwal di luar amplop {out_env}')

    def finish(rc):
        rec['stop'] = st['stop']
        rec['status_all'] = W.snap()[3]
        json.dump(rec, open(f'{OUT}/run.json', 'w'), indent=1, default=str)
        for fn in os.listdir(OUT):
            if os.path.isfile(f'{OUT}/{fn}'):
                shutil.copy(f'{OUT}/{fn}', os.path.join(a.archive, f'{a.prefix}{fn}'))
        return rc

    if why:
        st['stop'] = why
        say(f'AUTO-STOP: {why}')
        return finish(3)
    if a.dry:
        for g in (1, 2):
            for e in E[g]:
                if e['kind'] == 'task':
                    say(f'  perintah g{g} event {e["idx"]}: '
                        f'{" ".join(R32.task_cmd(e, a.mon, goal_tol=a.goal_tol_deg))} --reserve-dir {RES}')
        say('DRY: tidak ada yang dikirim.')
        W.n.destroy_node()
        return 0

    import multiprocessing as mp
    POOL = mp.get_context('spawn').Pool(1, initializer=RSV._init_worker)   # B2: screens off the runner's GIL
    POOL.apply(len, ((),))                     # worker spawned and checker loaded BEFORE t0
    m0 = measured()
    with RSV.Lock(RES) as L:
        for g in (1, 2):
            L.write(g, [m0], 'runner t0')

    # ---------------------------------------------------------------- workers
    want = {1: None, 2: None}              # time a traverse of g started waiting (fairness, A1)
    q_ev0 = {1: {}, 2: {}}                 # joint state when g's current event started (G56 inject-on-motion)

    def gate(g, kind):
        """Under cv. A traverse needs the other gantry idle; a traverse already WAITING goes before any new event of
        the other gantry (else the busier gantry starves it -- g54_sim B1)."""
        o = 2 if g == 1 else 1
        if want[o] is not None and (kind != 'traverse' or want[g] is None or want[o] < want[g]):
            return False
        if kind == 'traverse':
            if running[o] is not None:
                if want[g] is None:
                    want[g] = time.time()
                return False
            want[g] = None
            return True
        return running[o] != 'traverse'

    def run_tool(g, tag, cmd, logf):
        t0_ = time.time()
        lines = []
        with open(logf, 'w') as f:
            f.write(f'# {t0_:.4f} CMD {" ".join(cmd)}\n')
            with cv:                                  # B2: no tool is ever LAUNCHED after an AUTO-STOP
                if st['stop']:
                    f.write(f'# {time.time():.4f} TIDAK diluncurkan: AUTO-STOP\n')
                    return None, [], t0_, time.time()
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     cwd=SCRIPTS, start_new_session=True)
                procs[g] = p
            for ln in p.stdout:
                t = time.time()
                f.write(f'{t:.4f} {ln}')
                f.flush()
                lines.append((t, ln.rstrip('\n')))
            rc = p.wait()
            procs.pop(g, None)
        return rc, lines, t0_, time.time()

    def end_event(g, er, trav=None):
        """Event boundary of g: reservation shrinks to the measured state, expectations move, waiters wake."""
        if trav is not None:
            rails[g], rots[g] = trav
        try:
            with RSV.Lock(RES) as L:
                L.write(g, [measured()], f'runner g{g} ev {er["idx"]} selesai')
        except Exception as ex:                                       # noqa: BLE001
            stop_all(f'reservasi g{g} tidak bisa diperbarui: {ex}')
        with cv:
            running[g] = None
            epoch[g] += 1
            rec['events'].append(er)
            cv.notify_all()
        json.dump(rec, open(f'{OUT}/run.json', 'w'), indent=1, default=str)

    def wait_change(g, ep_other):
        """Blocked by the other gantry's reservation: wait for its next event boundary. Deadlock = both blocked and
        nothing running -> AUTO-STOP. True = retry."""
        o = 2 if g == 1 else 1
        dead = False
        with cv:
            blocked[g] = True
            cv.notify_all()
            while not st['stop'] and epoch[o] == ep_other:
                if blocked[o] and running[o] is None and running[g] is None:
                    dead = True
                    break
                cv.wait(0.2)
            blocked[g] = False
        if dead:
            stop_all(f'DEADLOCK: g{g} dan g{o} saling terhalang reservasi, tidak ada yang berjalan')
        return not st['stop']

    def retract_line(g):
        """(names, pts) of return_rest's straight line for BOTH arms of g from the measured state (None at REST)."""
        from return_rest import _rest_line
        m = measured()
        names = [f't{g}_a{k}_joint_{i}' for k in (1, 2) for i in range(1, 7)]
        cur = [m[n] for n in names]
        if max(abs(c - x) for c, x in zip(cur, list(P.REST) * 2)) < REST_TOL:
            return None
        return names, _rest_line(cur)

    def worker(g):
        o = 2 if g == 1 else 1
        for j, e in enumerate(E[g]):
            i = e['idx']
            if a.inject_stop == f'{g}:{j}':
                with cv:
                    others = running[o]
                rec['inject'] = dict(g=g, k=j, idx=i, other_running=others, t=time.time())
                stop_all(f'INJEKSI kontrol negatif sebelum g{g} event ke-{j} (idx {i}; gantry lain sedang: {others})')
                return
            n_blk = 0
            t_ready = time.time()
            while True:
                with cv:
                    while not st['stop'] and not gate(g, e['kind']):
                        cv.wait(0.2)
                    if st['stop']:
                        return
                    running[g] = e['kind']
                    q_ev0[g] = dict(W.snap()[0])
                    ep_o = epoch[o]
                    cv.notify_all()
                logf = f'{OUT}/ev{i:02d}_{e["kind"]}.log'
                if e['kind'] == 'retract':
                    line = retract_line(g)
                    if line is not None:
                        with RSV.Lock(RES) as L:
                            m = measured()
                            v, d, pair, _ = POOL.apply(RSV.screen_job, ((line[0], line[1], m, L.read(o), o),))
                            if v == 'CLEAR':
                                sts = []
                                for q in line[1]:
                                    s = dict(m)
                                    s.update(zip(line[0], q))
                                    sts.append(s)
                                L.write(g, [m] + sts, f'runner retract g{g} ev {i}')
                        say(f'  [g{g} {i}] retract vs reservasi g{o}: {d * 1000:.1f} mm {pair} -> {v}')
                        if v != 'CLEAR':
                            with cv:
                                running[g] = None
                                cv.notify_all()
                            n_blk += 1
                            if n_blk > a.max_blocked or not wait_change(g, ep_o):
                                if not st['stop']:
                                    stop_all(f'g{g} ev {i} retract terhalang {n_blk}x')
                                return
                            continue
                    cmd = ['python3', '-u', 'return_rest.py', '--arms', *P.ARMS[g], '--move', '--no-plan']
                elif e['kind'] == 'traverse':
                    cmd = ['python3', '-u', os.path.join(R, 'p1_g32/pose_to_g.py'), '--gantry', str(g),
                           '--seed', str(a.seed), '--plan', a.plan, '--variant', a.variant,
                           f'{e["to"][0]:.3f}', f'{math.degrees(e["to"][1]):.3f}', '--move'] + \
                        (['--rot-max', f'{a.rot_max:g}'] if a.rot_max is not None else [])
                else:
                    pj = f'{OUT}/ev{i:02d}_probe_try{n_blk}.json'
                    cmd = R32.task_cmd(e, a.mon, goal_tol=a.goal_tol_deg) + ['--reserve-dir', RES, '--out-json', pj]
                say(f'  [g{g} {i}] {e["kind"]} {e.get("arm", "")} {e.get("to", e.get("xyz", ""))} '
                    f'(tunggu {time.time() - t_ready:.1f} s, terhalang {n_blk})')
                rc, lines, ts, te = run_tool(g, e['kind'], cmd, logf)
                if st['stop']:
                    return
                er = dict(e, rc=rc, t_start=ts, t_end=te, t_ready=t_ready, n_blocked=n_blk)
                txt = '\n'.join(ln for _, ln in lines)
                if e['kind'] == 'task':
                    probe = None
                    if os.path.exists(pj):
                        pjs = json.load(open(pj))
                        probe = pjs[-1] if pjs else None
                    er['probe'] = probe
                    ms = (probe or {}).get('move_status') or {}
                    if rc == 0 and ms.get(e['arm']) == 'RESERVE-BLOCKED':
                        with cv:
                            running[g] = None
                            cv.notify_all()
                        n_blk += 1
                        say(f'  [g{g} {i}] RESERVE-BLOCKED ({n_blk}) -- tunggu batas event g{o}')
                        if n_blk > a.max_blocked or not wait_change(g, ep_o):
                            if not st['stop']:
                                stop_all(f'g{g} ev {i} tugas terhalang {n_blk}x')
                            return
                        continue
                    succ = [s for s in W.snap()[3] if s.get('event') == 'success'
                            and s.get('arm') == e['arm'] and s['_t_rx'] >= ts]
                    er['t_success'] = succ[0]['_t_rx'] if succ else None
                    er['success'] = succ[0] if succ else None
                    er['verdict_probe'] = (probe or {}).get('verdict')
                    if rc != 0:
                        end_event(g, er)
                        stop_all(f'g{g} ev {i} probe rc={rc}')
                        return
                    if 'UNSCREENED' in txt or 'TIDAK VALID (mesin):' in txt:
                        end_event(g, er)
                        stop_all(f'g{g} ev {i} probe: UNSCREENED / TIDAK VALID (mesin)')
                        return
                    if er['verdict_probe'] == 'HALTED':
                        say(f'  [g{g} {i}] HALTED -- dicatat, lanjut (A3); hasil TIDAK LENGKAP')
                    end_event(g, er)
                elif e['kind'] == 'traverse':
                    try:
                        er['rail'] = json.loads(lines[-1][1])
                    except (ValueError, IndexError):
                        er['rail'] = None
                    if rc != 0:
                        end_event(g, er)
                        stop_all(f'g{g} ev {i} pose_to_g rc={rc}')
                        return
                    end_event(g, er, trav=tuple(e['to']))
                else:
                    import re
                    mt = re.search(r'torsi puncak ([0-9.]+)', txt)
                    er['tau_peak'] = float(mt.group(1)) if mt else None
                    er['skipped'] = 'sudah di rest' in txt
                    if rc != 0 or (er['tau_peak'] and er['tau_peak'] > 13.5):
                        end_event(g, er)
                        stop_all(f'g{g} ev {i} return_rest rc={rc} torsi {er["tau_peak"]}')
                        return
                    end_event(g, er)
                say(f'  [g{g} {i}] rc={rc} {te - ts:.2f} s' +
                    (f' success@{er["t_success"] - t0:.2f}' if er.get('t_success') else ''))
                break

    def watchdog():
        while not st['stop'] and any(th.is_alive() for th in ths):
            with cv:
                trav = tuple(g for g in (1, 2) if running[g] == 'traverse')
            w = s26(trav)
            if w:
                stop_all(f'penjaga: {w}')
                return
            gi = a.inject_stop_on_motion
            if gi is not None and running[gi] == 'retract' and q_ev0[gi]:
                q = W.snap()[0]
                arms = [f't{gi}_a{k}_joint_{i}' for k in (1, 2) for i in range(1, 7)]
                moved = max(abs(q[n] - q_ev0[gi][n]) for n in arms)
                left = max(abs(q[n] - P.REST[(i - 1) % 6]) for i, n in enumerate(arms, 1))
                if moved > math.radians(2.0) and left > math.radians(10.0):
                    o = 2 if gi == 1 else 1
                    rec['inject'] = dict(g=gi, mode='on-motion', moved_deg=math.degrees(moved), left_deg=math.degrees(left),
                                         other_running=running[o], t=time.time())
                    stop_all(f'INJEKSI kontrol negatif: g{gi} retract di tengah gerak ({math.degrees(moved):.1f} deg '
                             f'bergerak, {math.degrees(left):.1f} deg dari REST)')
                    return
            time.sleep(0.1)

    rec['t0'] = t0 = time.time()
    say(f'===== t0 {t0:.4f} (PARALEL)')
    ths = [threading.Thread(target=worker, args=(g,), daemon=True) for g in (1, 2)]
    for th in ths:
        th.start()
    wd = threading.Thread(target=watchdog, daemon=True)
    wd.start()
    while any(th.is_alive() for th in ths) and not st['stop']:
        time.sleep(0.1)
    if not st['stop']:
        for th in ths:
            th.join()
    wd.join(timeout=2.0)
    if st['stop']:
        # A2.4 / B2: did EVERY joint stop and STAY stopped? max |dq| from t_stop + 1 s until 2 s after the last worker
        # exited (a tool launched after the stop would show here; the [t+1, t+3] window of the first version missed one).
        ts_ = st.get('t_stop') or time.time()
        while time.time() < ts_ + 1.0:
            time.sleep(0.05)
        q_ref, mx = W.snap()[0], {1: 0.0, 2: 0.0}
        t_end_ = time.time() + 2.0
        while time.time() < t_end_ or any(th.is_alive() for th in ths):
            q = W.snap()[0]
            for g in (1, 2):
                mx[g] = max([mx[g]] + [abs(q[k] - q_ref[k]) for k in RSV.GANTRY_JOINTS[g] if k in q and k in q_ref])
            if any(th.is_alive() for th in ths):
                t_end_ = time.time() + 2.0
            time.sleep(0.05)
        rec['post_stop_motion'] = mx
        say(f'gerak sesudah stop (maks |q - q(t_stop+1)| sampai 2 s sesudah worker terakhir keluar): {mx}')
        for th in ths:
            th.join(timeout=60)
        POOL.terminate()
        rec['t_stop'] = st.get('t_stop')
        rec['running_at_stop'] = {g: running[g] for g in (1, 2)}
        say(f'AUTO-STOP: {st["stop"]}')
        return finish(3)
    tasks = [x for x in rec['events'] if x['kind'] == 'task']
    ok = [x for x in tasks if x.get('t_success')]
    rec['complete'] = len(ok) == len(tasks) == sum(e['kind'] == 'task' for e in evs)
    rec['makespan'] = (max(x['t_success'] for x in ok) - t0) if rec['complete'] else None
    rec['peak_tau'] = {k: v[0] for k, v in W.snap()[2].items()}
    POOL.terminate()
    say(f'SELESAI: {len(ok)}/{len(tasks)} tugas sukses; makespan terukur {rec["makespan"]:.2f} s'
        if rec['complete'] else f'SELESAI: TIDAK LENGKAP {len(ok)}/{len(tasks)}')
    return finish(0)


if __name__ == '__main__':
    rc_ = main()
    sys.stdout.flush()
    os._exit(rc_)                       # the Watch spin thread makes a normal exit abort (rc 134, run_g22)
