"""G31 A5: whole-schedule screen WITH ROTATION, PLAN-ONLY, zero motion. Needs move_group (mock stack).

sched_screen.walk (p1_g22) copied, with:
  placed     carries BOTH rotation joints (p0 0/0), never a hard 0.0
  events     g22_plan.events copied; a traverse happens when the rail OR the rotation changes
  retract    same-gantry mesh + CrossGantryChecker (true hull, C-1) with the ROTATED held state
  traverse   g29_rot_screen.sweep_rot(RotCrossChecker, rect) (lin, rot) -> (lin', rot'), SS pairs included;
             asserts both arms of the gantry are at REST -- rotation only with arms REST (G30 G3a)
  task       reach_dwell_probe._plan_and_screen (C-2: S18 gets both rotations) with the rotated start
k-of-k `--repeats`. Every task trajectory is saved (jsonl).

    python3 g31_screen.py --seeds 2 13 ... [--variant R35] [--repeats 3]
"""
import argparse
import json
import math
import os
import sys
import time
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
sys.path.insert(0, os.path.join(HERE, '../p1_g22'))
sys.path.insert(0, os.path.join(HERE, '../p1_g29'))
import g22_plan as P  # noqa: E402

TAU_MAX = 12.0
TASK_ATTEMPTS = 3     # = reach_dwell_probe --plan-attempts default (run_g32 does not pass it)
REST_TOL = math.radians(0.5)


def events(sched_g, p0, final_retract=False, mode=None):
    """g22_plan.events with rotation: [retract, traverse] when (rail, rot) changes, then tasks per arm.
    G41 (docs/p1_g41_joint_goal.md A2): final_retract adds a retract of g after its LAST task (before the next
    gantry's events); a stop's optional 'goal' {task: {joint: pos}} is carried as ev['goal'].
    G64-C (docs/p1_g64c.md A0.4): mode {'1'|'2': 'REST'|'T1'} (None / absent / REST = the old list bit for bit). A T1
    gantry with tasks: 'fold' at p0 first, every retract carries target 'T1', tasks carry retract_target 'T1'; after its
    last task: retract (T1), traverse back to p0 (if not there), 'unfold' at p0 -- whatever final_retract says."""
    ev = []
    cur = {g: (p0[g], 0.0) for g in (1, 2)}
    for g in (1, 2):
        t1 = bool(mode) and mode.get(str(g), 'REST') == 'T1' and bool(sched_g[str(g)])
        if mode and mode.get(str(g), 'REST') not in ('REST', 'T1'):
            raise ValueError(f'mode g{g} {mode.get(str(g))!r} tidak dikenal')
        tx = dict(target='T1') if t1 else {}
        if t1:
            ev.append(dict(kind='fold', gantry=g, stop=-1, at=cur[g]))
        for k, st in enumerate(sched_g[str(g)]):
            to = (st['rail_m'], math.radians(st['rot_deg']))
            if abs(to[0] - cur[g][0]) > 1e-9 or abs(to[1] - cur[g][1]) > 1e-9:
                ev.append(dict(kind='retract', gantry=g, stop=k, **tx))
                ev.append(dict(kind='traverse', gantry=g, stop=k, frm=cur[g], to=to))
                cur[g] = to
            per = {a: sorted(int(t) for t, who in st['tasks'].items()
                             if P.SCHED_ARM[who] == a) for a in P.ARMS[g]}
            for j in range(max(len(v) for v in per.values())):
                for a in P.ARMS[g]:
                    if j < len(per[a]):
                        t = per[a][j]
                        ev.append(dict(kind='task', gantry=g, stop=k, arm=a, task=t, xyz=st['xyz'][str(t)],
                                       goal=st.get('goal', {}).get(str(t)),
                                       **({'retract_target': 'T1'} if t1 else {})))
        if t1:
            n = len(sched_g[str(g)])
            ev.append(dict(kind='retract', gantry=g, stop=n, **tx))
            home = (p0[g], 0.0)
            if abs(home[0] - cur[g][0]) > 1e-9 or abs(home[1] - cur[g][1]) > 1e-9:
                ev.append(dict(kind='traverse', gantry=g, stop=n, frm=cur[g], to=home, home=True))
                cur[g] = home
            ev.append(dict(kind='unfold', gantry=g, stop=n, at=cur[g]))
        elif final_retract and sched_g[str(g)]:
            ev.append(dict(kind='retract', gantry=g, stop=len(sched_g[str(g)])))
    return ev


_ENV = []


def env_checker():
    """G33: frozen camera map (scripts/env_collision.py). Missing map RAISES."""
    if not _ENV:
        sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
        from env_collision import EnvChecker
        _ENV.append(EnvChecker())
    return _ENV[0]


def walk(rec, var, node, chk_cross, chk_same, chk_rot, R, pose, _plan_and_screen, log, save, final_retract=False,
         task_fn=None):
    """G42 (docs/p1_g42_joint_runner.md A1): task_fn(ev, start) -> (verdict, end {joint: pos} | None) replaces the
    in-process task planning (the HW probe CLI, task_cli below); retract/traverse unchanged."""
    p0 = {g: rec['p0'][str(g)] for g in (1, 2)}
    placed = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2],
              't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
    for p in P.PREFIX.values():
        placed.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
    mode = rec[var].get('mode')                         # G64-C: absent = REST, the old walk
    if mode and 'T1' in mode.values():
        sys.path.insert(0, os.path.join(HERE, '../p1_g64c'))
        import g64c_strict
        from return_rest import T1

        def strict_fn(g_):
            return lambda nm_, pts_, placed_: g64c_strict.check(g_, nm_, pts_, placed_, env_checker())  # noqa: E731
    for k, ev in enumerate(events(rec[var]['schedule'], p0, final_retract, mode)):
        g = ev['gantry']
        if ev['kind'] in ('fold', 'unfold') or (ev['kind'] == 'retract' and ev.get('target') == 'T1'):
            # G64-C A0.3: straight line only (no MoveIt fallback), + static torque and box (= return_rest --strict)
            from return_rest import plan_retract
            lines = []
            v, segs = plan_retract(g, P.ARMS[g], placed, chk_same[g], chk_cross, env_checker(), None,
                                   log=lines.append, target=None if ev['kind'] == 'unfold' else T1,
                                   strict=strict_fn(g))
            for s_ in lines:
                log(f'    [{k}] {s_}')
            log(f'    [{k}] {ev["kind"]} g{g} @({placed[f"t{g}_linear_joint"]:.3f}, '
                f'{math.degrees(placed[f"t{g}_rotation_joint"]):+.0f}): {v}')
            if v != 'CLEAR':
                return f'{ev["kind"]} g{g}:TUCK-REFUSED:{v}'
            for _, _, nm, pos, _ in segs:
                placed.update(zip(nm, pos[-1]))
        elif ev['kind'] == 'retract':
            # G34: return_rest.plan_retract -- the SAME logic the HW tool runs:
            # straight both-arm line, else arm by arm with a MoveIt plan home
            # (every screen) when the straight line is refused (G33 B6).
            from return_rest import plan_retract
            others = lambda arm: [x for x in P.PREFIX if x != arm]  # noqa: E731

            def plan_fn(arm, goal, at, _first):
                return _plan_and_screen(arm, goal, node, 0.002, 2.0, 0.15, 15.0, TAU_MAX,
                                        others(arm), start_joints=dict(at))
            lines = []
            v, segs = plan_retract(g, P.ARMS[g], placed, chk_same[g], chk_cross, env_checker(),
                                   plan_fn, log=lines.append)
            for s_ in lines:
                log(f'    [{k}] {s_}')
            log(f'    [{k}] retract g{g} @rot {math.degrees(placed[f"t{g}_rotation_joint"]):+.0f}: {v}'
                + (f' ({"+".join(x[0] for x in segs)})' if segs else ''))
            if v != 'CLEAR':
                return f'retract g{g}:{v}'
            for kind, arm, nm, pos, traj in segs:
                if traj is not None:
                    jt = traj.joint_trajectory
                    save(dict(k=k, task=None, arm=arm, kind='retract-moveit', verdict='PLANNED',
                              start=dict(placed),
                              traj=dict(joint_names=list(jt.joint_names),
                                        t=[q.time_from_start.sec + q.time_from_start.nanosec * 1e-9
                                           for q in jt.points],
                                        pos=[list(q.positions) for q in jt.points])))
                placed.update(zip(nm, pos[-1]))
        elif ev['kind'] == 'traverse':
            held_q = T1 if (mode or {}).get(str(g)) == 'T1' else P.REST          # G64-C: a T1 gantry travels at T1
            dev = max(abs(placed[f'{P.PREFIX[a]}joint_{i}'] - held_q[i - 1]) for a in P.ARMS[g] for i in range(1, 7))
            assert dev < REST_TOL, f'traverse g{g} dengan lengan TIDAK {"T1" if held_q is not P.REST else "REST"} ' \
                                   f'({math.degrees(dev):.2f} deg)'
            sw = R.sweep_rot(chk_rot, g, ev['frm'], ev['to'], placed)
            log(f'    [{k}] traverse g{g} ({ev["frm"][0]:.3f}, {math.degrees(ev["frm"][1]):+.0f}) -> '
                f'({ev["to"][0]:.3f}, {math.degrees(ev["to"][1]):+.0f}): {sw["verdict"]} min {sw["d"]*1000:.1f} mm '
                f'(lengan {sw["d_arm"]*1000:.1f}, SS {sw["d_ss"]*1000:.1f}, n {sw["n"]}) {sw["pair"]}')
            if sw['verdict'] != 'CLEAR':
                return f'traverse g{g}:{sw["verdict"]}'
            ev_ = env_checker().screen_trajectory([f't{g}_linear_joint', f't{g}_rotation_joint'],
                                                  R.rect_points(ev['frm'], ev['to']), placed)
            log(f'    [{k}] traverse g{g} vs lingkungan: {ev_[0]} min {ev_[1]*1000:.1f} mm ({ev_[2]})')
            if ev_[0] != 'CLEAR':
                return f'traverse g{g}:ENV-{ev_[0]}'
            placed[f't{g}_linear_joint'], placed[f't{g}_rotation_joint'] = ev['to']
        else:
            arm = ev['arm']
            others = [x for x in P.PREFIX if x != arm]
            t0 = time.time()
            start = dict(placed)
            if task_fn is not None:
                v, end, extra = task_fn(ev, start)
                log(f'    [{k}] task t{ev["task"]} {arm} -> {ev["xyz"]} @({placed[f"t{g}_linear_joint"]:.3f}, '
                    f'{math.degrees(placed[f"t{g}_rotation_joint"]):+.0f}): {v} ({time.time()-t0:.1f} s, CLI {extra})')
                save(dict(k=k, task=ev['task'], arm=arm, xyz=ev['xyz'], verdict=v, start=start, end=end, cli=extra))
                if v != 'PLANNED':
                    return f't{ev["task"]}:{arm}:{v}'
                placed.update(end)
                continue
            # G36: as the HW probe does (move_to, --plan-attempts 3): retake a
            # RETRYABLE refusal, and refuse plans that end where the arm cannot go home.
            import reach_dwell_probe as RDP                     # G64-C: = probe --retract-target (None = REST, old)
            RDP.RETRACT_TARGET = ev.get('retract_target')
            for _ in range(TASK_ATTEMPTS):
                v, traj = _plan_and_screen(arm, ev['goal'] or pose(ev['xyz']), node, 0.002, 2.0, 0.15,
                                           15.0, TAU_MAX, others, start_joints=start,
                                           retract_check=True)
                log(f'    [{k}]   percobaan tugas: {v}')
                if v not in RETRYABLE:
                    break
            log(f'    [{k}] task t{ev["task"]} {arm} -> {ev["xyz"]} @({placed[f"t{g}_linear_joint"]:.3f}, '
                f'{math.degrees(placed[f"t{g}_rotation_joint"]):+.0f}): {v} ({time.time()-t0:.1f} s)')
            row = dict(k=k, task=ev['task'], arm=arm, xyz=ev['xyz'], verdict=v, start=start)
            if traj is not None:
                jt = traj.joint_trajectory
                row['traj'] = dict(joint_names=list(jt.joint_names),
                                   t=[p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points],
                                   pos=[list(p.positions) for p in jt.points])
            save(row)
            if v != 'PLANNED':
                return f't{ev["task"]}:{arm}:{v}'
            placed.update(zip(jt.joint_names, jt.points[-1].positions))
    return 'PLANNED'


def task_cli(ev, start, out='/tmp/g20_step5.json'):
    """G42 A1: one task through the HW probe CLI (run_g32.task_cmd, move=False) from the PLACED state.
    (verdict, end, extra); a probe that did not answer = 'CLI-FAIL rc=..'."""
    import subprocess
    sys.path.insert(0, os.path.join(HERE, '../p1_g32'))
    import run_g32
    if os.path.exists(out):
        os.remove(out)
    gt = os.environ.get('G47_GOAL_TOL_DEG')            # G47 A/B: task joint-goal tolerance
    cmd = run_g32.task_cmd(ev, '', move=False, start=start, goal_tol=float(gt) if gt else None)
    pr = subprocess.run(cmd, cwd=run_g32.SCRIPTS, capture_output=True, text=True)
    txt = pr.stdout + pr.stderr
    goal = [ln.split('ujung rencana vs target: ')[1] for ln in txt.splitlines() if 'ujung rencana vs target: ' in ln]
    try:
        p = json.load(open(out))[-1]['plans'][ev['arm']]
    except (OSError, ValueError, KeyError, IndexError):
        return f'CLI-FAIL rc={pr.returncode}', None, dict(rc=pr.returncode, tail=txt.splitlines()[-5:])
    return p['verdict'], p['end'], dict(rc=pr.returncode, tries=p['tries'], verdicts=p.get('verdicts'), goal_err=goal)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, nargs='+', required=True)
    ap.add_argument('--variant', default='R35')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--plan', default=os.path.join(HERE, 'g31_candidates.json'))
    ap.add_argument('--out', default=os.path.join(HERE, 'g31_screen.json'))
    ap.add_argument('--traj', default=os.path.join(HERE, 'g31_screen_plans.jsonl'))
    ap.add_argument('--final-retract', action='store_true', help='G41 A2: retract g sesudah tugas terakhirnya')
    ap.add_argument('--task-cli', action='store_true',
                    help='G42 A1: tugas lewat CLI probe HW (run_g32.task_cmd, tanpa --move) sebagai subproses')
    a = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient
    from reach_dwell_probe import JOINT_TORQUE_OFFSET_NM, Probe, _plan_and_screen
    global RETRYABLE
    from reach_dwell_probe import RETRYABLE
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker, InterArmChecker
        import g29_rot_screen as R
        chk_cross = CrossGantryChecker()
        chk_same = {1: InterArmChecker(gantry='gantry_1'), 2: InterArmChecker(gantry='gantry_2')}
        chk_rot = R.RotCrossChecker()
    assert JOINT_TORQUE_OFFSET_NM[2] == 7.7, 'A8-1: offset joint_2 harus 7.7'

    def pose(xyz):
        p = PoseStamped()
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in xyz)
        p.pose.orientation.x, p.pose.orientation.w = 1.0, 0.0
        return p

    rows = {r['seed']: r for r in json.load(open(a.plan))}
    rclpy.init()
    node = Probe('arm_1', 0.0, TAU_MAX, False, arms=list(P.PREFIX))
    node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
    if not node._plan_ac.wait_for_server(timeout_sec=15.0):
        raise RuntimeError('move_group tidak ada')
    out = json.load(open(a.out)) if os.path.exists(a.out) else []
    ftraj = open(a.traj, 'a')
    try:
        for seed in a.seeds:
            rec = rows[seed]
            lines = []

            def log(s):
                print(s, flush=True)
                lines.append(s)
            vs = []
            for r in range(a.repeats):
                log(f'  seed {seed} {a.variant} sampel {r + 1}/{a.repeats}')

                def save(row, _r=r):
                    ftraj.write(json.dumps(dict(seed=seed, variant=a.variant, sample=_r, **row)) + '\n')
                    ftraj.flush()
                v = walk(rec, a.variant, node, chk_cross, chk_same, chk_rot, R, pose, _plan_and_screen, log, save,
                         a.final_retract, task_cli if a.task_cli else None)
                vs.append(v)
                if v != 'PLANNED':
                    break
            ok = len(vs) == a.repeats and all(v == 'PLANNED' for v in vs)
            print(f'seed {seed} {a.variant}: {"/".join(vs)} -> {"LOLOS" if ok else "ditolak"}', flush=True)
            out.append(dict(seed=seed, variant=a.variant, samples=vs, ok=ok, log=lines, t=time.time()))
            json.dump(out, open(a.out, 'w'), indent=1)
    finally:
        ftraj.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
