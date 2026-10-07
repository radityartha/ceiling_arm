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
REST_TOL = math.radians(0.5)


def events(sched_g, p0):
    """g22_plan.events with rotation: [retract, traverse] when (rail, rot) changes, then tasks per arm."""
    ev = []
    cur = {g: (p0[g], 0.0) for g in (1, 2)}
    for g in (1, 2):
        for k, st in enumerate(sched_g[str(g)]):
            to = (st['rail_m'], math.radians(st['rot_deg']))
            if abs(to[0] - cur[g][0]) > 1e-9 or abs(to[1] - cur[g][1]) > 1e-9:
                ev.append(dict(kind='retract', gantry=g, stop=k))
                ev.append(dict(kind='traverse', gantry=g, stop=k, frm=cur[g], to=to))
                cur[g] = to
            per = {a: sorted(int(t) for t, who in st['tasks'].items()
                             if P.SCHED_ARM[who] == a) for a in P.ARMS[g]}
            for j in range(max(len(v) for v in per.values())):
                for a in P.ARMS[g]:
                    if j < len(per[a]):
                        t = per[a][j]
                        ev.append(dict(kind='task', gantry=g, stop=k, arm=a, task=t, xyz=st['xyz'][str(t)]))
    return ev


_ENV = []


def env_checker():
    """G33: frozen camera map (scripts/env_collision.py). Missing map RAISES."""
    if not _ENV:
        sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
        from env_collision import EnvChecker
        _ENV.append(EnvChecker())
    return _ENV[0]


def walk(rec, var, node, chk_cross, chk_same, chk_rot, R, pose, _plan_and_screen, log, save):
    p0 = {g: rec['p0'][str(g)] for g in (1, 2)}
    placed = {'t1_linear_joint': p0[1], 't2_linear_joint': p0[2],
              't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
    for p in P.PREFIX.values():
        placed.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
    for k, ev in enumerate(events(rec[var]['schedule'], p0)):
        g = ev['gantry']
        if ev['kind'] == 'retract':
            names = [f'{P.PREFIX[a]}joint_{i}' for a in P.ARMS[g] for i in range(1, 7)]
            start = [placed[x] for x in names]
            goal = P.REST * 2
            if max(abs(s - q) for s, q in zip(start, goal)) < REST_TOL:
                log(f'    [{k}] retract g{g}: sudah REST -> dilewati')
                continue
            pts = [[s + (q - s) * 0.5 * (1 - math.cos(math.pi * j / 60))
                    for s, q in zip(start, goal)] for j in range(1, 61)]
            held = {x: v for x, v in placed.items() if x not in names}
            r1 = chk_same[g].screen_trajectory(names, pts, held)
            r2 = chk_cross.screen_trajectory(names, pts, held, only=f't{g}_a')
            r3 = env_checker().screen_trajectory(names, pts, placed)
            v = 'CLEAR' if r1[0] == r2[0] == r3[0] == 'CLEAR' else f'{r1[0]}/{r2[0]}/{r3[0]}'
            log(f'    [{k}] retract g{g} @rot {math.degrees(placed[f"t{g}_rotation_joint"]):+.0f}: se-gantry '
                f'{r1[1]*1000:.1f} / antar {r2[1]*1000:.1f} / lingkungan {r3[1]*1000:.1f} mm -> {v}')
            if v != 'CLEAR':
                return f'retract g{g}:{v}'
            placed.update(dict(zip(names, goal)))
        elif ev['kind'] == 'traverse':
            dev = max(abs(placed[f'{P.PREFIX[a]}joint_{i}'] - P.REST[i - 1]) for a in P.ARMS[g] for i in range(1, 7))
            assert dev < REST_TOL, f'traverse g{g} dengan lengan TIDAK REST ({math.degrees(dev):.2f} deg)'
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
            v, traj = _plan_and_screen(arm, pose(ev['xyz']), node, 0.002, 2.0, 0.15,
                                       15.0, TAU_MAX, others, start_joints=start)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, nargs='+', required=True)
    ap.add_argument('--variant', default='R35')
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--plan', default=os.path.join(HERE, 'g31_candidates.json'))
    ap.add_argument('--out', default=os.path.join(HERE, 'g31_screen.json'))
    ap.add_argument('--traj', default=os.path.join(HERE, 'g31_screen_plans.jsonl'))
    a = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient
    from reach_dwell_probe import JOINT_TORQUE_OFFSET_NM, Probe, _plan_and_screen
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
                v = walk(rec, a.variant, node, chk_cross, chk_same, chk_rot, R, pose, _plan_and_screen, log, save)
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
