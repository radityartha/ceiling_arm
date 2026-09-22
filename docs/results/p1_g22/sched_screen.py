"""G22 A2 (iv): whole-schedule screen, PLAN-ONLY, zero motion. Needs move_group.

The A3 event list (g22_plan.events) is walked with everything PLACED through
MoveIt start_state -- rails, and every arm at REST or at the end of its last
plan -- exactly as g20 quad_screen did for four fixed targets:
  retract  : straight joint-space path -> REST (return_rest's own path),
             screened same-gantry mesh + cross-gantry hull
  traverse : S24 sweep (g22_plan.sweep_screen)
  task     : reach_dwell_probe._plan_and_screen -- torque (+7.7 j2, A8-1) and
             S18 vs the THREE other arms, --tau-max 12.0 as on the day
k-of-k: `--repeats` independent walks, every step must pass in all of them.

    python3 sched_screen.py --seeds 0 2 3 ...  [--repeats 3] [--stop-first]
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
sys.path.insert(0, HERE)
import g22_plan as P  # noqa: E402

TAU_MAX = 12.0


def walk(row, node, chk_cross, chk_same, pose, _plan_and_screen, log):
    rail = P.p0_rails(row)
    placed = {'t1_linear_joint': rail[1], 't2_linear_joint': rail[2],
              't1_rotation_joint': 0.0, 't2_rotation_joint': 0.0}
    for p in P.PREFIX.values():
        placed.update({f'{p}joint_{i}': v for i, v in enumerate(P.REST, 1)})
    for k, ev in enumerate(P.events(row)):
        g = ev['gantry']
        if ev['kind'] == 'retract':
            names = [f'{P.PREFIX[a]}joint_{i}' for a in P.ARMS[g] for i in range(1, 7)]
            start = [placed[x] for x in names]
            goal = P.REST * 2
            if max(abs(s - q) for s, q in zip(start, goal)) < math.radians(0.5):
                log(f'    [{k}] retract g{g}: sudah REST -> dilewati')
                continue
            pts = [[s + (q - s) * 0.5 * (1 - math.cos(math.pi * j / 60))
                    for s, q in zip(start, goal)] for j in range(1, 61)]
            held = {x: v for x, v in placed.items() if x not in names}
            r1 = chk_same[g].screen_trajectory(names, pts, held)
            r2 = chk_cross.screen_trajectory(names, pts, held, only=f't{g}_a')
            v = 'CLEAR' if r1[0] == r2[0] == 'CLEAR' else f'{r1[0]}/{r2[0]}'
            log(f'    [{k}] retract g{g}: se-gantry {r1[1]*1000:.1f} / antar {r2[1]*1000:.1f} mm -> {v}')
            if v != 'CLEAR':
                return f'retract g{g}:{v}'
            placed.update(dict(zip(names, goal)))
        elif ev['kind'] == 'traverse':
            sw = P.sweep_screen(chk_cross, g, ev['frm'], ev['to'], placed)
            log(f'    [{k}] traverse g{g} {ev["frm"]:.3f}->{ev["to"]:.3f}: {sw[0]} '
                f'min {sw[1]*1000:.1f} mm ({sw[2][0]} <-> {sw[2][1]})')
            if sw[0] != 'CLEAR':
                return f'traverse g{g}:{sw[0]}'
            placed[f't{g}_linear_joint'] = ev['to']
        else:
            arm = ev['arm']
            others = [x for x in P.PREFIX if x != arm]
            t0 = time.time()
            v, traj = _plan_and_screen(arm, pose(ev['xyz']), node, 0.002, 2.0, 0.15,
                                       15.0, TAU_MAX, others, start_joints=dict(placed))
            log(f'    [{k}] task t{ev["task"]} {arm} -> {ev["xyz"]} @rail '
                f'{placed[f"t{g}_linear_joint"]:.3f}: {v} ({time.time()-t0:.1f} s)')
            if v != 'PLANNED':
                return f't{ev["task"]}:{arm}:{v}'
            jt = traj.joint_trajectory
            placed.update(zip(jt.joint_names, jt.points[-1].positions))
    return 'PLANNED'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, nargs='+', required=True)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--stop-first', action='store_true',
                    help='A2: berhenti di seed PERTAMA yang lolos')
    ap.add_argument('--out', default=os.path.join(HERE, 'g22_screen.json'))
    ap.add_argument('--plan', default=None, help='candidates json (default g22_candidates.json)')
    a = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient
    from reach_dwell_probe import JOINT_TORQUE_OFFSET_NM, Probe, _plan_and_screen
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from interarm_collision import CrossGantryChecker, InterArmChecker
        chk_cross = CrossGantryChecker()
        chk_same = {1: InterArmChecker(gantry='gantry_1'), 2: InterArmChecker(gantry='gantry_2')}
    assert JOINT_TORQUE_OFFSET_NM[2] == 7.7, 'A8-1: offset joint_2 harus 7.7'

    def pose(xyz):
        p = PoseStamped()
        p.header.frame_id = 'world'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in xyz)
        p.pose.orientation.x, p.pose.orientation.w = 1.0, 0.0
        return p

    rclpy.init()
    node = Probe('arm_1', 0.0, TAU_MAX, False, arms=list(P.PREFIX))
    node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
    if not node._plan_ac.wait_for_server(timeout_sec=15.0):
        raise RuntimeError('move_group tidak ada')
    out = json.load(open(a.out)) if os.path.exists(a.out) else []
    try:
        for seed in a.seeds:
            row = P.load_row(seed, a.plan)
            if not row.get('ok_ii'):
                print(f'seed {seed}: tidak lolos (i)-(iii), dilewati')
                continue
            lines = []

            def log(s):
                print(s, flush=True)
                lines.append(s)
            vs = []
            for r in range(a.repeats):
                log(f'  seed {seed} sampel {r + 1}/{a.repeats}')
                v = walk(row, node, chk_cross, chk_same, pose, _plan_and_screen, log)
                vs.append(v)
                if v != 'PLANNED':
                    break
            ok = len(vs) == a.repeats and all(v == 'PLANNED' for v in vs)
            print(f'seed {seed}: {"/".join(vs)} -> {"LOLOS" if ok else "ditolak"}', flush=True)
            out.append(dict(seed=seed, samples=vs, ok=ok, log=lines, t=time.time()))
            json.dump(out, open(a.out, 'w'), indent=1)
            if ok and a.stop_first:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
