"""G44 B2 tests on the MOCK (ROS_DOMAIN_ID=77), plan only, zero motion (docs/p1_g44_start_jump.md).

    ROS_DOMAIN_ID=77 python3 g44_mock.py karsip   -> /check_state_validity on every archived PLANNED start
    ROS_DOMAIN_ID=77 python3 g44_mock.py controls -> P1 (G36 B1 finger +0.05), P2 (G38 s1 t2), N1 (finger 0), n = 5
"""
import json
import math
import os
import sys
import warnings

R = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R + '/scripts')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g44_measure as GM  # noqa: E402

import rclpy  # noqa: E402
from moveit_msgs.action import MoveGroup  # noqa: E402
from moveit_msgs.msg import MoveItErrorCodes  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402

import reach_dwell_probe as P  # noqa: E402
from return_rest import REST  # noqa: E402
from env_collision import CONFIG_JOINTS  # noqa: E402

N = 5
ARM3 = [f't2_a1_joint_{i}' for i in range(1, 7)]
ARMS = ['arm_1', 'arm_2', 'arm_3', 'arm_4']


def node_up():
    rclpy.init()
    node = P.Probe('arm_1', 0.0, 12.0, False, arms=ARMS)
    node._plan_ac = ActionClient(node, MoveGroup, 'move_action')
    assert node._plan_ac.wait_for_server(timeout_sec=15.0), 'move_group tidak ada'
    return node


def karsip():
    node = node_up()
    for name, files in (('NYATA', GM.NYATA), ('MOCK', GM.mock_files())):
        tot, bad = 0, []
        for f in files:
            for d, _ in GM.rows(f):
                tot += 1
                v, c = P.start_valid(node, d['start'])
                if v is not True:
                    bad.append((os.path.basename(f), d.get('seed'), d.get('task'), d.get('sample'), d.get('arm'), v, c))
        for b in bad:
            print(f'  INVALID {b}')
        print(f'{name}: start VALID {tot - len(bad)}/{tot}', flush=True)


def raw_plan(node, arm, start, goal):
    """The MoveIt request _plan_and_screen sends, WITHOUT any screen: did MoveIt itself answer SUCCESS?"""
    g = MoveGroup.Goal()
    g.request.group_name = arm
    g.request.start_state.is_diff = True
    g.request.start_state.joint_state.name = list(start)
    g.request.start_state.joint_state.position = [float(v) for v in start.values()]
    g.request.num_planning_attempts = 5
    g.request.allowed_planning_time = 15.0
    g.request.max_velocity_scaling_factor = 0.15
    g.request.max_acceleration_scaling_factor = 0.15
    g.request.goal_constraints.append(P._joint_goal(goal))
    g.planning_options.plan_only = True
    fut = node._plan_ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    gh = fut.result()
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=40.0)
    return rf.result().result.error_code.val == MoveItErrorCodes.SUCCESS


def controls():
    node = node_up()
    # P1 / N1: G36 B1 -- end of the archived ev 10 plan (hw_s0, +103.7 deg), arm_3 home to REST
    f = GM.D + 'p1_g34/g34_planonly_plans_g35b_hw_R10.jsonl'
    r = [x for x in map(json.loads, open(f)) if x['k'] == 10 and x['sample'] == 0][0]
    end = dict(r['start'])
    end.update(zip(r['traj']['joint_names'], r['traj']['pos'][-1]))
    base = {j: end[j] for j in CONFIG_JOINTS}
    # P2: G38 s1 t2 -- archived start, goal = the archived plan's end
    f2 = GM.D + 'p1_g38/g38_planonly_plans.jsonl'
    r2 = [x for x in map(json.loads, open(f2)) if x['seed'] == 1 and x['task'] == 2][0]
    cases = [('P1 G36 B1 jari +0.05', 'arm_3', dict(base, t2_a1_right_finger_bottom_joint=0.05),
              dict(zip(ARM3, REST)), 'TOLAK'),
             ('P2 G38 s1 t2', r2['arm'], dict(r2['start']),
              dict(zip(r2['traj']['joint_names'], r2['traj']['pos'][-1])), 'TOLAK'),
             ('N1 G36 B1 jari 0', 'arm_3', dict(base, t2_a1_right_finger_bottom_joint=0.0),
              dict(zip(ARM3, REST)), 'terima')]
    for lab, arm, start, goal, want in cases:
        v, c = P.start_valid(node, start)
        others = [a for a in ARMS if a != arm]
        verd, raw = [], []
        for _ in range(N):
            raw.append(raw_plan(node, arm, start, goal))
            vv, _t = P._plan_and_screen(arm, goal, node, 0.002, 2.0, 0.15, 15.0, 12.0, others, start_joints=start)
            verd.append(vv)
        print(f"{lab}: start valid {v} {c[:3]}; MoveIt mentah SUCCESS {sum(raw)}/{N}; verdict penyaring {verd} "
              f"(harap {want}: {'START-JUMP semua' if want == 'TOLAK' else 'tanpa START-JUMP'})", flush=True)
        ok = all(x == 'START-JUMP' for x in verd) if want == 'TOLAK' else 'START-JUMP' not in verd
        print(f'  -> {"LULUS" if ok else "GAGAL"}', flush=True)


if __name__ == '__main__':
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        {'karsip': karsip, 'controls': controls}[sys.argv[1]]()
