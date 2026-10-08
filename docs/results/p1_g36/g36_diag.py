#!/usr/bin/env python3
"""G36 A1: why does MoveIt NO-PLAN the ev 11 retract of arm_3 from the end of an archived ev 10 plan?
Mock stack only (ROS_DOMAIN_ID=77), plan_only, zero motion.

Per config (end of the archived ev 10 plan, every CONFIG_JOINTS joint placed):
  1. /check_state_validity (group '') at the start and with arm_3 at REST -> valid + contacts
  2. the EXACT plan_fn request (reach_dwell_probe._plan_and_screen: joint goal REST, start_joints diff,
     5 attempts, allowed_planning_time) x n -> error_code, seconds, move_group lines from the launch log

    ROS_DOMAIN_ID=77 python3 g36_diag.py --tag base --n 5 [--plan-time 60] [--expect-map 0] --out x.json
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node

R = '/home/user1/Documents/ceiling_arm'
sys.path.insert(0, R + '/scripts')
from reach_dwell_probe import _joint_goal  # noqa: E402
from return_rest import REST  # noqa: E402
from env_collision import CONFIG_JOINTS  # noqa: E402

G34 = R + '/docs/results/p1_g34/'
CONFIGS = [('hw_s0_+103.7', G34 + 'g34_planonly_plans_g35b_hw_R10.jsonl', 0),
           ('g34n_s0_+100.3', G34 + 'g34_planonly_plans_g34n_R10.jsonl', 0),
           ('g34bhw_s0_-119.5', G34 + 'g34_planonly_plans_g34b_hw_R10.jsonl', 0)]
ARM3 = [f't2_a1_joint_{i}' for i in range(1, 7)]
CODES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


def end_state(path, sample):
    for r in map(json.loads, open(path)):
        if r['k'] == 10 and r['sample'] == sample:
            s = dict(r['start'])
            s.update(zip(r['traj']['joint_names'], r['traj']['pos'][-1]))
            return {j: s[j] for j in CONFIG_JOINTS}
    raise KeyError((path, sample))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', required=True)
    ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--plan-time', type=float, default=15.0)
    ap.add_argument('--expect-map', type=int, default=1)
    ap.add_argument('--configs', nargs='*', default=None)
    ap.add_argument('--finger', type=float, default=None,
                    help='t2_a1_right_finger_bottom_joint in the start diff (HW publishes only this finger joint)')
    ap.add_argument('--mimic', action='store_true',
                    help='also the 3 mimic finger joints of t2_a1 from the URDF mimic (tip = -0.276 b - 0.1, left_bottom = b)')
    ap.add_argument('--launch-log', default='/tmp/g36_mock_launch.log')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    rclpy.init()
    n = Node('g36_diag')
    gps = n.create_client(GetPlanningScene, '/get_planning_scene')
    csv_ = n.create_client(GetStateValidity, '/check_state_validity')
    ac = ActionClient(n, MoveGroup, 'move_action')
    assert gps.wait_for_service(timeout_sec=30) and csv_.wait_for_service(timeout_sec=30)
    assert ac.wait_for_server(timeout_sec=30)

    def call(cli, req, t=30.0):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(n, f, timeout_sec=t)
        return f.result()
    rq = GetPlanningScene.Request()
    rq.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    ids = [o.id for o in call(gps, rq).scene.world.collision_objects]
    has_map = 'env_static_map' in ids
    print(f'scene objects {ids}', flush=True)
    if has_map != bool(a.expect_map):
        print(f'TOLAK: env_static_map di scene = {has_map}, diharapkan {bool(a.expect_map)}')
        sys.exit(2)

    def valid(joints):
        r = GetStateValidity.Request()
        r.group_name = ''
        r.robot_state.joint_state.name = list(joints)
        r.robot_state.joint_state.position = [float(v) for v in joints.values()]
        res = call(csv_, r)
        return dict(valid=res.valid,
                    contacts=sorted({f'{c.contact_body_1}|{c.contact_body_2}' for c in res.contacts}))

    def plan(start):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm_3'
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.name = list(start)
        goal.request.start_state.joint_state.position = [float(v) for v in start.values()]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = a.plan_time
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.goal_constraints.append(_joint_goal(dict(zip(ARM3, REST))))
        goal.planning_options.plan_only = True
        off = os.path.getsize(a.launch_log)
        t0 = time.time()
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return dict(code='NOT-ACCEPTED', s=time.time() - t0)
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=a.plan_time + 20.0)
        res = rf.result()
        dt = time.time() - t0
        time.sleep(0.5)
        with open(a.launch_log, errors='replace') as f:
            f.seek(off)
            mg = [ln.rstrip() for ln in f if ln.startswith('[move_group')]
        if res is None:
            return dict(code='TIMEOUT-CLIENT', s=dt, mg=mg[-12:])
        jt = res.result.planned_trajectory.joint_trajectory
        out = dict(code=CODES.get(res.result.error_code.val, res.result.error_code.val), s=round(dt, 2),
                   n_pts=len(jt.points), planning_time=res.result.planning_time, mg=mg[-12:])
        if jt.points:
            i1 = list(jt.joint_names).index('t2_a1_joint_1')
            j1 = [math.degrees(p.positions[i1]) for p in jt.points]
            out['j1_range'] = [round(min(j1), 1), round(max(j1), 1)]
        return out

    rep = dict(tag=a.tag, plan_time=a.plan_time, finger=a.finger, has_map=has_map, configs={})
    for name, path, s in CONFIGS:
        if a.configs and name not in a.configs:
            continue
        st = end_state(path, s)
        if a.finger is not None:
            st['t2_a1_right_finger_bottom_joint'] = a.finger
            if a.mimic:
                st.update({'t2_a1_left_finger_bottom_joint': a.finger,
                           't2_a1_right_finger_tip_joint': -0.276 * a.finger - 0.1,
                           't2_a1_left_finger_tip_joint': -0.276 * a.finger - 0.1})
        rest = dict(st)
        rest.update(zip(ARM3, REST))
        c = dict(arm3_deg=[round(math.degrees(st[j]), 1) for j in ARM3],
                 start_valid=valid(st), rest_valid=valid(rest), plans=[])
        print(f'== {name} arm_3 {c["arm3_deg"]}\n   start {c["start_valid"]}\n   rest  {c["rest_valid"]}', flush=True)
        for i in range(a.n):
            p = plan(st)
            c['plans'].append(p)
            print(f'   plan {i}: {p["code"]} {p["s"]:.1f} s n {p.get("n_pts")} j1 {p.get("j1_range")}', flush=True)
            for ln in p.get('mg', []):
                print('      ' + ln[:220], flush=True)
        rep['configs'][name] = c
    json.dump(rep, open(a.out, 'w'), indent=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
