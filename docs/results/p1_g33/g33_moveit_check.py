#!/usr/bin/env python3
"""G33 A4, MoveIt side: with `env_static_map` in the planning scene, is
  REST (capture config)            VALID (no "start state in collision"),
  the ev 8 CONTACT config          INVALID with a contact on env_static_map,
  R0 executed configs (sampled)    VALID?
Uses /check_state_validity (group '' = whole robot). Mock stack, no motion.

    ROS_DOMAIN_ID=77 python3 g33_moveit_check.py --out g33_moveit_check.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'scripts'))
from build_env_map import last_config  # noqa: E402
from env_collision import CONFIG_JOINTS  # noqa: E402
from env_static_map_pub import OBJECT_ID  # noqa: E402
from g33_controls import G32, T_CONTACT, States  # noqa: E402


def call(node, cli, req):
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(node, f, timeout_sec=30.0)
    if f.result() is None:
        raise RuntimeError(f'{cli.srv_name}: tidak ada jawaban')
    return f.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--r0-step', type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('g33_moveit_check')
    gps = n.create_client(GetPlanningScene, '/get_planning_scene')
    csv_ = n.create_client(GetStateValidity, '/check_state_validity')
    for c in (gps, csv_):
        if not c.wait_for_service(timeout_sec=30.0):
            print(f'TOLAK: {c.srv_name} tidak ada'); sys.exit(2)
    rq = GetPlanningScene.Request()
    rq.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    ids = [o.id for o in call(n, gps, rq).scene.world.collision_objects]
    if OBJECT_ID not in ids:
        print(f'TOLAK: {OBJECT_ID} tidak ada di planning scene ({ids})'); sys.exit(2)

    def check(joints):
        r = GetStateValidity.Request()
        r.group_name = ''
        r.robot_state.joint_state.name = list(joints)
        r.robot_state.joint_state.position = [float(v) for v in joints.values()]
        res = call(n, csv_, r)
        env = sorted({f'{c.contact_body_1}|{c.contact_body_2}' for c in res.contacts
                      if OBJECT_ID in (c.contact_body_1, c.contact_body_2)})
        other = sorted({f'{c.contact_body_1}|{c.contact_body_2}' for c in res.contacts
                        if OBJECT_ID not in (c.contact_body_1, c.contact_body_2)})
        return {'valid': res.valid, 'env_contacts': env, 'other_contacts': other}

    rep = {'scene_objects': ids}
    cfg, _ = last_config(os.path.join(G32, 'g32hw_joint_states2.csv.gz'))
    rep['REST_capture'] = check({j: cfg[j] for j in CONFIG_JOINTS})
    print('REST capture', rep['REST_capture'])
    st = States(os.path.join(G32, 'g32hw_joint_states1.csv.gz'))
    for dt in (-3.0, -2.0, -1.0, -0.5, 0.0):
        k = f'ev8_contact{dt:+.1f}s'
        rep[k] = check(st.at(T_CONTACT + dt))
        print(k, rep[k])
    r0 = json.load(open(os.path.join(G32, 'g32hw_R0_run.json')))
    bad, tot = [], 0
    for i, e in enumerate(r0['events']):
        for t in np.arange(e['t_start'], e['t_end'], a.r0_step):
            v = check(st.at(t))
            tot += 1
            if not v['valid']:
                bad.append({'ev': i, 't': float(t), **v})
    rep['R0_sampled'] = {'n': tot, 'invalid': len(bad), 'bad': bad[:20]}
    print('R0 sampled', tot, 'invalid', len(bad), bad[:3])
    json.dump(rep, open(a.out, 'w'), indent=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
