"""G40 K3 (docs/p1_g40_env_sched.md A3.1): BoxChecker (no grown box hit) vs MoveIt /check_state_validity. Mock only,
plan-free (a state query moves nothing). False-accept = BoxChecker clear AND MoveIt reports a contact with
env_static_map -- must be 0. Configs: REST arms at gantry poses whose d_box is near 0 (both sides), plus oracle IK
solutions of cached tuples whose best box distance is near 0.

    ROS_DOMAIN_ID=77 python3 g40_k3.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g40_make as W  # noqa: E402


def configs(rng):
    rows = json.load(open(W.POSE_TABLE))
    near = [d for d in rows if d['d_box'] is not None and -0.05 <= d['d_box'] <= 0.08]
    pick = [near[i] for i in rng.choice(len(near), size=min(80, len(near)), replace=False)]
    explicit = [[2, 7, 29], [1, 32, 30], [1, 31, 30], [2, 17, 34]]     # poses of the G38 NO-PLAN / PLANNED tasks
    pick = [d for d in rows if d['key'] in explicit] + pick
    out = []
    for d in pick:
        g, li, ri = d['key']
        out.append(('pose', d['key'], W.G.base_joints(g, float(W.REF.lin[li]), float(W.REF.rot[ri]))))
    tup = [json.loads(ln) for ln in open(W.TUPLE_CACHE)]
    tup = [d for d in tup if d['ok3'] and d['box_best_m'] is not None and -0.05 <= d['box_best_m'] <= 0.08]
    for d in [tup[i] for i in rng.choice(len(tup), size=min(12, len(tup)), replace=False)]:
        n, g, li, ri, s = d['key']
        arm, lin, rot = W.SLOT_ARM[(g, s)], float(W.REF.lin[li]), float(W.REF.rot[ri])
        r = W.O2.solve(W.X[n], lin, arm, tilt=True, envelope=True, keep_solutions=True, rot=rot)
        sols = r['solutions']
        for k in rng.choice(len(sols), size=min(5, len(sols)), replace=False):
            j = W.G.base_joints(g, lin, rot)
            j.update({f'{W.PREFIX[arm]}joint_{i}': float(v) for i, v in enumerate(sols[k][1], 1)})
            out.append(('task', d['key'], j))
    return out


def main():
    import rclpy
    from moveit_msgs.srv import GetStateValidity
    rng = np.random.default_rng(40)
    cfg = configs(rng)
    ec, bx = W.checkers()
    rclpy.init()
    node = rclpy.create_node('g40_k3')
    cli = node.create_client(GetStateValidity, '/check_state_validity')
    assert cli.wait_for_service(timeout_sec=15.0), '/check_state_validity tidak ada'
    rows, fa, fr = [], 0, 0
    for kind, key, j in cfg:
        g = key[1] if kind == 'task' else key[0]
        q = ec.q_from(j)
        b, wb = bx.dist(ec, q, W.gp(g))
        req = GetStateValidity.Request()
        req.group_name = ''          # '' = WHOLE robot; a group name restricts the contacts to that group's links
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(j)
        req.robot_state.joint_state.position = [float(v) for v in j.values()]
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        res = f.result()
        env_hit = [(c.contact_body_1, c.contact_body_2) for c in res.contacts
                   if 'env_static_map' in (c.contact_body_1, c.contact_body_2)]
        mine_clear = b > 0.0
        fa += mine_clear and bool(env_hit)
        fr += (not mine_clear) and not env_hit
        rows.append(dict(kind=kind, key=key, d_box=None if b == float('inf') else b, geom=wb, valid=res.valid,
                         env_contacts=env_hit[:3], n_contacts=len(res.contacts)))
        print(f'{kind} {key}: d_box {b * 1000:8.1f} mm ({wb}) | MoveIt valid {res.valid}, kontak env {len(env_hit)} '
              f'{env_hit[:1]}', flush=True)
    print(f'K3: {len(rows)} keadaan; salah-terima (kotak bebas, MoveIt tabrak env) {fa}; '
          f'kotak tabrak tapi MoveIt bebas-env {fr}')
    json.dump(dict(rows=rows, false_accept=fa, box_hit_moveit_clear=fr), open(os.path.join(HERE, 'g40_k3.json'), 'w'),
              indent=1, default=float)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
