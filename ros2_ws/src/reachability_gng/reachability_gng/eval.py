"""Phase 3 evaluation: the paper's quantitative results.

Two subcommands:

  volume   Offline reachable-workspace comparison from FK dataset(s). No ROS.
           Reports occupied voxel volume + orientation coverage. Pass two
           datasets (table-locked vs table-active) to quantify the reach gain
           the table provides -- the headline arm-only vs arm+table figure.

  ik       MoveIt IK benchmark over held-out reachable poses. Needs move_group
           running (/compute_ik) and a trained GNG model. Compares seeding
           strategies: gng | none | random. Reports success rate, solve time,
           and (with --config) manipulability of the returned solution.

Examples
--------
  python3 -m reachability_gng.eval volume --datasets locked.npz active.npz
  python3 -m reachability_gng.eval ik --model model.npz --dataset dataset.npz \
      --config .../arm1_table1.yaml --methods gng none random --n 500 --csv out.csv
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import yaml


# --------------------------------------------------------------------------
# Offline: reachable-volume / coverage
# --------------------------------------------------------------------------
def voxel_volume(points_xyz, res):
    """Occupied-voxel volume (m^3) of a workspace point cloud at voxel size res."""
    keys = np.floor(points_xyz / res).astype(np.int64)
    occupied = len(np.unique(keys, axis=0))
    return occupied * (res ** 3), occupied


def cmd_volume(args):
    print(f'voxel res = {args.res} m\n')
    results = []
    for path in args.datasets:
        d = np.load(path)
        xyz = d['pose'][:, :3]
        vol, occ = voxel_volume(xyz, args.res)
        bbox = xyz.max(0) - xyz.min(0)
        results.append((path, vol, occ, bbox))
        print(f'{path}: {len(xyz)} samples | reachable volume {vol:.3f} m^3 '
              f'({occ} voxels) | bbox {bbox.round(3)}')
    if len(results) == 2:
        gain = results[1][1] / results[0][1] if results[0][1] else float('nan')
        print(f'\nreach gain (dataset2 / dataset1) = {gain:.2f}x')


# --------------------------------------------------------------------------
# Pinocchio model (for random-restart limits + manipulability)
# --------------------------------------------------------------------------
def build_model(cfg):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(cfg['urdf'])
    group = cfg['joints']
    lock = [j for j in range(1, model.njoints) if model.names[j] not in group]
    if lock:
        model = pin.buildReducedModel(model, lock, pin.neutral(model))
    data = model.createData()
    ee_id = model.getFrameId(cfg['ee_frame'])
    order = [model.names[j] for j in range(1, model.njoints)]
    lo = np.array([dict(zip(group, cfg['lower']))[n] for n in order])
    hi = np.array([dict(zip(group, cfg['upper']))[n] for n in order])
    return pin, model, data, ee_id, order, lo, hi


def manip_at(pin, model, data, ee_id, q):
    pin.forwardKinematics(model, data, q)
    J = pin.computeFrameJacobian(model, data, q, ee_id)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


# --------------------------------------------------------------------------
# Online: MoveIt IK benchmark
# --------------------------------------------------------------------------
def _pose_stamped(row, frame):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, row[:3])
    ps.pose.orientation.x, ps.pose.orientation.y, \
        ps.pose.orientation.z, ps.pose.orientation.w = map(float, row[3:7])
    return ps


def cmd_ik(args):
    import rclpy
    from moveit_msgs.srv import GetPositionIK
    from rclpy.node import Node

    from reachability_gng.gng import GNG
    from reachability_gng.seed_ik import build_ik_request, solve_ik

    gng = GNG.load(args.model)
    stats = (args.model[:-4] if args.model.endswith('.npz') else args.model) \
        + '_stats.npz'
    try:
        names = [str(n) for n in np.load(stats)['joint_names']]
    except OSError:
        names = ['t1_linear_joint', 't1_rotation_joint'] + \
            [f't1_a1_joint_{i}' for i in range(1, 7)]

    # held-out reachable test poses
    d = np.load(args.dataset)
    rng = np.random.default_rng(args.seed)
    idx = rng.choice(len(d['pose']), size=min(args.n, len(d['pose'])),
                     replace=False)
    poses = d['pose'][idx]

    # optional model for random-restart limits + manipulability
    pinmod = None
    if args.config:
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
        pinmod = build_model(cfg)
        # reorder limits to match `names`
        order = pinmod[4]
        assert order == names, f'joint order mismatch: {order} vs {names}'

    rclpy.init()
    node = Node('gng_eval')
    cli = node.create_client(GetPositionIK, '/compute_ik')
    node.get_logger().info('waiting for /compute_ik ...')
    cli.wait_for_service()

    def task_vec(row):
        if gng.task_dim == 3:
            return row[:3]
        v = row[:7].copy()
        v[3:] *= args.ori_weight
        return v

    def seed_for(method, row):
        if method == 'gng':
            return [gng.seed_q(task_vec(row))]
        if method == 'none':
            return [np.zeros(len(names))]
        if method == 'random':
            _, _, _, _, _, lo, hi = pinmod
            return [lo + rng.random(len(lo)) * (hi - lo)
                    for _ in range(args.restarts)]
        raise ValueError(method)

    rows = []
    summary = {}
    for method in args.methods:
        if method == 'random' and pinmod is None:
            node.get_logger().warn('skipping "random": needs --config')
            continue
        n_ok = 0
        times, manips = [], []
        for row in poses:
            ps = _pose_stamped(row, args.frame)
            t0 = time.perf_counter()
            ok, js, _ = False, None, None
            for seed in seed_for(method, row):
                req = build_ik_request(args.group, args.ee_frame, ps,
                                       names, seed, args.ik_timeout)
                ok, js, _ = solve_ik(cli, req, node)
                if ok:
                    break
            dt = (time.perf_counter() - t0) * 1e3
            mm = np.nan
            if ok:
                n_ok += 1
                times.append(dt)
                if pinmod is not None:
                    q = np.array(js.position[:len(names)])
                    mm = manip_at(pinmod[0], pinmod[1], pinmod[2], pinmod[3], q)
                    manips.append(mm)
            rows.append((method, ok, dt, mm))
        sr = n_ok / len(poses)
        summary[method] = (sr,
                           np.mean(times) if times else float('nan'),
                           np.median(times) if times else float('nan'),
                           np.mean(manips) if manips else float('nan'))

    node.destroy_node()
    rclpy.shutdown()

    print(f'\nIK benchmark over {len(poses)} poses, group={args.group}\n')
    print(f'{"method":<10}{"success":>9}{"mean ms":>10}'
          f'{"median ms":>11}{"mean manip":>12}')
    for m, (sr, mt, md, mm) in summary.items():
        print(f'{m:<10}{sr*100:>8.1f}%{mt:>10.2f}{md:>11.2f}{mm:>12.4f}')

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['method', 'success', 'time_ms', 'manip'])
            w.writerows(rows)
        print(f'\nper-pose results -> {args.csv}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    pv = sub.add_parser('volume', help='offline reachable-volume comparison')
    pv.add_argument('--datasets', nargs='+', required=True,
                    help='1 or 2 dataset.npz (2nd should be table-active)')
    pv.add_argument('--res', type=float, default=0.05, help='voxel size (m)')
    pv.set_defaults(func=cmd_volume)

    pk = sub.add_parser('ik', help='MoveIt IK benchmark (needs move_group)')
    pk.add_argument('--model', required=True)
    pk.add_argument('--dataset', required=True, help='source of test poses')
    pk.add_argument('--config', help='YAML (enables random-restart + manip)')
    pk.add_argument('--methods', nargs='+', default=['gng', 'none'],
                    choices=['gng', 'none', 'random'])
    pk.add_argument('--group', default='table_1_with_arm_1')
    pk.add_argument('--ee-frame', default='t1_a1_tool_frame')
    pk.add_argument('--frame', default='world')
    pk.add_argument('--task-dim-ori-weight', dest='ori_weight',
                    type=float, default=0.3)
    pk.add_argument('--n', type=int, default=500)
    pk.add_argument('--restarts', type=int, default=10,
                    help='random-restart attempts per pose')
    pk.add_argument('--ik-timeout', type=float, default=0.05)
    pk.add_argument('--seed', type=int, default=0)
    pk.add_argument('--csv')
    pk.set_defaults(func=cmd_ik)

    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
