"""Offline forward-kinematics sampler for the redundant arm+table system.

Samples joint configurations q = [table_linear, table_rotation, arm_j1..j6]
uniformly within limits, computes the end-effector pose via FK and the
manipulability index sqrt(det(J J^T)) via the Jacobian, and writes a dataset:

    dataset.npz
        pose : (N, 7)  end-effector xyz + quaternion (xyzw)
        q    : (N, 8)  joint configuration
        manip: (N,)    manipulability index

Dataset column convention consumed by train.py: task features are the LEADING
columns (xyz, then quat), q follows -- matching GNG's contiguous task_dim slice.

FK backend: Pinocchio (recommended). If unavailable, install python3-pinocchio
or swap in PyKDL here. The URDF must be a flattened .urdf (xacro already
expanded), produced e.g. with:

    xacro workcell.urdf.xacro > /tmp/workcell.urdf
"""

from __future__ import annotations

import argparse

import numpy as np
import yaml


def _load_pinocchio():
    try:
        import pinocchio as pin
    except ImportError as e:  # pragma: no cover - environment dependent
        raise SystemExit(
            'pinocchio not found. Install with `sudo apt install '
            'ros-humble-pinocchio` (or python3-pinocchio), or replace the FK '
            'backend in data_gen.py with PyKDL.'
        ) from e
    return pin


def sample(cfg, n_samples, seed=0):
    pin = _load_pinocchio()
    rng = np.random.default_rng(seed)

    model = pin.buildModelFromUrdf(cfg['urdf'])
    # lock every actuated joint NOT in the group, so we sample only our 8 DOF.
    # (joint 0 is the universe joint; skip it.)
    group = cfg['joints']
    lock = [j for j in range(1, model.njoints)
            if model.names[j] not in group]
    if lock:
        model = pin.buildReducedModel(model, lock, pin.neutral(model))
    data = model.createData()

    # reorder limits to match the joint order pinocchio actually uses
    order = [model.names[j] for j in range(1, model.njoints)]
    assert set(order) == set(group), f'group/model mismatch: {order} vs {group}'

    ee_id = model.getFrameId(cfg['ee_frame'])
    assert len(group) == len(cfg['lower']) == len(cfg['upper']), \
        'joints/lower/upper length mismatch'
    lo_by_name = dict(zip(group, cfg['lower']))
    hi_by_name = dict(zip(group, cfg['upper']))
    lo = np.array([lo_by_name[n] for n in order], dtype=float)
    hi = np.array([hi_by_name[n] for n in order], dtype=float)

    poses = np.empty((n_samples, 7))
    qs = np.empty((n_samples, len(group)))
    manip = np.empty((n_samples,))

    for i in range(n_samples):
        q = lo + rng.random(len(group)) * (hi - lo)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, ee_id)
        M = data.oMf[ee_id]
        quat = pin.Quaternion(M.rotation).coeffs()  # xyzw
        poses[i] = np.concatenate([M.translation, quat])
        J = pin.computeFrameJacobian(model, data, q, ee_id)
        JJt = J @ J.T
        manip[i] = float(np.sqrt(max(np.linalg.det(JJt), 0.0)))
        qs[i] = q

    return poses, qs, manip, order


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', required=True, help='YAML with urdf/joints/limits')
    ap.add_argument('--out', default='dataset.npz')
    ap.add_argument('--n', type=int, default=200000, help='number of samples')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    poses, qs, manip, order = sample(cfg, args.n, args.seed)
    np.savez(args.out, pose=poses, q=qs, manip=manip,
             joint_names=np.array(order))
    print(f'Wrote {args.out}: {len(poses)} samples, '
          f'joints={order}, manip[min/mean/max]='
          f'{manip.min():.4f}/{manip.mean():.4f}/{manip.max():.4f}')


if __name__ == '__main__':
    main()
