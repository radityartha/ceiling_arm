#!/usr/bin/env python3
"""CAM-6DOF: one screened, slow straight return; --move needs step approval.

Reuse return_rest's cosine trajectory and all retract collision screens.
Add the campaign's 97% torque margin and cancel measured torque >14 N.m.
No automatic retry, alternate segment, rail motion, or MoveIt fallback.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
import reach_dwell_probe as probe
import return_rest as rest


def margin(peak, prefix):
    expected = {f'{prefix}joint_{j}' for j in range(1, 7)}
    if peak is None or set(peak) != expected:
        raise ValueError('missing torque prediction')
    fractions = {}
    for name, value in peak.items():
        if not math.isfinite(value) or value < 0:
            raise ValueError('invalid torque prediction')
        j = int(name.rsplit('joint_', 1)[1])
        fractions[name] = (value + probe.JOINT_TORQUE_OFFSET_NM[j]) / probe.JOINT_EFFORT_LIMIT[j]
    return fractions


def wait_motion(node, goal, seconds, spin=None, clock=time.monotonic):
    """One result only; cancellation never starts a recovery movement."""
    spin = spin or (lambda: rclpy.spin_once(node, timeout_sec=0.02))
    result = goal.get_result_async()
    deadline = clock() + seconds * 3 + 5
    while True:
        reason = ('TORQUE-ABORT' if node.tau_peak > 14 else
                  'EFFORT-INVALID' if node.invalid_effort else
                  'EXEC-TIMEOUT' if clock() > deadline else None)
        if reason:
            cancel = goal.cancel_goal_async()
            cancel_deadline = clock() + 3
            while not cancel.done() and clock() < cancel_deadline:
                spin()
            return reason, None
        if result.done():
            value = result.result()
            if value is None or value.status != 4 or value.result.error_code != 0:
                return 'EXEC-FAIL', value
            return 'MOVED', value
        spin()


class GuardProbe(probe.Probe):
    def __init__(self, arms):
        self.invalid_effort = False
        super().__init__(arms[0], 0.0, 14.0, False, arms=arms)

    def _on_js(self, msg):
        super()._on_js(msg)
        if any('_joint_' in name and not math.isfinite(float(eff))
               for name, eff in zip(msg.name, msg.effort or [])):
            self.invalid_effort = True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arms', nargs='+', choices=sorted(rest.PREFIX), default=['arm_3', 'arm_4'])
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--move', action='store_true')
    ap.add_argument('--out-json', required=True)
    args = ap.parse_args()
    if len(set(args.arms)) != len(args.arms) or len({rest.GANTRY[a] for a in args.arms}) != 1:
        ap.error('one gantry, distinct arms required')
    if not math.isfinite(args.seconds) or args.seconds < 30:
        ap.error('return duration must be >=30 seconds')
    output = Path(args.out_json)
    if output.exists():
        ap.error('refuse to overwrite previous return evidence')
    record = {'verdict': 'UNSCREENED', 'move_requested': args.move, 'arms': args.arms,
              'seconds': args.seconds, 'start_time': time.time()}
    rclpy.init()
    node = GuardProbe(args.arms)
    try:
        from env_collision import CONFIG_JOINTS, EnvChecker
        from interarm_collision import CrossGantryChecker, InterArmChecker
        state = node.wait_joints(CONFIG_JOINTS, seconds=2)
        if set(state) != set(CONFIG_JOINTS) or any(not math.isfinite(v) for v in state.values()):
            raise ValueError('incomplete/nonfinite robot configuration')
        record['initial_configuration'] = state
        gantry = rest.GANTRY[args.arms[0]]
        same = InterArmChecker(gantry=f'gantry_{gantry}')
        cross = CrossGantryChecker()
        env = EnvChecker()
        verdict, segments = rest.plan_retract(gantry, args.arms, state, same, cross, env)
        if verdict != 'CLEAR':
            record['verdict'] = verdict
            return 1
        if not segments:
            record['verdict'] = 'ALREADY-REST'
            return 0
        if len(segments) != 1 or segments[0][0] != 'lurus':
            raise ValueError('one straight segment required; no fallback/retry')
        names = segments[0][2]
        moving = [a for a in args.arms if any(n.startswith(rest.PREFIX[a]) for n in names)]
        traj = RobotTrajectory()
        jt = traj.joint_trajectory
        jt.joint_names = names
        initial = JointTrajectoryPoint()
        initial.positions = [state[n] for n in names]
        initial.velocities = [0.0] * len(names)
        jt.points = [initial] + rest.build(names, initial.positions, rest.REST * len(moving), args.seconds, 60)
        # Includes the measured start point and requires the frozen map in the live scene.
        si = probe.screen_interarm(traj, moving[0], node,
                                   [a for a in probe.TOOL_FRAME if a not in moving], other_joints=state)
        se = probe.screen_env(traj, node, start_joints=state)
        if si is None or se is None or si[0] != 'CLEAR' or se[0] != 'CLEAR':
            record['verdict'] = 'COLLISION-UNSAFE'
            return 1
        record['interarm_screen'] = list(si)
        record['environment_screen'] = list(se)
        record['torque_fractions'] = {}
        for arm in moving:
            fractions = margin(probe.predict_peak_torque(traj, arm, node), rest.PREFIX[arm])
            record['torque_fractions'].update(fractions)
        record['worst_fraction'] = max(record['torque_fractions'].values())
        print(f"CAM-6DOF return margin {record['worst_fraction'] * 100:.6f}% (limit97%)", flush=True)
        if record['worst_fraction'] > .97:
            record['verdict'] = 'TORQUE-UNSAFE'
            return 1
        record['trajectory'] = {'names': names, 'positions': [list(p.positions) for p in jt.points]}
        if not args.move:
            record['verdict'] = 'DRY:CLEAR'
            return 0
        fresh = node.wait_joints(CONFIG_JOINTS, seconds=.2)
        if set(fresh) != set(state) or max(abs(fresh[n] - state[n]) for n in state) > .01:
            record['verdict'] = 'START-CHANGED'
            return 1
        if node.invalid_effort:
            record['verdict'] = 'EFFORT-INVALID'
            return 1
        import pinocchio as pin
        model, data = node._pin
        q = pin.neutral(model)
        goal_state = dict(state)
        goal_state.update(zip(names, jt.points[-1].positions))
        for name, value in goal_state.items():
            if not model.existJointName(name):
                raise ValueError(f'unknown joint {name}')
            joint = model.joints[model.getJointId(name)]
            if joint.nq != 1:
                raise ValueError(f'non-scalar joint {name}')
            q[joint.idx_q] = value
        pin.framesForwardKinematics(model, data, q)
        record['targets'] = {}
        for arm in moving:
            frame = probe.TOOL_FRAME[arm]
            if not model.existFrame(frame):
                raise ValueError(f'missing frame {frame}')
            tf = data.oMf[model.getFrameId(frame)]
            target = PoseStamped()
            target.header.frame_id = 'world'
            target.header.stamp = node.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, tf.translation)
            quat = list(map(float, pin.Quaternion(tf.rotation).coeffs()))
            target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = quat
            record['targets'][arm] = {'position': list(map(float, tf.translation)), 'quaternion_xyzw': quat}
            if not node.publish_target(target, arm):
                record['verdict'] = 'NO-MONITOR'
                return 1
        client = ActionClient(node, FollowJointTrajectory,
                              f'/gantry_{gantry}_with_arm_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=10):
            record['verdict'] = 'NO-CONTROLLER'
            return 1
        node.tau_peak = 0.0
        node.status.clear()
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = jt
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        goal = future.result() if future.done() else None
        if goal is None or not goal.accepted:
            record['verdict'] = 'REJECTED'
            return 1
        record['move_status'], _ = wait_motion(node, goal, args.seconds)
        if record['move_status'] != 'MOVED':
            record['verdict'] = record['move_status']
            return 1
        record['verdict'], won = node.collect_concurrent(8, moving)
        record['per_arm_success'] = sorted(won)
        if node.invalid_effort:
            record['verdict'] = 'EFFORT-INVALID'
            return 1
        record['final_configuration'] = node.wait_joints(CONFIG_JOINTS, seconds=.2)
        record['rest_error_deg'] = max(math.degrees(abs(record['final_configuration'][n] - rest.REST[j-1]))
                                       for arm in args.arms for j in range(1, 7)
                                       for n in [f'{rest.PREFIX[arm]}joint_{j}'])
        return 0 if record['verdict'] == 'SUCCESS' and record['rest_error_deg'] <= .5 else 1
    except Exception as exc:
        record['verdict'] = 'ERROR'
        record['error'] = str(exc)
        print(f'REFUSED/STOP: {exc}', flush=True)
        return 1
    finally:
        record.update(end_time=time.time(), tau_peak_nm=node.tau_peak, invalid_effort=node.invalid_effort)
        output.write_text(json.dumps(record, indent=2) + '\n')
        print(json.dumps({k: v for k, v in record.items() if k not in ['trajectory', 'initial_configuration', 'final_configuration']}, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
