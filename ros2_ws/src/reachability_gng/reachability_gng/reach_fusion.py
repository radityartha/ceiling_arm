"""Collision-free fusion of the reach-map (action) and env-map (perception).

Sensei's Meso adjacency-matrix method run ON each arm's reach graph (the free
config-space graph, world frame). For a chosen target object:
  * carve  : env nodes within target_radius of the target are not obstacles.
  * danger : reach nodes within collision_radius of a remaining obstacle.
  * S      : diffusion sum_l gamma^l A_hat^l (Meso sumMat), precomputed per arm.
  * cfree  : norm(S@target) - norm(S@danger) > 0 = collision-free corridor.
Target = /detected_objects centroid, selected by obj_N (ground truth, via
ObjnLocalizer), class substring, or index; switch live on /reach_fusion/set_target.
Publishes /reach_fusion/markers: per arm armN_{free,danger,cfree,edges} + target.
"""
from __future__ import annotations

import re

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG
from reachability_gng.objn_localizer import ObjnLocalizer

_OBJN_RE = re.compile(r'^obj_\d+$')
# green is reserved for the env topo_map; red=danger, cyan=collision-free.
_ARM_COLORS = {'arm1': (0.95, 0.95, 0.95), 'arm2': (0.15, 0.45, 1.0),
               'arm3': (0.98, 0.70, 0.10), 'arm4': (0.85, 0.25, 0.90)}


def _diffusion_matrix(n, edges, gamma, levels):
    """S = sum_{l=1..levels} gamma^l A_hat^l (row-normalised adjacency, Meso sumMat)."""
    A = np.zeros((n, n))
    for i, j in edges:
        A[i, j] = A[j, i] = 1.0
    deg = A.sum(1)
    deg[deg == 0] = 1.0
    A_hat = A / deg[:, None]
    S, P = np.zeros((n, n)), np.eye(n)
    for l in range(1, levels + 1):
        P = A_hat @ P
        S += (gamma ** l) * P
    return S


class ReachFusion(Node):
    def __init__(self):
        super().__init__('reach_fusion')
        p = self.declare_parameter
        p('arm_models', ['/tmp/arm1_model.npz', '/tmp/arm2_model.npz',
                         '/tmp/arm3_model.npz', '/tmp/arm4_model.npz'])
        p('arm_labels', ['arm1', 'arm2', 'arm3', 'arm4'])
        p('env_markers_topic', '/topo_map/markers')
        p('objects_topic', '/detected_objects')
        p('world_frame', 'world')
        p('collision_radius', 0.15)   # reach node this close to an obstacle = danger
        p('target_radius', 0.15)      # env nodes this close to target = carved out
        p('target_label', '')         # obj_N / class substring; '' -> target_index
        p('target_index', 0)
        p('reach_tol', 0.20)          # arm reaches target if nearest node this close
        p('diffusion_gamma', 0.5)
        p('diffusion_levels', 4)
        p('reach_max_z', 2.05)        # drop reach nodes above the ceiling gantry
        p('publish_hz', 2.0)
        # 2c energy: pick the winning arm by the executor's calibrated J over the
        # collision-free grasp candidates (cfree nodes within pool_radius of the
        # target). Travel terms need /joint_states; without it J uses dist+hold-manip.
        p('joint_states_topic', '/joint_states')
        p('energy_pool_radius', 0.20)   # grasp candidates: non-danger within this
        p('w_gantry_lin', 2.0); p('w_gantry_rot', 12.0); p('w_arm', 20.0)
        p('w_dist', 3.0); p('w_hold', 1.0); p('w_manip', 1.0)
        p('ref_gantry_lin', 0.95); p('ref_gantry_rot', 0.70); p('ref_arm', 6.0)
        p('ref_dist', 1.36); p('ref_hold', 2.90); p('ref_manip', 0.145)
        # 3a execution: publish to /reach_fusion/execute to send the winning
        # arm's grasp-node q as a MoveGroup joint goal (plan + execute).
        # coupled per-arm groups (plan gantry + arm together). gantry_2_with_arm_1/
        # _2 must exist in the loaded SRDF -- restart move_group after an SRDF
        # update. Arm-only groups (no 'gantry' in the name) still work: _on_execute
        # then plans the 6 arm joints only.
        p('arm_groups', ['gantry_1_with_arm_1', 'gantry_1_with_arm_2',
                         'gantry_2_with_arm_1', 'gantry_2_with_arm_2'])
        p('plan_time', 5.0); p('plan_attempts', 10)
        p('vel_scale', 0.1); p('acc_scale', 0.1); p('joint_tol', 0.01)
        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.coll_r = float(g('collision_radius'))
        self.target_r = float(g('target_radius'))
        self.target_label = str(g('target_label')).strip().lower()
        self.target_index = int(g('target_index'))
        self.reach_tol = float(g('reach_tol'))
        max_z = float(g('reach_max_z'))
        gamma, levels = float(g('diffusion_gamma')), int(g('diffusion_levels'))
        self.pool_r = float(g('energy_pool_radius'))
        self.w = {k: float(g('w_' + k)) for k in
                  ('gantry_lin', 'gantry_rot', 'arm', 'dist', 'hold', 'manip')}
        self.ref = {k: float(g('ref_' + k)) for k in
                    ('gantry_lin', 'gantry_rot', 'arm', 'dist', 'hold', 'manip')}
        self._joints = {}             # live joint name -> position
        self._winner = None           # (arm dict, grasp node idx) for execution

        self.arms = []
        groups = list(g('arm_groups'))
        for k, (lab, path) in enumerate(zip(list(g('arm_labels')),
                                            list(g('arm_models')))):
            try:
                gng = GNG.load(path)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{lab}: cannot load {path}: {e}')
                continue
            td = gng.task_dim
            R, q = gng.W[:, :td].astype(np.float64), gng.W[:, td:].astype(np.float64)
            manip, hold, jnames = self._load_stats(path, len(R))
            keep = R[:, 2] <= max_z   # crop above-gantry nodes; remap edges
            remap = {int(o): i for i, o in enumerate(np.where(keep)[0])}
            edges = [(remap[i], remap[j]) for i, j in (tuple(e) for e in gng._edges)
                     if i in remap and j in remap]
            R, q, manip, hold = R[keep], q[keep], manip[keep], hold[keep]
            S = _diffusion_matrix(len(R), edges, gamma, levels)
            self.arms.append(dict(lab=lab, R=R, edges=edges, S=S, q=q,
                                  manip=manip, hold=hold, jnames=jnames,
                                  group=groups[k] if k < len(groups) else ''))
            self.get_logger().info(f'{lab}: {len(R)} nodes, {len(edges)} edges')
        self.plan_time = float(g('plan_time'))
        self.plan_attempts = int(g('plan_attempts'))
        self.vel_scale, self.acc_scale = float(g('vel_scale')), float(g('acc_scale'))
        self.joint_tol = float(g('joint_tol'))
        self.move_cli = ActionClient(self, MoveGroup, 'move_action')

        self.env_pts = np.empty((0, 3))
        self.poses = np.empty((0, 3))
        self.labels = []              # [(label_lower, marker_xyz)] class-label path
        self.objn = ObjnLocalizer(self, world_frame=self.world_frame)
        obj_topic = g('objects_topic')
        self.create_subscription(MarkerArray, g('env_markers_topic'), self._on_env, 1)
        self.create_subscription(PoseArray, obj_topic, self._on_objects, 1)
        self.create_subscription(MarkerArray, obj_topic + '/markers',
                                 self._on_obj_markers, 1)
        self.create_subscription(String, '/reach_fusion/set_target',
                                 self._on_set_target, 1)
        self.create_subscription(JointState, g('joint_states_topic'),
                                 self._on_joints, 10)
        self.create_subscription(Empty, '/reach_fusion/execute',
                                 self._on_execute, 1)
        self.pub = self.create_publisher(MarkerArray, '/reach_fusion/markers', 1)
        self.create_timer(1.0 / max(float(g('publish_hz')), 0.5), self._tick)

    @staticmethod
    def _load_stats(path, n):
        """(manip, hold, joint_names) from <model>_stats.npz; zeros/None if absent."""
        try:
            s = np.load((path[:-4] if path.endswith('.npz') else path) + '_stats.npz')
        except Exception:  # noqa: BLE001
            return np.zeros(n), np.zeros(n), None
        manip = s['manip'] if 'manip' in s else np.zeros(n)
        hold = s['hold'] if 'hold' in s else np.zeros(n)
        jn = [str(x) for x in s['joint_names']] if 'joint_names' in s else None
        return manip, hold, (jn or None)

    def _on_joints(self, msg):
        self._joints.update(dict(zip(msg.name, msg.position)))

    def _current_q(self, jnames):
        if not jnames:
            return None
        try:
            return np.array([self._joints[n] for n in jnames])
        except KeyError:
            return None

    def _arm_energy(self, arm, target, danger):
        """(min J, grasp node idx) over graspable candidates, or None.

        Candidates = non-danger reach nodes within pool_r of the target (the arm
        can reach & grasp there collision-free). J = executor's calibrated energy:
        gantry+arm travel from current state + task gap + gravity hold - manip."""
        if target is None:
            return None
        dR = np.linalg.norm(arm['R'] - target, axis=1)
        cand = np.where(~danger & (dR <= self.pool_r))[0]
        if len(cand) == 0:
            return None
        cur = self._current_q(arm['jnames'])
        w, ref = self.w, self.ref
        best = None
        for i in cand:
            j = (w['dist'] * dR[i] / ref['dist']
                 + w['hold'] * arm['hold'][i] / ref['hold']
                 - w['manip'] * arm['manip'][i] / ref['manip'])
            if cur is not None:
                q = arm['q'][i]
                j += (w['gantry_lin'] * abs(q[0] - cur[0]) / ref['gantry_lin']
                      + w['gantry_rot'] * abs(q[1] - cur[1]) / ref['gantry_rot']
                      + w['arm'] * np.abs(q[2:] - cur[2:]).sum() / ref['arm'])
            if best is None or j < best[0]:
                best = (float(j), int(i))
        return best

    def _on_env(self, msg):
        if msg.markers:
            self.env_pts = np.array([[p.x, p.y, p.z] for p in msg.markers[0].points])

    def _on_objects(self, msg):
        self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                               for p in msg.poses]) if msg.poses else np.empty((0, 3))

    def _on_obj_markers(self, msg):
        lab = [(m.text.strip().lower(),
                np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))
               for m in msg.markers if m.text]
        if lab:
            self.labels = lab

    def _on_set_target(self, msg):
        v = msg.data.strip()
        if v.lstrip('-').isdigit():
            self.target_index, self.target_label = int(v), ''
        else:
            self.target_label = v.lower()
        self.get_logger().info(f'target -> {v}')

    def _resolve_target(self):
        """Target centroid by obj_N (ground truth) / class substring / index."""
        if len(self.poses) == 0:
            return None
        lbl = self.target_label
        if lbl:
            if _OBJN_RE.match(lbl):
                return self.objn.find(lbl, self.poses)
            for lab, mxyz in self.labels:      # class substring (seg -- unreliable)
                if lbl in lab:
                    return self.poses[int(np.argmin(
                        np.linalg.norm(self.poses - mxyz, axis=1)))]
            return None
        return (self.poses[self.target_index]
                if 0 <= self.target_index < len(self.poses) else None)

    def _classify(self, R, S, target):
        """(danger, cfree) masks: danger near obstacles, cfree = corridor to target."""
        from scipy.spatial import cKDTree
        n = len(R)
        if len(self.env_pts) == 0:
            return np.zeros(n, bool), np.zeros(n, bool)
        obst = (self.env_pts[np.linalg.norm(self.env_pts - target, axis=1) > self.target_r]
                if target is not None else self.env_pts)
        danger = (cKDTree(obst).query(R)[0] < self.coll_r
                  if len(obst) else np.zeros(n, bool))
        cfree = np.zeros(n, bool)
        if target is not None:
            dR = np.linalg.norm(R - target, axis=1)
            it = int(np.argmin(dR))
            if dR[it] <= self.reach_tol:
                tgt = np.zeros(n); tgt[it] = 1.0
                pot, dang = S @ tgt, S @ danger.astype(float)
                pot = pot / pot.max() if pot.max() > 0 else pot
                dang = dang / dang.max() if dang.max() > 0 else dang
                cfree = (pot - dang > 0.0) & ~danger
        return danger, cfree

    def _sphere_list(self, lab, ns, mid, size, color, now):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.world_frame, now
        m.ns, m.id, m.type, m.action = f'{lab}_{ns}', mid, Marker.SPHERE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = color
        return m

    def _tick(self):
        if not self.arms:
            return
        markers, mid, now = [], 0, self.get_clock().now().to_msg()
        target = self._resolve_target()
        energies = {}                 # lab -> (J, grasp_idx)
        for a in self.arms:
            lab, R, edges, S = a['lab'], a['R'], a['edges'], a['S']
            danger, cfree = self._classify(R, S, target)
            e = self._arm_energy(a, target, danger)
            if e is not None:
                energies[lab] = e
            cr, cg, cb = _ARM_COLORS.get(lab, (0.6, 0.6, 0.6))
            free = self._sphere_list(lab, 'free', mid, 0.022,
                                     ColorRGBA(r=cr, g=cg, b=cb, a=1.0), now)
            dang = self._sphere_list(lab, 'danger', mid + 1, 0.03,
                                     ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0), now)
            cfr = self._sphere_list(lab, 'cfree', mid + 2, 0.032,
                                    ColorRGBA(r=0.1, g=0.95, b=0.95, a=1.0), now)
            for i, w in enumerate(R):
                pt = Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                (dang if danger[i] else cfr if cfree[i] else free).points.append(pt)
            net = self._sphere_list(lab, 'edges', mid + 3, 0.004,
                                    ColorRGBA(r=cr, g=cg, b=cb, a=0.3), now)
            net.type, net.scale.x = Marker.LINE_LIST, 0.004
            for i, j in edges:
                net.points += [Point(x=float(R[i][0]), y=float(R[i][1]), z=float(R[i][2])),
                               Point(x=float(R[j][0]), y=float(R[j][1]), z=float(R[j][2]))]
            markers += [free, dang, cfr, net]
            mid += 4

        winner = min(energies, key=lambda k: energies[k][0]) if energies else None
        if target is not None:
            markers.append(self._point_marker(
                'target_obj', mid, target, 0.10, ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9), now))
        if winner is not None:                 # highlight the winning arm's grasp node
            gi = energies[winner][1]
            arm = next(a for a in self.arms if a['lab'] == winner)
            self._winner = (arm, gi)           # latched for /reach_fusion/execute
            markers.append(self._point_marker(
                'winner', mid + 1, arm['R'][gi], 0.07,
                ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0), now))
            self._log_energy(winner, energies)
        else:
            self._winner = None
        self.pub.publish(MarkerArray(markers=markers))

    # ---- 3a execution: winner grasp node q -> MoveGroup joint goal -----------
    def _on_execute(self, _msg):
        if self._winner is None:
            self.get_logger().warn('execute: no winning arm yet')
            return
        if not self.move_cli.server_is_ready():
            self.get_logger().error('execute: move_action server not ready')
            return
        arm, gi = self._winner
        # arm-only groups (no 'gantry' in the name) plan just the 6 arm joints;
        # q/jnames lead with the 2 gantry DOFs, so drop them for those groups.
        jn, q = arm['jnames'], arm['q'][gi]
        if 'gantry' not in arm['group']:
            jn, q = jn[2:], q[2:]
        req = MotionPlanRequest(group_name=arm['group'])
        con = Constraints()
        for n, v in zip(jn, q):
            con.joint_constraints.append(JointConstraint(
                joint_name=n, position=float(v), tolerance_above=self.joint_tol,
                tolerance_below=self.joint_tol, weight=1.0))
        req.goal_constraints = [con]
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False))
        self.get_logger().info(f"executing {arm['lab']} -> {arm['group']} (grasp node {gi})")
        self.move_cli.send_goal_async(goal).add_done_callback(
            lambda f, a=arm: self._on_goal_resp(f, a))

    def _on_goal_resp(self, fut, arm):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"{arm['lab']}: MoveGroup goal rejected")
            return
        gh.get_result_async().add_done_callback(
            lambda f, a=arm: self._on_exec_result(f, a))

    def _on_exec_result(self, fut, arm):
        code = fut.result().result.error_code.val
        msg = 'OK' if code == 1 else f'FAILED (code {code})'
        self.get_logger().info(f"{arm['lab']} execute: {msg}")

    def _point_marker(self, ns, mid, xyz, size, color, now):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.world_frame, now
        m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = color
        m.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        return m

    def _log_energy(self, winner, energies):
        rank = sorted(energies.items(), key=lambda kv: kv[1][0])
        if rank != getattr(self, '_last_rank', None):
            self._last_rank = rank
            txt = '  '.join(f'{k}={v[0]:.2f}{"*" if k == winner else ""}'
                            for k, v in rank)
            self.get_logger().info(f'energy J (collision-free): {txt}  -> WIN {winner}')


def main():
    rclpy.init()
    node = ReachFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
