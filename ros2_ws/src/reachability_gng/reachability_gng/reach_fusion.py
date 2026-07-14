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

import json
import re
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG
from reachability_gng.objn_localizer import ObjnLocalizer
from reachability_gng.seed_ik import build_ik_request

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
        p('target_index', -1)         # -1 -> no default target (idle until one is set)
        p('reach_tol', 0.20)          # arm reaches target if nearest node this close
        p('diffusion_gamma', 0.5)
        p('diffusion_levels', 4)
        p('reach_max_z', 2.05)        # drop reach nodes above the ceiling gantry
        p('publish_hz', 0.5)
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
        p('arm_ee_frames', ['t1_a1_tool_frame', 't1_a2_tool_frame',
                            't2_a1_tool_frame', 't2_a2_tool_frame'])
        p('grasp_orientation', [1.0, 0.0, 0.0, 0.0])   # top-down grasp (xyzw)
        # IK is kinematics-only (fast, ~ms): collision-aware IK vs the 787 GNG
        # spheres is slow (~3 s/call) AND rejects valid stand-off configs; the
        # PLAN (MoveGroup) is collision-aware and is the real collision arbiter.
        p('ik_avoid_collisions', False)
        # KDL does random-restart IK until this budget; 0.05 s under a contended
        # move_group yields only 1-2 restarts, so a marginal-reach target (obj_2,
        # workspace edge) solves only ~10-13/16 orientations -- flaky, and a whole
        # sweep can miss. The per-call WALL time is dominated by move_group service
        # latency (~seconds under contention), not this budget, so raising it to
        # 0.3 s barely changes wall time but gives KDL many more restarts.
        p('ik_timeout', 0.3)
        # APPROACH mode: aim the EE this far ABOVE the object so it stops over the
        # target without touching it (object stays a GNG obstacle, not carved).
        # Must clear the gripper length + the object's GNG collision sphere, else
        # the approach config sits on the collision boundary (IK -31 / plan fails).
        p('grasp_standoff', 0.20)
        p('approach_tol', 0.12)         # EE this close to the stand-off = success
        # MoveGroup returns code=1 as soon as it finishes STREAMING the trajectory
        # to Isaac's topic_based_ros2_control bridge, but Isaac physics (finite PD
        # gains) then needs a few seconds to actually drive the arm to the last
        # setpoint. Verifying the EE immediately reads the arm mid-motion -> false
        # "APPROACH FAILED" (measured 0.53 m while still settling; 0.013 m once
        # settled). So after code=1, poll until the actual joints match the
        # commanded IK config within settle_tol (or settle_timeout elapses), then
        # measure the EE. Gantry tracks near-instantly; only the arm needs this.
        p('settle_tol', 0.03)           # max |cmd-act| joint residual = settled (rad)
        # a large arm+gantry move (e.g. arm4 swinging across, after -4 retries)
        # can still be 1+ rad out at 8 s under sim contention yet reach 0.01 m of
        # the stand-off shortly after -- 8 s reported a false failure. 20 s covers
        # the worst observed settle; a genuine miss just takes that long to report.
        p('settle_timeout', 20.0)       # give up waiting, verify anyway (s)
        p('settle_period', 0.5)         # poll interval (s)
        # try the vertical grasp at several yaws, then tilted approaches -- IK
        # takes the first that solves (many -31 are just an unreachable yaw).
        p('grasp_yaw_samples', 8)
        p('grasp_tilt_samples', 2)
        p('grasp_tilt_max', 30.0)       # degrees off vertical
        p('grasp_tilt_azimuths', 4)
        p('plan_time', 8.0); p('plan_attempts', 20)
        p('vel_scale', 0.1); p('acc_scale', 0.1); p('joint_tol', 0.01)
        # -4 (CONTROL_FAILED) / -3 (env change) are transient sim mis-fires that
        # the executor also retries; re-attempt the IK->plan->execute this often.
        p('max_execute_retries', 3)
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
        self._ranked = []             # [(arm dict, grasp idx)] by energy, best first

        self.arms = []
        groups = list(g('arm_groups'))
        ee_frames = list(g('arm_ee_frames'))
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
            # R/edges are static once the GNG model is loaded -- build each node's
            # Point and the whole edges LINE_LIST marker ONCE here, not per tick.
            # Reconstructing ~2450 node Points + ~12800*2 edge Points from scratch
            # every 0.5s tick was blocking reach_fusion's single-threaded executor
            # for 10s+ under host CPU contention, delaying the /compute_ik future's
            # done-callback by the same amount (measured: 50s/tick -> IK "stuck" 100s).
            cr, cg, cb = _ARM_COLORS.get(lab, (0.6, 0.6, 0.6))
            pts = [Point(x=float(w[0]), y=float(w[1]), z=float(w[2])) for w in R]
            net = self._sphere_list(lab, 'edges', k * 4 + 3, 0.004,
                                    ColorRGBA(r=cr, g=cg, b=cb, a=0.3),
                                    self.get_clock().now().to_msg())
            net.type, net.scale.x = Marker.LINE_LIST, 0.004
            for i, j in edges:
                net.points += [pts[i], pts[j]]
            self.arms.append(dict(lab=lab, R=R, edges=edges, S=S, q=q,
                                  manip=manip, hold=hold, jnames=jnames, pts=pts, net=net,
                                  group=groups[k] if k < len(groups) else '',
                                  ee_frame=ee_frames[k] if k < len(ee_frames) else ''))
            self.get_logger().info(f'{lab}: {len(R)} nodes, {len(edges)} edges')
        self.plan_time = float(g('plan_time'))
        self.plan_attempts = int(g('plan_attempts'))
        self.vel_scale, self.acc_scale = float(g('vel_scale')), float(g('acc_scale'))
        self.joint_tol = float(g('joint_tol'))
        self.grasp_ori = [float(v) for v in g('grasp_orientation')]
        self.ik_avoid = bool(g('ik_avoid_collisions'))
        self.ik_timeout = float(g('ik_timeout'))
        self.standoff = float(g('grasp_standoff'))
        self.approach_tol = float(g('approach_tol'))
        self.settle_tol = float(g('settle_tol'))
        self.settle_timeout = float(g('settle_timeout'))
        self.settle_period = float(g('settle_period'))
        self._settle_timer = None
        self.yaw_samples = int(g('grasp_yaw_samples'))
        self.tilt_samples = int(g('grasp_tilt_samples'))
        self.tilt_max = float(g('grasp_tilt_max'))
        self.tilt_azimuths = int(g('grasp_tilt_azimuths'))
        self.max_retries = int(g('max_execute_retries'))
        self._retries = 0
        self._target = None           # latest resolved target xyz (for IK)
        self.move_cli = ActionClient(self, MoveGroup, 'move_action')
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.hold_pub = self.create_publisher(Bool, '/gng_collision/hold', 1)

        self.env_pts = np.empty((0, 3))
        self.poses = np.empty((0, 3))
        self.labels = []              # [(label_lower, marker_xyz)] class-label path
        self._tracks_xyz = {}         # stable track id -> centroid (identity handle)
        self._tracks_lbl = {}         # stable track id -> class label (for logging)
        self.objn = ObjnLocalizer(self, world_frame=self.world_frame)
        obj_topic = g('objects_topic')
        self.create_subscription(MarkerArray, g('env_markers_topic'), self._on_env, 1)
        self.create_subscription(PoseArray, obj_topic, self._on_objects, 1)
        self.create_subscription(MarkerArray, obj_topic + '/markers',
                                 self._on_obj_markers, 1)
        # Stable-identity handles from object_localizer (persistent track id per
        # physical object); targeting a track id is immune to the per-frame label
        # flicker that made obj_N nondeterministic. Latched to match the publisher.
        tq = QoSProfile(depth=1)
        tq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._tracks_topic = obj_topic + '/tracks'
        self.create_subscription(String, self._tracks_topic, self._on_tracks, tq)
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

    def _on_tracks(self, msg):
        try:
            arr = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._tracks_xyz = {int(t['tid']): np.array(
            [float(t['x']), float(t['y']), float(t['z'])]) for t in arr}
        self._tracks_lbl = {int(t['tid']): str(t.get('label', '')) for t in arr}

    def _on_set_target(self, msg):
        v = msg.data.strip()
        if v.lstrip('-').isdigit():
            self.target_index, self.target_label = int(v), ''
        else:
            self.target_label = v.lower()
        self._last_winner = None       # re-log the arm choice once for the new target
        # log the object by name, not the raw handle: for a #<tid> track resolve
        # the class label AND the nearest deterministic obj_N marker, so the class
        # target can be cross-checked against the ground-truth object id.
        name = v
        if v.startswith('#'):
            try:
                tid = int(v[1:])
                name = self._tracks_lbl.get(tid) or v
                objn = self._nearest_objn(self._tracks_xyz.get(tid))
                if objn:
                    name = f'{name} ({objn})'
            except ValueError:
                name = v
        self.get_logger().info(f'target -> {name}')

    def _nearest_objn(self, xyz):
        """Deterministic obj_N marker (object_localizer) closest to xyz, or ''."""
        if xyz is None or not self.labels:
            return ''
        objns = [(lab, mxyz) for lab, mxyz in self.labels if _OBJN_RE.match(lab)]
        if not objns:
            return ''
        return min(objns, key=lambda lm: np.linalg.norm(lm[1] - xyz))[0]

    def _resolve_target(self):
        """Target centroid by stable track id (#<tid>) / obj_N / class / index.

        A #<tid> handle is the PREFERRED, source-agnostic target: it names a
        persistent spatial track (object_localizer), so it follows one physical
        object even as the per-frame class label flickers. The obj_N / class /
        index paths are kept for backward compatibility.
        """
        lbl = self.target_label
        if lbl.startswith('#'):
            try:
                return self._tracks_xyz.get(int(lbl[1:]))
            except ValueError:
                return None
        if len(self.poses) == 0:
            return None
        if lbl:
            # Match the object-marker labels first: under seg_source:=isaac,
            # object_localizer publishes the ground-truth obj_N prim name as each
            # marker's text at the correct centroid. ObjnLocalizer's reverse-
            # projection (world centroid -> pixel -> instance id) was mislabelling
            # obj_N -- it read a NEIGHBOURING instance's pixels and returned the
            # wrong object (observed: obj_2 -> obj_1's position), so reach_fusion
            # aimed at the wrong target and its IK failed. Exact match for obj_N,
            # substring for a class label. Projection stays as a fallback only.
            objn = bool(_OBJN_RE.match(lbl))
            for lab, mxyz in self.labels:
                if (lbl == lab) if objn else (lbl in lab):
                    return self.poses[int(np.argmin(
                        np.linalg.norm(self.poses - mxyz, axis=1)))]
            if objn:                           # no marker label -> projection GT
                return self.objn.find(lbl, self.poses)
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
        # Guard: a #<tid> handle is only consistent if ONE object_localizer
        # publishes the tracks. Two instances (e.g. a stale + a fresh launch)
        # number tracks independently, so reach_fusion's #tid can point at a
        # DIFFERENT object than target_cli's -> wrong pick. Surface it loudly
        # rather than silently target the wrong object.
        npub = self.count_publishers(self._tracks_topic)
        if npub > 1:
            self.get_logger().error(
                f'{npub} publishers on {self._tracks_topic} -- MULTIPLE '
                'object_localizer instances running; #tid handles are '
                'INCONSISTENT (target_cli and reach_fusion may disagree). Kill '
                'the stale topo_fusion/object_localizer, keep exactly one.',
                throttle_duration_sec=5.0)
        markers, mid, now = [], 0, self.get_clock().now().to_msg()
        target = self._resolve_target()
        self._target = target         # latched for IK on /reach_fusion/execute
        energies = {}                 # lab -> (J, grasp_idx)
        for a in self.arms:
            lab, R, S = a['lab'], a['R'], a['S']
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
            pts = a['pts']
            for i in range(len(R)):
                (dang if danger[i] else cfr if cfree[i] else free).points.append(pts[i])
            net = a['net']              # static geometry, cached at load time
            net.header.stamp = now
            markers += [free, dang, cfr, net]
            mid += 4

        # arms ranked by energy J (winner first); execution tries them in order,
        # falling back to the next when the winner's exact-pose IK has no solution
        # (the position-only reach map can flag an arm reachable that isn't).
        by_j = sorted(energies.items(), key=lambda kv: kv[1][0])
        self._ranked = [(next(a for a in self.arms if a['lab'] == lab), e[1])
                        for lab, e in by_j]
        winner = by_j[0][0] if by_j else None
        if target is not None:
            markers.append(self._point_marker(
                'target_obj', mid, target, 0.10, ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9), now))
        if winner is not None:                 # highlight the winning arm's grasp node
            arm, gi = self._ranked[0]
            markers.append(self._point_marker(
                'winner', mid + 1, arm['R'][gi], 0.07,
                ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0), now))
            self._log_energy(winner, energies)
        self.pub.publish(MarkerArray(markers=markers))

    # ---- 3a execution: winner node q = IK seed -> IK to exact target -> plan --
    @staticmethod
    def _quat_mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz)

    def _grasp_orientations(self):
        """Grasp orientations to try (best-first): vertical at each yaw, then
        tilted fallbacks for spots only reachable at an angle."""
        base = tuple(self.grasp_ori)
        out = []
        for i in range(max(1, self.yaw_samples)):
            yaw = 2.0 * np.pi * i / max(1, self.yaw_samples)
            out.append(self._quat_mul(
                (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)), base))
        if self.tilt_samples > 0 and self.tilt_max > 0:
            for k in range(1, self.tilt_samples + 1):
                t = np.radians(self.tilt_max * k / self.tilt_samples)
                st, ct = np.sin(t / 2), np.cos(t / 2)
                for a in range(max(1, self.tilt_azimuths)):
                    phi = 2.0 * np.pi * a / max(1, self.tilt_azimuths)
                    out.append(self._quat_mul(
                        (np.cos(phi) * st, np.sin(phi) * st, 0.0, ct), base))
        return out

    def _on_execute(self, _msg):
        self._retries = 0
        self._attempt()

    def _hold(self, on):
        self.hold_pub.publish(Bool(data=bool(on)))   # freeze GNG collision scene

    def _attempt(self):
        if not self._ranked or self._target is None:
            self.get_logger().warn('execute: no reachable arm / target yet')
            return
        if not self.ik_cli.service_is_ready():
            self.get_logger().error('execute: /compute_ik not ready')
            return
        self._hold(True)                          # scene stays put through execution
        self._exec_ranked = list(self._ranked)   # freeze arm order for this attempt
        self._exec_target = self._target.copy()
        self._oris = self._grasp_orientations()
        self._ai = 0
        self._t_attempt = time.monotonic()
        self._ik_calls = 0
        self._ik_time = 0.0
        self.get_logger().info(
            f"=== TIMING: attempt start, {len(self._exec_ranked)} arm(s) ranked, "
            f"{len(self._oris)} orientations/arm ===")
        self._try_arm()

    def _approach_failed(self, reason):
        dt = time.monotonic() - self._t_attempt if hasattr(self, '_t_attempt') else -1.0
        self.get_logger().error(f"=== APPROACH FAILED: {reason} ({dt:.3f}s total) ===")
        self._hold(False)

    def _try_arm(self):
        if self._ai >= len(self._exec_ranked):
            self._approach_failed('no arm can reach the target (IK/plan)')
            return
        self._exec_arm, self._exec_gi = self._exec_ranked[self._ai]
        arm, tgt = self._exec_arm, self._exec_target
        # Seed candidates (tried in order until one solves the orientation sweep):
        # the map node NEAREST the target, then the energy-ranked grasp node gi.
        # A GNG node's stored q is a topological AVERAGE, so which node seeds KDL
        # into a solution is target-dependent -- the nearest node solves some
        # targets the energy node misses (obj_2 @ 1.772) and vice-versa (@ 3.051).
        # Trying both recovers IK on targets that a single fixed seed drops.
        near = int(np.argmin(np.linalg.norm(arm['R'] - tgt, axis=1)))
        self._seed_nodes = [near] + ([self._exec_gi] if self._exec_gi != near else [])
        self._seed_i = 0
        self._oi = 0
        self.get_logger().info(
            f"approach {self._exec_arm['lab']} -> target "
            f"(arm {self._ai + 1}/{len(self._exec_ranked)}, {len(self._oris)} oris, "
            f"{len(self._seed_nodes)} seed(s))")
        self._try_ik()

    def _try_ik(self):
        arm, tgt = self._exec_arm, self._exec_target
        si = self._seed_nodes[self._seed_i]      # current seed-node candidate
        if self._oi == 0:
            self.get_logger().info(
                f"=== DIAG seed {self._seed_i + 1}/{len(self._seed_nodes)}: "
                f"tgt=({tgt[0]:+.3f},{tgt[1]:+.3f},{tgt[2]:+.3f}) "
                f"seed_node=({arm['R'][si][0]:+.3f},{arm['R'][si][1]:+.3f},"
                f"{arm['R'][si][2]:+.3f}) node_dist={np.linalg.norm(arm['R'][si]-tgt):.3f} ===")
        # arm-only groups plan only the 6 arm joints; q leads with 2 gantry DOFs.
        seed_names = arm['jnames'] if 'gantry' in arm['group'] else arm['jnames'][2:]
        seed_pos = arm['q'][si] if 'gantry' in arm['group'] else arm['q'][si][2:]
        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.pose.position = Point(x=float(tgt[0]), y=float(tgt[1]),
                                 z=float(tgt[2] + self.standoff))  # stand-off above
        ox, oy, oz, ow = self._oris[self._oi]
        ps.pose.orientation.x, ps.pose.orientation.y = float(ox), float(oy)
        ps.pose.orientation.z, ps.pose.orientation.w = float(oz), float(ow)
        req = build_ik_request(arm['group'], arm['ee_frame'], ps,
                               seed_names, seed_pos, timeout_s=self.ik_timeout,
                               avoid_collisions=self.ik_avoid)
        self._ik_calls += 1
        self._t_ik = time.monotonic()
        self.ik_cli.call_async(req).add_done_callback(self._on_ik)

    def _on_ik(self, fut):
        arm = self._exec_arm
        dt = time.monotonic() - self._t_ik
        self._ik_time += dt
        res = fut.result()
        ok = res is not None and res.error_code.val == 1
        code = res.error_code.val if res is not None else None
        self.get_logger().info(
            f"=== TIMING: IK #{self._ik_calls} {arm['lab']} ori{self._oi} "
            f"({dt:.3f}s) code={code} {'OK' if ok else ''} ===")
        if not ok:
            self._oi += 1
            if self._oi < len(self._oris):       # try the next orientation
                self._try_ik()
            elif self._seed_i + 1 < len(self._seed_nodes):   # next seed candidate
                self._seed_i += 1
                self._oi = 0
                self.get_logger().warn(
                    f"{arm['lab']}: no IK over {len(self._oris)} oris on seed "
                    f"{self._seed_i}/{len(self._seed_nodes)} -> next seed")
                self._try_ik()
            else:                                # seeds exhausted -> next-best arm
                self.get_logger().warn(
                    f"{arm['lab']}: no IK over {len(self._oris)} oris x "
                    f"{len(self._seed_nodes)} seeds -> next arm")
                self._ai += 1
                self._try_arm()
            return
        self.get_logger().info(
            f"=== TIMING: IK solved after {self._ik_calls} call(s), "
            f"{self._ik_time:.3f}s IK time, "
            f"{time.monotonic() - self._t_attempt:.3f}s since attempt start ===")
        if not self.move_cli.server_is_ready():
            self.get_logger().error('execute: move_action server not ready')
            return
        self._t_plan = time.monotonic()
        js = res.solution.joint_state
        grp = set(arm['jnames'] if 'gantry' in arm['group'] else arm['jnames'][2:])
        # stash the commanded (IK-solution) group config for the post-exec diag:
        # lets _verify_approach tell "gantry/arm never reached goal" (controller
        # didn't track, code=1 anyway) apart from "object drifted / FK mismatch".
        self._ik_sol = {n: float(v) for n, v in zip(js.name, js.position) if n in grp}
        req = MotionPlanRequest(group_name=arm['group'])
        con = Constraints()
        for n, v in zip(js.name, js.position):
            if n in grp:
                con.joint_constraints.append(JointConstraint(
                    joint_name=n, position=float(v), tolerance_above=self.joint_tol,
                    tolerance_below=self.joint_tol, weight=1.0))
        req.goal_constraints = [con]
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False))
        self.get_logger().info(f"executing {arm['lab']} to IK solution")
        self.move_cli.send_goal_async(goal).add_done_callback(
            lambda f, a=arm: self._on_goal_resp(f, a))

    def _on_goal_resp(self, fut, arm):
        gh = fut.result()
        self.get_logger().info(
            f"=== TIMING: MoveGroup goal {'accepted' if gh and gh.accepted else 'REJECTED'} "
            f"after {time.monotonic() - self._t_plan:.3f}s ===")
        if gh is None or not gh.accepted:
            self._approach_failed(f"{arm['lab']} MoveGroup goal rejected")
            return
        gh.get_result_async().add_done_callback(
            lambda f, a=arm: self._on_exec_result(f, a))

    def _on_exec_result(self, fut, arm):
        code = fut.result().result.error_code.val
        plan_exec_dt = time.monotonic() - self._t_plan
        total_dt = time.monotonic() - self._t_attempt
        self.get_logger().info(
            f"=== TIMING: plan+execute took {plan_exec_dt:.3f}s, "
            f"total attempt {total_dt:.3f}s, code={code} ===")
        if code == 1:
            self.get_logger().info(f"{arm['lab']} execute: OK; waiting for arm to settle")
            self._settle_arm = arm
            self._settle_t0 = time.monotonic()
            if self._settle_timer is not None:
                self._settle_timer.cancel()
            self._settle_timer = self.create_timer(self.settle_period, self._settle_check)
        elif code in (-3, -4, 99999) and self._retries < self.max_retries:
            # -3 env change, -4 CONTROL_FAILED, 99999 MoveItErrorCodes.FAILURE
            # (generic) are all transient sim mis-fires under host contention --
            # observed recovering on retry (e.g. -4 -> 99999 -> code=1).
            self._retries += 1
            self.get_logger().warn(
                f"{arm['lab']} execute code {code} (transient) -> "
                f"retry {self._retries}/{self.max_retries}")
            self._attempt()               # keep the scene held across the retry
        elif code in (-1, -2):            # plan invalid/collision -> next-best arm
            self.get_logger().warn(
                f"{arm['lab']} plan code {code} (path collides) -> next arm")
            self._ai += 1
            self._try_arm()               # hold stays on
        else:
            self._approach_failed(f"{arm['lab']} execute code {code}")

    def _settle_check(self):
        """Poll until the arm reaches the commanded IK config, then verify.

        Isaac's physics lags MoveGroup's code=1 by a few seconds (see settle_tol
        comment); measuring the EE before the arm settles gives a false failure.
        Waits until max |cmd-act| joint residual <= settle_tol or settle_timeout."""
        arm = self._settle_arm
        sol = getattr(self, '_ik_sol', {}) or {}
        max_d = 0.0
        for n, v in sol.items():
            act = self._joints.get(n)
            if act is not None:
                max_d = max(max_d, abs(act - v))
        elapsed = time.monotonic() - self._settle_t0
        if max_d <= self.settle_tol or elapsed >= self.settle_timeout:
            self._settle_timer.cancel()
            self._settle_timer = None
            settled = max_d <= self.settle_tol
            self.get_logger().info(
                f"=== settled={settled}: max joint residual {max_d:.3f} rad "
                f"after {elapsed:.1f}s ===")
            self._hold(False)
            self._verify_approach(arm)

    def _verify_approach(self, arm):
        """Log APPROACH SUCCESS once the REAL EE (from TF) is above the object."""
        goal = self._exec_target + np.array([0.0, 0.0, self.standoff])
        try:
            t = self.objn.tf.lookup_transform(
                self.world_frame, arm['ee_frame'],
                rclpy.time.Time()).transform.translation
        except Exception:  # noqa: BLE001
            self.get_logger().info(
                f"=== APPROACH DONE: {arm['lab']} (EE pose unavailable to verify) ===")
            return
        err = float(np.linalg.norm(np.array([t.x, t.y, t.z]) - goal))
        total_dt = time.monotonic() - self._t_attempt
        # DIAG: commanded (IK solution) vs actual joints -- large residual on a
        # gantry joint => controller reported success without tracking the goal.
        sol = getattr(self, '_ik_sol', {}) or {}
        diffs = []
        for n in sorted(sol):
            act = self._joints.get(n)
            if act is not None:
                diffs.append(f"{n}={sol[n]:+.3f}->{act:+.3f}(d{act - sol[n]:+.3f})")
        self.get_logger().info(
            f"=== DIAG EE: want=({goal[0]:+.3f},{goal[1]:+.3f},{goal[2]:+.3f}) "
            f"got=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f}) err={err:.3f} ===")
        self.get_logger().info(f"=== DIAG joints cmd->act: {'  '.join(diffs)} ===")
        if err <= self.approach_tol:
            self.get_logger().info(
                f"=== APPROACH SUCCESS: {arm['lab']} is above the object "
                f"(EE {err:.3f} m from the stand-off, {total_dt:.3f}s total) ===")
        else:
            self.get_logger().error(
                f"=== APPROACH FAILED: {arm['lab']} executed but EE is "
                f"{err:.2f} m from the stand-off (not above the object) ===")

    def _point_marker(self, ns, mid, xyz, size, color, now):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.world_frame, now
        m.ns, m.id, m.type, m.action = ns, mid, Marker.SPHERE, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = color
        m.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        return m

    def _log_energy(self, winner, energies):
        # Log once, and re-log only when the WINNING arm changes -- dedup on the
        # arm label alone so per-tick J jitter (e.g. 29.45 -> 29.44) stays quiet.
        rank = sorted(energies.items(), key=lambda kv: kv[1][0])
        if winner != getattr(self, '_last_winner', None):
            self._last_winner = winner
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
