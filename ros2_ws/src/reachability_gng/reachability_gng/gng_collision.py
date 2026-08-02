"""Feed the GNG environment map into MoveIt as collision spheres (no octomap).

Replaces MoveIt's octomap with the topological map: takes env_gng's nodes
(/topo_map/markers), downsamples them to a coarse grid (lighter than octomap's
dense voxels), and publishes them as one CollisionObject of spheres to
/planning_scene. MoveIt then plans avoiding the topological obstacles.

During APPROACH the target object stays an obstacle (not carved), so the arm
does not touch it; a carve region (for a later GRASP mode) can be excluded via
/gng_collision/carve (PointStamped: xyz of the target, carved within
carve_radius). Publish an empty/NaN point to clear the carve.

    /topo_map/markers  -> (downsample, optional carve) -> /planning_scene
"""
from __future__ import annotations

import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PointStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from reachability_gng.env_gng import arm_positions, filter_by_positions


class GngCollision(Node):
    def __init__(self):
        super().__init__('gng_collision')
        p = self.declare_parameter
        p('env_markers_topic', '/topo_map/markers')
        # Fixed background layer (topo_static_pub); merged with the live nodes so
        # static structure stays solid even when a camera is occluded and the
        # live map has a hole there. Empty = live-only (original behaviour).
        p('static_markers_topic', '/topo_map/static/markers')
        p('world_frame', 'world')
        p('object_id', 'gng_obstacles')
        # A CollisionObject with N sphere primitives costs MoveIt O(N) per
        # collision check AND an FCL world rebuild on every /planning_scene diff.
        # At 0.08/1.5 Hz that was ~800 spheres rebuilt 1.5x/s -> move_group
        # saturated (IK/plan/scene-query all timed out). 0.15 m halves the count
        # (~670) and 0.5 Hz cuts the rebuild churn 3x, keeping move_group
        # responsive while still lighter than a dense octomap.
        p('collision_leaf', 0.15)     # downsample grid (m) -> fewer spheres
        p('sphere_radius', 0.07)      # per-node collision sphere radius (m):
        #   smaller = less conservative, arm paths clear the obstacles more easily
        p('carve_radius', 0.15)       # exclude nodes within this of the carve point
        p('publish_hz', 0.5)
        # Live self-filter for the STATIC layer too: a bad map_topo_static
        # capture (self-filter failed at capture time, e.g. duplicate
        # realsense/TF processes corrupting the depth feed) can permanently
        # bake arm-surface points into topo_static.npz. Those static_nodes
        # bypass env_gng's live filtering entirely (they come straight from
        # the saved map), so without this they'd sit on the arm as collision
        # spheres FOREVER -- constant false self-collision. Re-checking them
        # against the CURRENT arm TF every tick (same capsule test env_gng
        # uses for live nodes) drops them regardless of capture quality.
        #
        # Radii here are intentionally LARGER than map_topo_static's/env_gng's
        # capture-time filter (0.07/0.05): a point can survive that capsule
        # test (be > filter_r from the link centerline) yet still lie inside
        # the arm's real URDF collision mesh, which is chunkier than the
        # capsule at the shoulder/gripper/wrist -- verified empirically via
        # /check_state_validity across several arm poses: 0.07/0.05 and even
        # 0.11/0.08 left real contacts (gripper_base_link, shoulder_link,
        # arm_link, left_finger_prox_link -- the last because
        # left_finger_prox/dist_link and right_finger_dist_link have NO TF at
        # all on this rig's gripper, so the finger capsule test silently
        # never applies to them). This is the PERMANENT background layer only
        # (real dynamic obstacles come from env_gng's live layer, filtered
        # separately with its own moving-window history) -- a generous
        # margin here just means a real static object within ~18cm of the
        # arm's own body won't show up via this layer, an acceptable trade
        # for never again painting the arm as its own obstacle.
        p('world_frame_self_filter', True)
        p('arm_prefixes', ['t1_a1', 't1_a2', 't2_a1', 't2_a2'])
        p('self_filter_radius', 0.18)
        p('finger_radius', 0.15)
        # Sample the arm's TF pose at this rate and keep a short HISTORY
        # (self_filter_frames snapshots), filtering the static layer against
        # the whole swept history rather than one instantaneous snapshot --
        # same fix env_gng already applies to its live layer. Needed because
        # this node only republishes /planning_scene at publish_hz (0.5 Hz
        # default = every 2s): a single-snapshot filter taken at publish time
        # is stale by the time a planner checks state validity a moment
        # later, so a still-moving arm can outrun it and show transient
        # false self-collision even with a generous radius (seen empirically
        # 2026-07-30 -- contacts kept reappearing on DIFFERENT links each
        # retest, consistent with the arm moving between checks, not a fixed
        # radius shortfall).
        p('self_filter_sample_hz', 10.0)
        p('self_filter_frames', 20)   # 10 Hz x 20 = 2s of recent pose history
        p('tf_warn_grace_sec', 5.0)   # suppress missing-TF warnings this long
        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.object_id = g('object_id')
        self.leaf = float(g('collision_leaf'))
        self.radius = float(g('sphere_radius'))
        self.carve_r = float(g('carve_radius'))
        self.static_self_filter = bool(g('world_frame_self_filter'))
        self.arm_prefixes = list(g('arm_prefixes'))
        self.filter_r = float(g('self_filter_radius'))
        self.finger_r = float(g('finger_radius'))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._arm_hist = deque(maxlen=max(1, int(g('self_filter_frames'))))
        self._start_t = time.monotonic()
        self.warn_grace_s = float(g('tf_warn_grace_sec'))
        sample_hz = max(float(g('self_filter_sample_hz')), 1.0)
        self.create_timer(1.0 / sample_hz, self._sample_arm_pose)

        self.nodes = np.empty((0, 3))
        self.static_nodes = np.empty((0, 3))
        self.carve = None             # xyz to exclude (GRASP mode), or None
        self.hold = False             # freeze the scene during arm execution
        self.create_subscription(MarkerArray, g('env_markers_topic'),
                                 self._on_env, 1)
        static_topic = g('static_markers_topic')
        if static_topic:
            # transient-local to match topo_static_pub's latched publisher.
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile
            sq = QoSProfile(depth=1)
            sq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(MarkerArray, static_topic,
                                     self._on_static, sq)
        self.create_subscription(PointStamped, '/gng_collision/carve',
                                 self._on_carve, 1)
        # while an arm is executing, DON'T republish: a changing collision world
        # aborts MoveIt execution (MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE).
        self.create_subscription(Bool, '/gng_collision/hold',
                                 lambda m: setattr(self, 'hold', m.data), 1)
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 1)
        self.create_timer(1.0 / max(float(g('publish_hz')), 0.5), self._tick)
        self.get_logger().info(
            f'gng_collision up; leaf={self.leaf} radius={self.radius}')

    def _on_env(self, msg):
        if msg.markers:
            self.nodes = np.array([[q.x, q.y, q.z] for q in msg.markers[0].points])

    def _on_static(self, msg):
        if msg.markers:
            self.static_nodes = np.array(
                [[q.x, q.y, q.z] for q in msg.markers[0].points])

    def _on_carve(self, msg):
        p = msg.point
        self.carve = (None if not np.isfinite([p.x, p.y, p.z]).all()
                      else np.array([p.x, p.y, p.z]))

    def _sample_arm_pose(self):
        cache = arm_positions(self.tf_buffer, self.world_frame,
                              self.arm_prefixes)
        self._arm_hist.append(cache)
        # FAIL LOUD when arm TF is missing. filter_by_positions SKIPS any
        # capsule whose endpoint TF is None, so with no arm TF at all the
        # self-filter silently removes ZERO points -- behaviourally identical
        # to self_filter:=false, and the static map's baked-in arm points come
        # straight back as false self-collision (the arm goes red). That is
        # exactly what "I relaunched and it's still red" looked like on
        # 2026-07-31: the 4 arms were unreachable on 192.168.2.x, so
        # ros2_control never started, nothing published /joint_states,
        # robot_state_publisher emitted no arm TF, and this filter quietly
        # no-opped. Without this warning the node looks perfectly healthy
        # while doing nothing. See [[network-192-168-2-subnet]].
        # Grace period: the TransformListener starts with an empty buffer, so
        # the first samples legitimately find no TF. Warning there would cry
        # wolf on every startup.
        if time.monotonic() - self._start_t < self.warn_grace_s:
            return
        missing = sum(1 for v in cache.values() if v is None)
        if missing == len(cache):
            self.get_logger().warn(
                'self-filter INACTIVE: no arm TF at all for '
                f'{self.arm_prefixes} -- the static map\'s arm points are NOT '
                'being removed, so the arms will show FALSE self-collision. '
                'Is ros2_control/joint_state_broadcaster up and publishing '
                '/joint_states? (check the arms are reachable)',
                throttle_duration_sec=10.0)
        elif missing:
            self.get_logger().warn(
                f'self-filter PARTIAL: {missing}/{len(cache)} arm TF frames '
                'missing -- those links will not be filtered out of the '
                'static map', throttle_duration_sec=30.0)

    def _spheres(self):
        static_nodes = self.static_nodes
        if self.static_self_filter and len(static_nodes) and self._arm_hist:
            static_nodes = filter_by_positions(
                static_nodes, list(self._arm_hist), self.arm_prefixes,
                self.filter_r, self.finger_r)
        # union of the live (dynamic) and fixed (static background) node sets;
        # the leaf downsample below collapses any overlap between them.
        if len(static_nodes) and len(self.nodes):
            pts = np.vstack([self.nodes, static_nodes])
        elif len(static_nodes):
            pts = static_nodes
        else:
            pts = self.nodes
        if len(pts) == 0:
            return pts
        if self.leaf > 0:             # coarse grid -> lighter than octomap
            keys = np.floor(pts / self.leaf).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            pts = pts[idx]
        if self.carve is not None:    # GRASP mode: drop the target region
            pts = pts[np.linalg.norm(pts - self.carve, axis=1) > self.carve_r]
        return pts

    def _tick(self):
        if self.hold:                 # frozen during execution -> scene stays put
            return
        pts = self._spheres()
        co = CollisionObject()
        co.header.frame_id = self.world_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = self.object_id
        co.operation = CollisionObject.ADD
        for xyz in pts:
            s = SolidPrimitive()
            s.type = SolidPrimitive.SPHERE
            s.dimensions = [self.radius]
            co.primitives.append(s)
            co.primitive_poses.append(Pose(position=Point(
                x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))))
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        self.pub.publish(scene)

    def clear_scene(self):
        """Remove gng_obstacles so killing this node (Ctrl+C, crash) doesn't
        leave a stale collision object frozen in the planning scene forever --
        PlanningScene ADD diffs persist in move_group's scene independent of
        whether the publisher is still alive, so without an explicit REMOVE
        on shutdown the LAST published pose (possibly mid-arm-motion) stays
        stuck as a permanent false self-collision until someone notices and
        clears it by hand (seen 2026-07-30)."""
        co = CollisionObject()
        co.header.frame_id = self.world_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = self.object_id
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        # publish a few times with a short pause: this runs during shutdown,
        # after rclpy.spin() has already returned, so nothing is spinning to
        # flush the DDS write -- a single publish can be lost if move_group
        # isn't listening at that exact instant.
        for _ in range(3):
            self.pub.publish(scene)
            time.sleep(0.1)


def main():
    # NO: rclpy's default SIGINT handler calls try_shutdown() on the context
    # as soon as Ctrl+C arrives -- BEFORE our `finally` block runs -- so
    # clear_scene()'s publish() would silently no-op against an already-dead
    # context and the stale gng_obstacles never gets removed (seen
    # 2026-07-30: node exited cleanly on `kill -INT`, but the collision
    # object stayed stuck). Disabling it here means Ctrl+C instead raises a
    # normal Python KeyboardInterrupt while the context is still alive, so
    # clear_scene() can actually publish the REMOVE before we shut down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GngCollision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_scene()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
