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

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PointStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray


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
        g = lambda k: self.get_parameter(k).value
        self.world_frame = g('world_frame')
        self.object_id = g('object_id')
        self.leaf = float(g('collision_leaf'))
        self.radius = float(g('sphere_radius'))
        self.carve_r = float(g('carve_radius'))

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

    def _spheres(self):
        # union of the live (dynamic) and fixed (static background) node sets;
        # the leaf downsample below collapses any overlap between them.
        if len(self.static_nodes) and len(self.nodes):
            pts = np.vstack([self.nodes, self.static_nodes])
        elif len(self.static_nodes):
            pts = self.static_nodes
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


def main():
    rclpy.init()
    node = GngCollision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
