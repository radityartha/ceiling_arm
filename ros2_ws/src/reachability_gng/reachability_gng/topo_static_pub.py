"""Publish the saved STATIC GNG map as RViz/collision markers.

Loads the graph mapped once by map_topo_static.py and republishes it on a timer
as /topo_map/static/markers (transient-local, so a late RViz/gng_collision still
receives it). The layout is fixed -- same nodes/edges every run -- so it is the
reproducible backbone of the scene, while env_gng's /topo_map/markers carries
only the live/dynamic remainder. gng_collision merges both.

Drawn in a distinct colour (blue-grey) so it reads apart from the green live map.

    ros2 run reachability_gng topo_static_pub
    ros2 run reachability_gng topo_static_pub --ros-args -p map_file:=/tmp/topo_static.npz
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG


class TopoStaticPub(Node):
    def __init__(self):
        super().__init__('topo_static_pub')
        self.declare_parameter('map_file', '/tmp/topo_static.npz')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_period', 2.0)
        # Skip edges longer than this (m): the saved GNG has a few long "bridge"
        # edges spanning empty space (e.g. across the self-filtered arm gap) that
        # render as structure where the scene is empty. Nodes are unaffected, so
        # collision (which uses nodes) is unchanged. <=0 disables.
        self.declare_parameter('max_edge_len', 0.15)
        self.map_file = self.get_parameter('map_file').value
        self.world_frame = self.get_parameter('world_frame').value
        self.max_edge = float(self.get_parameter('max_edge_len').value)

        # transient-local so a late-joining subscriber (RViz, gng_collision)
        # still receives the last-published static map.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, '/topo_map/static/markers',
                                         qos)
        self._markers = self._build()
        if self._markers is None:
            self.get_logger().warn(
                f'map file {self.map_file} not found -- run map_topo_static '
                'first; will keep checking and publish once it appears')
        else:
            self.pub.publish(self._markers)
        self.create_timer(
            float(self.get_parameter('publish_period').value), self._tick)

    def _build(self):
        if not os.path.exists(self.map_file):
            return None
        g = GNG.load(self.map_file)
        W = g.W
        blue = ColorRGBA(r=0.35, g=0.55, b=0.95, a=1.0)
        now = self.get_clock().now().to_msg()

        def mk(ns, mid, mtype, size):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.world_frame, now
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = size
            m.color = blue
            return m

        nodes = mk('topo_static_nodes', 0, Marker.SPHERE_LIST, 0.02)
        nodes.points = [Point(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                        for w in W]
        edges = mk('topo_static_edges', 1, Marker.LINE_LIST, 0.005)
        for e in g._edges:
            i, j = tuple(e)
            if self.max_edge > 0 and np.linalg.norm(W[i] - W[j]) > self.max_edge:
                continue          # skip bridge edges spanning empty space
            edges.points += [
                Point(x=float(W[i][0]), y=float(W[i][1]), z=float(W[i][2])),
                Point(x=float(W[j][0]), y=float(W[j][1]), z=float(W[j][2]))]
        self.get_logger().info(
            f'static GNG: {len(W)} nodes, {len(g._edges)} edges from '
            f'{self.map_file}')
        return MarkerArray(markers=[nodes, edges])

    def _tick(self):
        if self._markers is None:
            self._markers = self._build()
            if self._markers is None:
                return
        self.pub.publish(self._markers)


def main():
    rclpy.init()
    node = TopoStaticPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
