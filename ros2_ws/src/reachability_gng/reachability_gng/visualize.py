"""Publish the GNG map as RViz markers (ports the sensei's drawGCS/drawNode).

    pub : ~/gng_markers  visualization_msgs/MarkerArray

Nodes are spheres coloured by a per-node scalar (reachability hits or
manipulability); edges are drawn as a line list. Node positions use the xyz
part of each node's task vector, expressed in --frame.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.gng import GNG


def _colormap(t):
    """Simple blue->red ramp for t in [0,1] -> (r,g,b)."""
    t = float(np.clip(t, 0.0, 1.0))
    return t, 0.0, 1.0 - t


class GngViz(Node):
    def __init__(self):
        super().__init__('gng_visualize')
        self.declare_parameter('model_path', 'model.npz')
        self.declare_parameter('frame', 'world')
        self.declare_parameter('color_by', 'manip')   # 'manip' or 'hits'

        model_path = self.get_parameter('model_path').value
        self.frame = self.get_parameter('frame').value
        self.color_by = self.get_parameter('color_by').value

        self.gng = GNG.load(model_path)
        self.stats = {}
        stats_path = (model_path[:-4] if model_path.endswith('.npz')
                      else model_path) + '_stats.npz'
        try:
            self.stats = dict(np.load(stats_path))
        except OSError:
            self.get_logger().warn(f'No stats file {stats_path}; coloring flat.')

        self.pub = self.create_publisher(MarkerArray, '~/gng_markers', 1)
        self.timer = self.create_timer(1.0, self._publish)

    def _scalar(self):
        s = self.stats.get(self.color_by)
        if s is None:
            return np.ones(len(self.gng.W))
        rng = np.ptp(s)
        return (s - s.min()) / rng if rng > 0 else np.zeros_like(s)

    def _publish(self):
        W = self.gng.W
        xyz = W[:, :3]
        scalar = self._scalar()
        arr = MarkerArray()

        nodes = Marker()
        nodes.header.frame_id = self.frame
        nodes.ns = 'gng_nodes'
        nodes.id = 0
        nodes.type = Marker.SPHERE_LIST
        nodes.action = Marker.ADD
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.02
        for p, t in zip(xyz, scalar):
            from geometry_msgs.msg import Point
            from std_msgs.msg import ColorRGBA
            nodes.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            r, g, b = _colormap(t)
            nodes.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))
        arr.markers.append(nodes)

        edges = Marker()
        edges.header.frame_id = self.frame
        edges.ns = 'gng_edges'
        edges.id = 1
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.scale.x = 0.003
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = \
            0.0, 1.0, 0.0, 0.4
        from geometry_msgs.msg import Point
        for e in self.gng._edges:
            a, b = tuple(e)
            edges.points.append(Point(x=float(xyz[a][0]), y=float(xyz[a][1]),
                                      z=float(xyz[a][2])))
            edges.points.append(Point(x=float(xyz[b][0]), y=float(xyz[b][1]),
                                      z=float(xyz[b][2])))
        arr.markers.append(edges)

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = GngViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
