#!/usr/bin/env python3
"""DEMO-ZONE-HW tampilan RViz (HANYA tampil): /demo_zone/{tracks,rejected} -> /demo_zone/markers (MarkerArray, world).
  kotak zona (garis biru, meja 0.77) | botol terkoreksi (silinder hijau 7 cm) | p_cmd = terkoreksi + standoff (bola kuning)
  track mentah rgbd2 (bola merah) + panah mentah -> terkoreksi | ditolak (bola abu + alasan)
"""
import json
import os
import sys

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'p1_demo_zone'))
from zone_correct_node import TABLE_Z, ZONE_XY  # noqa: E402


class ZoneMarkers(Node):
    def __init__(self):
        super().__init__('demo_zone_markers')
        self.standoff = self.declare_parameter('standoff', 0.25).value
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.ok, self.rej = [], []
        self.create_subscription(String, '/demo_zone/tracks', lambda m: self._on(m, 'ok'), q)
        self.create_subscription(String, '/demo_zone/rejected', lambda m: self._on(m, 'rej'), q)
        self.pub = self.create_publisher(MarkerArray, '/demo_zone/markers', q)
        self._publish()

    def _on(self, m, k):
        try:
            setattr(self, k, json.loads(m.data))
        except (ValueError, TypeError):
            return
        self._publish()

    def _mk(self, ns, i, typ, rgba, scale, pos=None):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.pose.orientation.w = 1.0
        if pos is not None:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        return m

    def _publish(self):
        arr = [Marker(action=Marker.DELETEALL)]
        (x0, x1), (y0, y1) = ZONE_XY
        z = self._mk('zona', 0, Marker.LINE_STRIP, (0.2, 0.4, 1.0, 1.0), (0.01, 0, 0))
        z.points = [Point(x=x, y=y, z=TABLE_Z) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))]
        arr.append(z)
        for i, t in enumerate(self.ok):
            c, r = (t['x'], t['y'], t['z']), (t['x_trk'], t['y_trk'], t['z_trk'])
            arr.append(self._mk('botol', i, Marker.CYLINDER, (0.1, 0.9, 0.2, 0.9), (0.065, 0.065, 0.07), c))
            arr.append(self._mk('p_cmd', i, Marker.SPHERE, (1.0, 0.85, 0.0, 1.0), (0.04,) * 3,
                                (c[0], c[1], c[2] + self.standoff)))
            arr.append(self._mk('mentah', i, Marker.SPHERE, (0.9, 0.1, 0.1, 0.8), (0.03,) * 3, r))
            a = self._mk('koreksi', i, Marker.ARROW, (0.9, 0.5, 0.1, 1.0), (0.008, 0.02, 0.03))
            a.points = [Point(x=r[0], y=r[1], z=r[2]), Point(x=c[0], y=c[1], z=c[2])]
            arr.append(a)
            tx = self._mk('label', i, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1), (0, 0, 0.05),
                          (c[0], c[1], c[2] + self.standoff + 0.08))
            tx.text = '#%d' % t['tid']
            arr.append(tx)
        for i, t in enumerate(self.rej):
            r = (t['x_trk'], t['y_trk'], t['z_trk'])
            arr.append(self._mk('tolak', i, Marker.SPHERE, (0.5, 0.5, 0.5, 0.7), (0.03,) * 3, r))
            tx = self._mk('tolak_label', i, Marker.TEXT_VIEW_FACING, (0.7, 0.7, 0.7, 1), (0, 0, 0.035),
                          (r[0], r[1], r[2] + 0.06))
            tx.text = 'TOLAK #%d' % t['tid']
            arr.append(tx)
        self.pub.publish(MarkerArray(markers=arr))


def main():
    rclpy.init()
    n = ZoneMarkers()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
