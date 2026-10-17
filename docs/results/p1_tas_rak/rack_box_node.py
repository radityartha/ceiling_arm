#!/usr/bin/env python3
"""TAS-RAK: teruskan HANYA track di kotak atas lemari (sisi gantry 2), TANPA koreksi (xy, z = terukur; z track ~ permukaan atas,
bukti botol DEMO-ZONE-HW B1). Track lain (tas lantai, botol meja, ...) -> /rack_box/rejected. Runner/CLI di-remap ke /rack_box/tracks.
  in : /detected_objects/tracks (object_localizer fusi rgbd + rgbd2)   out: /rack_box/tracks (skema sama), /rack_box/rejected
    python3 rack_box_node.py [--ros-args -p in_topic:=/loc_rgbd2/tracks]
"""
import json

BOX = ((0.80, 1.16), (-0.98, -0.68), (1.00, 1.25))   # tas terlihat (0.956-1.001, -0.807..-0.855, 1.076-1.094) +- ~15 cm; z <= 1.25


def process(tracks, box=BOX):
    ok, rej = [], []
    for t in tracks:
        p = (float(t['x']), float(t['y']), float(t['z']))
        inside = all(lo <= v <= hi for v, (lo, hi) in zip(p, box))
        (ok if inside else rej).append(dict(t) if inside else dict(t, reason='di luar kotak rak %s' % (box,)))
    return ok, rej


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class RackBox(Node):
        def __init__(self):
            super().__init__('rack_box')
            inp = self.declare_parameter('in_topic', '/detected_objects/tracks').value
            q = QoSProfile(depth=1)
            q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.pub_ok = self.create_publisher(String, '/rack_box/tracks', q)
            self.pub_rej = self.create_publisher(String, '/rack_box/rejected', q)
            self.create_subscription(String, inp, self._on, q)
            self.last = None
            self.get_logger().info('kotak %s; in %s' % (BOX, inp))

        def _on(self, m):
            ok, rej = process(json.loads(m.data))
            self.pub_ok.publish(String(data=json.dumps(ok)))
            self.pub_rej.publish(String(data=json.dumps(rej)))
            key = tuple(t['tid'] for t in ok)
            if key != self.last:
                self.last = key
                for t in ok:
                    self.get_logger().info('TERIMA #%d %s (%.3f, %.3f, %.3f)' % (t['tid'], t['label'], t['x'], t['y'], t['z']))

    rclpy.init()
    n = RackBox()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
