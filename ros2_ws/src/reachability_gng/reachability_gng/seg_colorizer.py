"""Colorize instance-segmentation (32SC1 label image) -> rgb8 for viewing.

The Isaac segmentation publishers emit a 32-bit label-id image (each pixel = an
instance id), which RViz/rqt cannot display (32SC1 is not a colour encoding).
This node maps each instance id to a distinct colour and republishes an rgb8
image you can show with an RViz Image display or rqt_image_view. Visualization
only -- object_localizer reads the raw 32SC1 mask directly, not this.

    /<ns>/instance_segmentation  (32SC1)  ->  /<ns>/instance_segmentation_color (rgb8)

    ros2 run reachability_gng seg_colorizer
"""
from __future__ import annotations

import colorsys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def color_for_id(i):
    """Deterministic, well-separated colour per instance id (bg/unlabelled black).

    Hues are spaced by the golden-ratio conjugate so consecutive ids land far
    apart on the colour wheel (no two near-identical colours).
    """
    if i <= 1:                                # 0 BACKGROUND, 1 UNLABELLED
        return (0, 0, 0)
    h = (i * 0.61803398875) % 1.0
    r, g, b = colorsys.hsv_to_rgb(h, 0.85, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


class SegColorizer(Node):
    def __init__(self):
        super().__init__('seg_colorizer')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        nss = list(self.get_parameter('camera_namespaces').value)
        self._pubs = {}
        self._lut = {0: (0, 0, 0), 1: (0, 0, 0)}   # id -> colour cache
        for ns in nss:
            self._pubs[ns] = self.create_publisher(
                Image, f'/{ns}/instance_segmentation_color', 1)
            self.create_subscription(
                Image, f'/{ns}/instance_segmentation',
                lambda m, ns=ns: self._on_seg(ns, m), 1)
        self.get_logger().info(f'seg_colorizer up; cameras={nss}')

    def _on_seg(self, ns, msg):
        cols = msg.step // 4
        seg = np.frombuffer(bytes(msg.data), dtype=np.int32) \
            .reshape(msg.height, cols)[:, :msg.width]
        out = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
        for uid in np.unique(seg):
            uid = int(uid)
            if uid not in self._lut:
                self._lut[uid] = color_for_id(uid)
            c = self._lut[uid]
            if c != (0, 0, 0):
                out[seg == uid] = c
        o = Image()
        o.header = msg.header
        o.height = msg.height
        o.width = msg.width
        o.encoding = 'rgb8'
        o.is_bigendian = 0
        o.step = msg.width * 3
        o.data = out.tobytes()
        self._pubs[ns].publish(o)


def main():
    rclpy.init()
    node = SegColorizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
