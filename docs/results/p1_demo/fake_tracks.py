#!/usr/bin/env python3
"""DEMO-OFF mock fixture: one fake perceived object, as object_localizer would publish it.

    /detected_objects         PoseArray (world)
    /detected_objects/tracks  String JSON [{tid,label,x,y,z,conf}]  (TRANSIENT_LOCAL)

    python3 fake_tracks.py --tid 101 --label bottle --xyz 1.75,0.55,0.97
"""
import argparse
import json

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tid', type=int, required=True)
    ap.add_argument('--label', default='bottle')
    ap.add_argument('--xyz', required=True)
    a = ap.parse_args()
    x, y, z = (float(v) for v in a.xyz.split(','))
    rclpy.init()
    n = Node('fake_tracks')
    pose_pub = n.create_publisher(PoseArray, '/detected_objects', 1)
    q = QoSProfile(depth=1)
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    tr_pub = n.create_publisher(String, '/detected_objects/tracks', q)
    tracks = json.dumps([{'tid': a.tid, 'label': a.label, 'x': x, 'y': y, 'z': z, 'conf': 0.9}])

    def tick():
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp = n.get_clock().now().to_msg()
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        pa.poses.append(p)
        pose_pub.publish(pa)
        tr_pub.publish(String(data=tracks))

    n.create_timer(0.5, tick)
    n.get_logger().info(f'fake track #{a.tid} {a.label} ({x}, {y}, {z})')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
