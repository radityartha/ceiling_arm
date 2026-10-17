#!/usr/bin/env python3
"""MOCK saja: putar ulang rekaman mentah live_rec (<M>_raw.jsonl) ke /loc_rgbd2/tracks (QoS = object_localizer), berulang 2 Hz.
    python3 replay.py <M>_raw.jsonl"""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

rows = [json.loads(ln)['tracks'] for ln in open(sys.argv[1]) if ln.strip()]
rclpy.init()
n = Node('demo_zone_replay')
q = QoSProfile(depth=1)
q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
pub = n.create_publisher(String, '/loc_rgbd2/tracks', q)
i = [0]


def tick():
    pub.publish(String(data=json.dumps(rows[i[0] % len(rows)])))
    i[0] += 1


n.create_timer(0.5, tick)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
