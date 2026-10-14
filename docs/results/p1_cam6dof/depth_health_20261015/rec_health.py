#!/usr/bin/env python3
"""Depth-health recorder: per-frame valid fraction of RAW depth (image_rect_raw, 16UC1, 0 = invalid) + depth metadata.

rec_health.py <ns> <seconds> <out.jsonl> [aligned]   (domain from env; aligned = aligned_depth_to_color). Also saves up to 3 infra1 PNGs if infra1 is streaming.
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import Image

ns, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
rclpy.init()
node = rclpy.create_node(f'rec_health_{ns}')
f = open(out, 'w', buffering=1)
meta = {}
n_ir = [0]


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def on_meta(m):
    try:
        meta[round(stamp(m.header), 4)] = json.loads(m.json_data)
    except ValueError:
        pass


def on_depth(m):
    d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
    f.write(json.dumps({'t': stamp(m.header), 'rx': time.time(), 'kind': 'depth',
                        'valid': float((d > 0).mean()), 'w': m.width, 'h': m.height}) + '\n')


def on_ir(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
    f.write(json.dumps({'t': stamp(m.header), 'kind': 'ir', 'mean': float(a.mean()),
                        'p99': float(np.percentile(a, 99))}) + '\n')
    if n_ir[0] < 3:
        import cv2
        cv2.imwrite(out.replace('.jsonl', f'_ir{n_ir[0]}.png'), a)
        n_ir[0] += 1


base = f'/{ns}/{ns}'
topic = f'{base}/aligned_depth_to_color/image_raw' if len(sys.argv) > 4 and sys.argv[4] == 'aligned' \
    else f'{base}/depth/image_rect_raw'
node.create_subscription(Image, topic, on_depth, qos_profile_sensor_data)
node.create_subscription(Metadata, f'{base}/depth/metadata', on_meta, qos_profile_sensor_data)
node.create_subscription(Image, f'{base}/infra1/image_rect_raw', on_ir, qos_profile_sensor_data)
t_end = time.time() + secs
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.1)
for t, j in sorted(meta.items()):
    f.write(json.dumps({'t': t, 'kind': 'meta', 'meta': j}) + '\n')
f.close()
node.destroy_node()
rclpy.shutdown()
