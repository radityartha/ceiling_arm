#!/usr/bin/env python3
"""Mean RGB brightness of N frames per camera (numbers only, no image saved)."""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('rgb_brightness')
vals = {'rgbd': [], 'rgbd2': []}
for ns in vals:
    node.create_subscription(Image, f'/{ns}/{ns}/color/image_raw',
                             lambda m, ns=ns: vals[ns].append(float(np.frombuffer(m.data, np.uint8).mean())),
                             qos_profile_sensor_data)
t = time.time() + 5
while time.time() < t:
    rclpy.spin_once(node, timeout_sec=0.1)
print(json.dumps({k: {'n': len(v), 'mean': float(np.median(v)) if v else None} for k, v in vals.items()}))
