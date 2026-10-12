#!/usr/bin/env python3
"""ARM1-ZONE: TCP = TF world -> <frame> (robot_state_publisher from MEASURED joints), mean/std over N s.
    python3 tcp_read.py t1_a1_tool_frame 4"""
import sys, time
import numpy as np, rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
fr, dur = sys.argv[1], float(sys.argv[2])
rclpy.init(); n = rclpy.create_node('tcp_read'); tf = Buffer(); TransformListener(tf, n); P = []; t0 = time.time()
while time.time() - t0 < dur + 1:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 > 1 and tf.can_transform('world', fr, Time()):
        tr = tf.lookup_transform('world', fr, Time()).transform.translation; P.append((tr.x, tr.y, tr.z))
P = np.array(P); print(f'{fr}: n {len(P)} mean {P.mean(0).round(4)} std_mm {(P.std(0) * 1000).round(2)}')
