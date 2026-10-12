#!/usr/bin/env python3
"""CAM-6DOF K4 (DEMO-HW B1 pattern): project world points into a live camera image and save an annotated jpg. NO motion.
Uses /<ns>/rgb + /<ns>/camera_info + live TF world -> <ns>_camera_optical (whatever extrinsic is running).
    python3 proj_marks.py rgbd out.jpg F1:0.40,-0.55,0 F3:0.55,-0.55,0 ...
"""
import sys, time
import cv2, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
ns, out, pts = sys.argv[1], sys.argv[2], [(a.split(':')[0], np.array(a.split(':')[1].split(','), float)) for a in sys.argv[3:]]
rclpy.init(); n = rclpy.create_node('proj_marks'); tf = Buffer(); TransformListener(tf, n); got = {}
n.create_subscription(Image, f'/{ns}/rgb', lambda m: got.__setitem__('img', m), 1)
n.create_subscription(CameraInfo, f'/{ns}/camera_info', lambda m: got.__setitem__('K', np.array(m.k).reshape(3, 3)), 1)
t0 = time.time()
while time.time() - t0 < 15 and not ({'img', 'K'} <= got.keys() and tf.can_transform('world', f'{ns}_camera_optical', Time())):
    rclpy.spin_once(n, timeout_sec=0.1)
m = got['img']; img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1).copy()
if m.encoding == 'rgb8': img = img[:, :, ::-1].copy()
tr = tf.lookup_transform('world', f'{ns}_camera_optical', Time()).transform
x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([tr.translation.x, tr.translation.y, tr.translation.z]); K = got['K']
for name, p in pts:
    c = R.T @ (p - T); u, v = (K @ c)[:2] / c[2]; inside = c[2] > 0 and 0 <= u < m.width and 0 <= v < m.height
    print(f'{name} world {p} -> range {np.linalg.norm(c):.2f} m  pixel ({u:.0f}, {v:.0f}) {"IN" if inside else "OUT of frame"}')
    if inside:
        cv2.circle(img, (int(u), int(v)), 10, (0, 0, 255), 2)
        cv2.putText(img, name, (int(u) + 12, int(v) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
cv2.imwrite(out, img); print('saved', out)
