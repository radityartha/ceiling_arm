import sys, rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node('grab'); got = {}
def cb(ns):
    def f(m):
        if ns in got: return
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8': a = a[:, :, ::-1]
        got[ns] = a
    return f
for ns in ('rgbd', 'rgbd2'): n.create_subscription(Image, f'/{ns}/rgb', cb(ns), 1)
import time; t = time.time()
while len(got) < 2 and time.time() - t < 15: rclpy.spin_once(n, timeout_sec=0.2)
for ns, a in got.items(): cv2.imwrite(f'{sys.argv[1]}/{ns}.jpg', cv2.resize(a, (640, 360))); print(ns, a.shape)
