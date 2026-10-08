import rclpy, time
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('tfq'); b = Buffer(); TransformListener(b, n)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.1)
for f in ['t1_a1_base_link','t1_a2_base_link','t1_a1_tool_frame','t1_a2_tool_frame','t2_a1_base_link','t2_a2_base_link']:
    t = b.lookup_transform('world', f, Time()).transform.translation
    print(f'{f:18s} {t.x:+.3f} {t.y:+.3f} {t.z:+.3f}')
