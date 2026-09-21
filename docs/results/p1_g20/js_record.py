"""READ-ONLY recorder (g20): t, joint, pos, effort for ALL four arms + both rails -> CSV.
From p1_g18/js_record.py; only KEEP widened to t2_* (step 5)."""
import sys, time, rclpy, csv
from rclpy.node import Node
from sensor_msgs.msg import JointState
out = open(sys.argv[1], 'w', newline=''); w = csv.writer(out); w.writerow(['t', 'joint', 'pos', 'eff'])
KEEP = {f't{g}_a{a}_joint_{j}' for g in (1, 2) for a in (1, 2) for j in range(1, 7)} | \
       {f't{g}_{k}_joint' for g in (1, 2) for k in ('linear', 'rotation')}
def cb(m):
    t = time.time()
    for i, n in enumerate(m.name):
        if n in KEEP:
            w.writerow([f'{t:.4f}', n, f'{m.position[i]:.6f}', f'{m.effort[i]:.4f}' if i < len(m.effort) else ''])
rclpy.init(); nd = Node('g20_js_recorder'); nd.create_subscription(JointState, '/joint_states', cb, 200)
try:
    while rclpy.ok(): rclpy.spin_once(nd, timeout_sec=0.1); out.flush()
except KeyboardInterrupt: pass
out.close()
