#!/usr/bin/env python3
"""Measure Isaac sim load: real-time factor (from /clock) + bridge %CPU/RSS.

Usage (system Humble, same env as the bridge):
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 scripts/sim_perf_probe.py [window_s] [extra_topic ...]

Prints one line per KPI so before/after runs can be diffed directly.
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock


HZ = os.sysconf("SC_CLK_TCK")


def bridge_pid():
    out = subprocess.run(["pgrep", "-f", "ros2_bridge_gui.py"],
                         capture_output=True, text=True).stdout.split()
    return int(out[0]) if out else None


def cpu_ticks(pid):
    """(utime+stime ticks, rss_mb) for pid, or (None, None) if it is gone.

    Read from /proc directly: `ps %cpu` is an average over the process's whole
    lifetime, which for a hours-old bridge hides what it is doing right now.
    """
    try:
        f = open(f"/proc/{pid}/stat").read()
        fields = f[f.rindex(")") + 2:].split()
        ticks = int(fields[11]) + int(fields[12])          # utime + stime
        rss = int(open(f"/proc/{pid}/statm").read().split()[1]) * 4096 / 2**20
        return ticks, rss
    except (FileNotFoundError, ProcessLookupError, IndexError):
        return None, None


class Probe(Node):
    def __init__(self, topics):
        super().__init__("sim_perf_probe")
        self.sim_t = []
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        # NOTE: not `self._clock` -- rclpy's Node already owns that attribute.
        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.counts = {t: 0 for t in topics}
        self._subs = []
        for t in topics:
            self._subs.append(self._sub_any(t))

    def _on_clock(self, msg):
        self.sim_t.append((msg.clock.sec + msg.clock.nanosec * 1e-9, time.time()))

    def _sub_any(self, topic):
        """Generic subscriber: resolve the type at runtime, count messages."""
        from rclpy.utilities import get_rmw_implementation_identifier  # noqa: F401
        for _ in range(20):
            info = self.get_topic_names_and_types()
            match = [ty for nm, ty in info if nm == topic]
            if match:
                break
            time.sleep(0.25)
        else:
            self.get_logger().warn(f"topic not found: {topic}")
            return None
        from rosidl_runtime_py.utilities import get_message
        msg_type = get_message(match[0][0])
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        return self.create_subscription(
            msg_type, topic, lambda _m, t=topic: self._bump(t), best)

    def _bump(self, topic):
        self.counts[topic] += 1


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    topics = sys.argv[2:]
    rclpy.init()
    node = Probe(topics)
    pid = bridge_pid()
    tick0, _ = cpu_ticks(pid) if pid else (None, None)
    t0 = time.time()
    while time.time() - t0 < window:
        rclpy.spin_once(node, timeout_sec=0.1)
    tick1, rss = cpu_ticks(pid) if pid else (None, None)
    wall = time.time() - t0

    if len(node.sim_t) >= 2:
        (s0, w0), (s1, w1) = node.sim_t[0], node.sim_t[-1]
        rtf = (s1 - s0) / (w1 - w0)
        print(f"RTF                : {rtf:.3f}x  "
              f"(sim {s1 - s0:.2f}s / wall {w1 - w0:.2f}s, {len(node.sim_t)} /clock msgs)")
        print(f"/clock hz          : {len(node.sim_t) / (w1 - w0):.1f}")
    else:
        print(f"RTF                : NO DATA ({len(node.sim_t)} /clock msgs in {window}s)")

    if tick0 is not None and tick1 is not None:
        print(f"bridge %CPU        : {(tick1 - tick0) / HZ / wall * 100:.0f}% "
              f"(pid {pid}, instantaneous over {wall:.0f}s)")
        print(f"bridge RSS         : {rss:.0f} MB")
    else:
        print("bridge %CPU        : bridge process NOT FOUND")

    for t, c in node.counts.items():
        print(f"{t:<19}: {c / window:.2f} hz ({c} msgs / {window:.0f}s)")

    ctrl = subprocess.run(["pgrep", "-fc", "ros2_control_node"],
                          capture_output=True, text=True).stdout.strip()
    print(f"ros2_control_node  : {ctrl or 0} alive")
    print(f"load average       : {open('/proc/loadavg').read().split()[0]}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
