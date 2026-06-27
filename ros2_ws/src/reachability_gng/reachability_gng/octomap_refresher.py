"""Periodically clear MoveIt's octomap so it rebuilds fresh (no stale voxels).

The overhead cameras are static and look down a corridor, so cells behind a
moving arm often have no depth return -- no clearing ray passes through them and
the arm "bakes in" as a permanent obstacle even after it moves away. This node
calls move_group's `/clear_octomap` (std_srvs/Empty) on a timer; the octomap
immediately starts rebuilding from the live clouds, so stale arm/object trails
disappear within one refresh period. It is the clean, MoveIt-native version of
"always rebuild": the blind window is only ~1/max_update_rate (tens of ms at
5-50 Hz) while the refresh period is ~1 s, so the map is populated almost all the
time. Pair it with an aggressive sensor self-filter (sensors_3d.yaml padding) so
the rebuilt map never contains the arm in the first place.

    ros2 run reachability_gng octomap_refresher
    ros2 run reachability_gng octomap_refresher --ros-args -p period:=1.5
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class OctomapRefresher(Node):
    def __init__(self):
        super().__init__('octomap_refresher')
        self.declare_parameter('service', '/clear_octomap')
        self.declare_parameter('period', 1.0)
        service = self.get_parameter('service').value
        period = float(self.get_parameter('period').value)

        self.cli = self.create_client(Empty, service)
        self._warned = False
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'octomap_refresher up; clearing {service} every {period:.1f}s')

    def _tick(self):
        if not self.cli.service_is_ready():
            if not self._warned:
                self.get_logger().warn(
                    f'{self.cli.srv_name} not available yet; waiting for move_group')
                self._warned = True
            return
        self._warned = False
        self.cli.call_async(Empty.Request())


def main():
    rclpy.init()
    node = OctomapRefresher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
