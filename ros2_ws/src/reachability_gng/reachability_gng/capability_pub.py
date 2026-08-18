"""Publish the capability map (Layer 2) as RViz markers.

    pub : /capability/markers  visualization_msgs/MarkerArray  (transient local)

The map itself (`capability.CapabilityMap`, data/cap_g{1,2}_rail160.npz) answers
"from which gantry poses can this arm reach this point?". A picture of it has to
commit to ONE of those two questions, so there are two modes:

  pose   (default) the nodes this arm reaches AT ONE gantry pose -- the set
         reachable_targets.py picks its targets from. Needs `lin`/`rot`.
  index  every node, coloured by the FRACTION of the 2376 gantry poses that
         reach it (blue = few, red = many). Pose-independent; `lin`/`rot` are
         ignored, and the result is the classic capability/reachability index.

`lin` is NOT cosmetic in pose mode: the reachable set moves with the rail, so a
wrong `lin` draws a confident picture of the wrong workspace. Read it from the
machine rather than assuming -- the same rule reachable_targets.py states:

    ros2 topic echo /joint_states --once | grep -A30 t1_linear

    ros2 run reachability_gng capability_pub --ros-args -p arm:=arm_1 -p lin:=0.55
    ros2 run reachability_gng capability_pub --ros-args -p mode:=index

Reachable is NOT torque-safe: ~30% of arm_1's reachable workspace exceeds
joint_2's 14 N.m rating (docs/p1_g17_hw.md). This draws the reachable set only;
scripts/torque_safe_workspace.py is what scores the safe subset.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from reachability_gng.capability import GANTRY_ARM, PARTNER, CapabilityMap

# Which capability map, and which of its two arm slots, each real arm is.
# The in-map names are not 'arm1'/'arm2' on both gantries (gantry 2's pair is
# 'arm3'/'arm4'), so they come from the map's own tables -- same construction
# as scripts/reachable_targets.py, so the picture and the target list agree.
ARM_MAP = {'arm_1': (1, GANTRY_ARM[1]), 'arm_2': (1, PARTNER[1]),
           'arm_3': (2, GANTRY_ARM[2]), 'arm_4': (2, PARTNER[2])}


def _map_path(gantry):
    """Repo copy first, /tmp as fallback -- same resolution as sched_arm.CLOUD."""
    for up in Path(__file__).resolve().parents:
        cand = up / 'data' / f'cap_g{gantry}_rail160.npz'
        if cand.exists():
            return str(cand)
    return f'/tmp/cap_g{gantry}_rail160.npz'


def _ramp(t):
    """blue -> red for t in [0,1]; same ramp as visualize.py's nodes."""
    return float(t), 0.0, float(1.0 - t)


class CapabilityPub(Node):
    def __init__(self):
        super().__init__('capability_pub')
        self.declare_parameter('map_file', '')      # '' -> resolve from `arm`
        self.declare_parameter('arm', 'arm_1')
        self.declare_parameter('lin', 0.55)         # rail position, METRES
        self.declare_parameter('rot', 0.0)          # gantry rotation, DEGREES
        self.declare_parameter('tol_i', 0)          # 0 = tightest (5 cm, L1)
        self.declare_parameter('mode', 'pose')      # 'pose' | 'index'
        self.declare_parameter('cube_size', 0.06)   # node grid step is ~7 cm
        self.declare_parameter('alpha', 0.35)       # see the room cloud through it
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_period', 2.0)

        arm = self.get_parameter('arm').value
        if arm not in ARM_MAP:
            raise ValueError(f'arm must be one of {sorted(ARM_MAP)}, got {arm!r}')
        self.arm = arm
        self.gantry, self.arm_key = ARM_MAP[arm]
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('pose', 'index'):
            raise ValueError(f"mode must be 'pose' or 'index', got {self.mode!r}")
        self.world_frame = self.get_parameter('world_frame').value
        self.cube = float(self.get_parameter('cube_size').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.map_file = (self.get_parameter('map_file').value
                         or _map_path(self.gantry))

        # transient-local, like topo_static_pub: the set is fixed for a given
        # pose, so a late-joining RViz must still get it.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, '/capability/markers', qos)

        self._markers = self._build()
        self.pub.publish(self._markers)
        self.create_timer(float(self.get_parameter('publish_period').value),
                          lambda: self.pub.publish(self._markers))

    def _build(self):
        if not os.path.exists(self.map_file):
            raise FileNotFoundError(
                f'{self.map_file} not found. Restore it with: cp ros2_ws/src/'
                'reachability_gng/data/cap_g*_rail160.npz /tmp/')
        cm = CapabilityMap.load(self.map_file)
        tol_i = int(self.get_parameter('tol_i').value)
        L, Rn = len(cm.lin), len(cm.rot)

        # masks are (n_tol, N, L*Rn) in meshgrid 'ij' order, so a node's row
        # reshapes to (L, Rn) directly -- no per-node KD-tree lookup needed
        # (cm.reach's BMU of a node is that node). Partner arm = half-turn roll,
        # exact, the same operation cm.reach applies.
        m = cm.masks[tol_i].reshape(-1, L, Rn)
        if self.arm_key == PARTNER[cm.gantry]:
            m = np.roll(m, Rn // 2, axis=2)

        if self.mode == 'index':
            frac = m.reshape(len(cm.nodes), -1).mean(axis=1)
            keep = frac > 0
            pts, col = cm.nodes[keep], [_ramp(f / frac.max()) for f in frac[keep]]
            label = (f'{self.arm} capability index\n{keep.sum()} of '
                     f'{len(cm.nodes)} nodes, from >=1 of {L * Rn} poses\n'
                     f'red = {frac.max() * 100:.0f}% of poses, '
                     f'tol {cm.tols[tol_i] * 100:.0f} cm')
        else:
            lin = float(self.get_parameter('lin').value)
            rot = float(self.get_parameter('rot').value)
            li = int(np.argmin(abs(cm.lin - lin)))
            ri = int(np.argmin(abs(cm.rot - np.radians(rot))))
            keep = m[:, li, ri]
            pts, col = cm.nodes[keep], [(1.0, 0.55, 0.0)] * int(keep.sum())
            label = (f'{self.arm} reachable\nlin={cm.lin[li]:.3f} m  '
                     f'rot={np.degrees(cm.rot[ri]):+.0f} deg\n{keep.sum()} of '
                     f'{len(cm.nodes)} nodes, tol {cm.tols[tol_i] * 100:.0f} cm')
            if abs(cm.lin[li] - lin) > 1e-9 or abs(cm.rot[ri] - np.radians(rot)) > 1e-9:
                self.get_logger().warn(
                    f'asked lin={lin:.3f} m rot={rot:+.1f} deg, snapped to the '
                    f'map grid: lin={cm.lin[li]:.3f} m '
                    f'rot={np.degrees(cm.rot[ri]):+.0f} deg')
        if not len(pts):
            self.get_logger().error(
                'NOTHING reachable -- a wrong `lin` is the likely cause, not a '
                'broken map. Read the rail position from /joint_states.')

        now = self.get_clock().now().to_msg()
        cubes = Marker()
        cubes.header.frame_id, cubes.header.stamp = self.world_frame, now
        cubes.ns, cubes.id = f'capability_{self.mode}', 0
        cubes.type, cubes.action = Marker.CUBE_LIST, Marker.ADD
        cubes.scale.x = cubes.scale.y = cubes.scale.z = self.cube
        cubes.color.a = self.alpha
        cubes.pose.orientation.w = 1.0
        cubes.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                        for p in pts]
        cubes.colors = [ColorRGBA(r=r, g=g, b=b, a=self.alpha)
                        for r, g, b in col]

        # An RViz screenshot does not say which arm or gantry pose it is of;
        # without that the picture is unfalsifiable. Put it in the scene.
        text = Marker()
        text.header.frame_id, text.header.stamp = self.world_frame, now
        text.ns, text.id = 'capability_label', 1
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.scale.z = 0.06
        text.color = ColorRGBA(r=1.0, g=0.75, b=0.3, a=1.0)
        text.pose.position.x = float(cm.nodes[:, 0].mean())
        text.pose.position.y = float(cm.nodes[:, 1].mean())
        # clear of the ceiling (z=2.05) so it does not sit inside the room cloud
        text.pose.position.z = float(cm.nodes[:, 2].max() + 0.9)
        text.pose.orientation.w = 1.0
        text.text = label

        self.get_logger().info(
            f"{label.replace(chr(10), ' | ')}  <- {self.map_file}")
        return MarkerArray(markers=[cubes, text])


def main():
    rclpy.init()
    node = CapabilityPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
