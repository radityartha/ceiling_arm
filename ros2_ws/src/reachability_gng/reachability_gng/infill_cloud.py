"""Predict sensor-occluded surface cells from the surrounding voxels.

The two static RGBD cameras cannot see the surface behind/under a standing
object, so its shadow reads as empty -> holey octomap. But that shadow is
ENCLOSED by real, sensed surface voxels at the SAME height (the table around the
object). So we can infer it WITHOUT reading any ground truth: purely from the
sensor's own occupied voxels, an empty cell that is fully surrounded (at its own
z-level) by occupied cells is very likely part of the same continuous surface and
is filled. This is a live, per-frame prediction (2D binary hole-fill per z-slice
of the fused world voxel grid) -- not a saved/hardcoded prior.

    /<ns>/collision_cloud (optical, per cam)  --fuse->  world voxel grid
        --per z-slice: fill ENCLOSED empty cells-->  /infill_cloud (world)

`/infill_cloud` carries ONLY the predicted (inferred) points; feed it to MoveIt's
octomap as an extra world-frame source, or view it in RViz over the real cloud.

    ros2 run reachability_gng infill_cloud
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


def _enclosed_holes(occ):
    """Empty cells not reachable from the border through empty space => holes
    fully enclosed by occupied cells. Pure-numpy flood fill (4-connectivity)."""
    free = ~occ
    reach = np.zeros_like(occ)
    reach[0, :] |= free[0, :]
    reach[-1, :] |= free[-1, :]
    reach[:, 0] |= free[:, 0]
    reach[:, -1] |= free[:, -1]
    while True:
        prev = reach.sum()
        g = reach.copy()
        g[1:, :] |= reach[:-1, :]
        g[:-1, :] |= reach[1:, :]
        g[:, 1:] |= reach[:, :-1]
        g[:, :-1] |= reach[:, 1:]
        reach = g & free
        if reach.sum() == prev:
            break
    return free & ~reach


def _dilate(occ):
    """1-cell 4-neighbour dilation (seals 1-cell gaps so holes stay enclosed)."""
    g = occ.copy()
    g[1:, :] |= occ[:-1, :]
    g[:-1, :] |= occ[1:, :]
    g[:, 1:] |= occ[:, :-1]
    g[:, :-1] |= occ[:, 1:]
    return g


def _close(occ, k):
    """Morphological closing: dilate k then erode k. Fills SMALL gaps/holes
    (width <= 2k cells) whether or not they are enclosed, but a small k cannot
    bridge large open areas -- so it targets occlusion-shadow specks without
    flooding genuinely empty space."""
    g = occ
    for _ in range(k):
        g = _dilate(g)
    for _ in range(k):                 # erode = complement of dilate of complement
        g = ~_dilate(~g)
    return g


class InfillCloud(Node):
    def __init__(self):
        super().__init__('infill_cloud')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('cloud_topic_suffix', 'collision_cloud')
        self.declare_parameter('voxel_size', 0.03)      # matches octomap res
        self.declare_parameter('rate', 3.0)             # Hz to recompute
        self.declare_parameter('min_cells_per_level', 30)  # skip sparse slices
        self.declare_parameter('seal_gaps', True)       # dilate before fill
        self.declare_parameter('max_hole_cells', 0)     # 0 = no cap on hole size
        # Morphological closing to fill SMALL occlusion-shadow specks that are not
        # strictly enclosed (the table shadows stay "open" via thin channels, so
        # binary_fill_holes misses them). close_iters cells: fills gaps up to
        # ~2*close_iters wide; keep it small (1-2) so it can't bridge the room.
        self.declare_parameter('close_iters', 1)
        # Which slice orientations to infer surfaces from: 2=z (horizontal floors/
        # tables), 0=x and 1=y (vertical planes -> walls / boards). A cell is filled
        # if it is an enclosed hole in ANY enabled orientation, so a hole in a wall
        # is inferred from the wall voxels around it just like a table hole is.
        self.declare_parameter('fill_axes', [0, 1, 2])

        self.world_frame = self.get_parameter('world_frame').value
        self.suffix = self.get_parameter('cloud_topic_suffix').value
        self.res = float(self.get_parameter('voxel_size').value)
        self.min_cells = int(self.get_parameter('min_cells_per_level').value)
        self.seal = bool(self.get_parameter('seal_gaps').value)
        self.max_hole = int(self.get_parameter('max_hole_cells').value)
        self.close_iters = int(self.get_parameter('close_iters').value)
        self.axes = [int(a) for a in self.get_parameter('fill_axes').value]
        nss = list(self.get_parameter('camera_namespaces').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._latest = {ns: None for ns in nss}
        for ns in nss:
            self.create_subscription(
                PointCloud2, f'/{ns}/{self.suffix}',
                lambda m, ns=ns: self._on_cloud(ns, m), 1)
        self.pub = self.create_publisher(PointCloud2, '/infill_cloud', 1)
        self.create_timer(1.0 / float(self.get_parameter('rate').value),
                          self._tick)
        self.get_logger().info(
            f'infill_cloud up; cameras={nss}, voxel={self.res} m -- filling '
            'ENCLOSED empty cells per z-slice (sensor-inferred, no prior)')

    def _on_cloud(self, ns, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        a = np.frombuffer(bytes(msg.data), np.uint8)
        a = a.reshape(-1, msg.point_step)[:, :12].copy().view(np.float32)
        a = a[np.isfinite(a).all(axis=1)]
        if a.size == 0:
            self._latest[ns] = np.zeros((0, 3), np.float32)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        self._latest[ns] = a @ R.T + np.array([t.x, t.y, t.z])

    def _tick(self):
        clouds = [c for c in self._latest.values() if c is not None and len(c)]
        if not clouds:
            return
        P = np.concatenate(clouds, axis=0)
        vi = np.floor(P / self.res).astype(np.int64)   # integer voxel indices
        idx = [self._fill_axis(vi, ax) for ax in self.axes]
        idx = [a for a in idx if len(a)]
        if idx:
            F = np.unique(np.concatenate(idx, axis=0), axis=0)   # dedup across axes
            out = ((F + 0.5) * self.res).astype(np.float32)
        else:
            out = np.zeros((0, 3), np.float32)
        self.pub.publish(self._make_cloud(out))

    def _fill_axis(self, vi, ax):
        """Enclosed-hole voxels found by slicing perpendicular to axis `ax`
        (ax=2 -> horizontal z-slices; ax=0/1 -> vertical x/y-slices). Returns
        (M,3) int voxel indices. A hole in a wall is inferred from the wall's
        own voxels exactly like a hole in a table is."""
        oa, ob = [i for i in (0, 1, 2) if i != ax]
        a = vi[:, oa]
        b = vi[:, ob]
        s = vi[:, ax]
        a0, b0 = a.min(), b.min()
        ga, gb = a - a0, b - b0
        na, nb = int(ga.max()) + 1, int(gb.max()) + 1
        if na * nb > 4_000_000:            # safety: skip pathological extents
            return np.zeros((0, 3), np.int64)
        out = []
        for sv in np.unique(s):
            sel = s == sv
            if sel.sum() < self.min_cells:
                continue
            occ = np.zeros((na, nb), bool)
            occ[ga[sel], gb[sel]] = True
            sealed = _dilate(occ) if self.seal else occ
            enc = _enclosed_holes(sealed) & ~occ
            if self.max_hole > 0 and enc.sum() > self.max_hole:
                enc = np.zeros_like(enc)    # too big to be a surface shadow
            # small occlusion-shadow specks (not strictly enclosed): closing
            small = (_close(occ, self.close_iters) & ~occ
                     if self.close_iters > 0 else np.zeros_like(occ))
            holes = enc | small
            if not holes.any():
                continue
            ha, hb = np.nonzero(holes)
            k = np.empty((len(ha), 3), np.int64)
            k[:, ax] = sv
            k[:, oa] = ha + a0
            k[:, ob] = hb + b0
            out.append(k)
        return np.concatenate(out, axis=0) if out else np.zeros((0, 3), np.int64)

    def _make_cloud(self, pts):
        n = len(pts)
        arr = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        if n:
            arr['x'], arr['y'], arr['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
        msg = PointCloud2()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.data = arr.tobytes()
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = InfillCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
