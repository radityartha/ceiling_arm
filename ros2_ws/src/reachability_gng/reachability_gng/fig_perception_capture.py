"""One-off capture tool for the paper's perception figure (Fig. 3).

Subscribes to one camera namespace's RGB + instance-segmentation + labels,
blends the mask over the image with per-object label text, projects the
fused world-frame object points from /detected_objects back into the image
(showing the back-projection step), and saves a PNG on Enter.

    ros2 run reachability_gng fig_perception_capture --ros-args -p ns:=rgbd

Press Enter in the terminal to save a snapshot to ~/paper_figures/.
"""
from __future__ import annotations

import json
import os
import threading
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R
from reachability_gng.seg_colorizer import color_for_id


class FigPerceptionCapture(Node):
    def __init__(self):
        super().__init__('fig_perception_capture')
        self.declare_parameter('ns', 'rgbd')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('mask_alpha', 0.45)
        self.declare_parameter('min_conf', 0.35)
        self.declare_parameter(
            'save_dir',
            '~/arm_WS/ceiling_arm/docs/IEEE-conference-Energy_Aware/figure')

        self.ns = self.get_parameter('ns').value
        self.world_frame = self.get_parameter('world_frame').value
        self.cam_frame = self.ns + self.get_parameter('optical_frame_suffix').value
        self.mask_alpha = float(self.get_parameter('mask_alpha').value)
        self.min_conf = float(self.get_parameter('min_conf').value)
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        os.makedirs(self.save_dir, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._rgb = None
        self._seg = None
        self._labels = {}
        self._conf = {}
        self._K = None
        self._objects = None

        self.create_subscription(
            Image, f'/{self.ns}/rgb', self._on_rgb, 1)
        self.create_subscription(
            Image, f'/{self.ns}/seg/instance_segmentation', self._on_seg, 1)
        self.create_subscription(
            String, f'/{self.ns}/seg/instance_segmentation_labels',
            self._on_labels, 1)
        self.create_subscription(
            String, f'/{self.ns}/seg/instance_segmentation_conf',
            self._on_conf, 1)
        self.create_subscription(
            CameraInfo, f'/{self.ns}/camera_info', self._on_info, 1)
        self.create_subscription(
            PoseArray, '/detected_objects', self._on_objects, 1)

        self.get_logger().info(
            f'fig_perception_capture up; ns={self.ns}, saving to {self.save_dir}')

    # ---- subscriber callbacks ----------------------------------------------
    def _decode(self, msg, dtype, channels=1):
        a = np.frombuffer(bytes(msg.data), dtype=dtype)
        cols = msg.step // (np.dtype(dtype).itemsize * channels)
        out = a.reshape(msg.height, cols, channels) if channels > 1 \
            else a.reshape(msg.height, cols)
        return out[:, :msg.width] if channels == 1 else out[:, :msg.width, :]

    def _on_rgb(self, msg):
        try:
            self._rgb = self._decode(msg, np.uint8, channels=3)
        except ValueError:
            pass

    def _on_seg(self, msg):
        try:
            self._seg = self._decode(msg, np.int32)
        except ValueError:
            pass

    def _on_labels(self, msg):
        try:
            raw = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._labels = {
            int(k): str(v).rsplit('/', 1)[-1]
            for k, v in raw.items() if k.isdigit()
        }

    def _on_conf(self, msg):
        try:
            raw = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._conf = {int(k): float(v) for k, v in raw.items() if k.isdigit()}

    def _on_info(self, msg):
        k = msg.k
        self._K = (k[0], k[4], k[2], k[5])

    def _on_objects(self, msg):
        self._objects = [(p.position.x, p.position.y, p.position.z)
                          for p in msg.poses]

    # ---- capture ------------------------------------------------------------
    def save_snapshot(self):
        import cv2

        if self._rgb is None or self._seg is None or not self._labels:
            self.get_logger().warn('no rgb/seg/labels received yet...')
            return
        if self._rgb.shape[:2] != self._seg.shape:
            self.get_logger().warn('rgb/seg size mismatch, skipping')
            return

        # conf defaults to 1.0 for ids with no entry (e.g. isaac GT never
        # publishes confidence), so filtering only bites in yoloe mode.
        kept = {i: l for i, l in self._labels.items()
                if self._conf.get(i, 1.0) >= self.min_conf}
        dropped = self._labels.keys() - kept.keys()
        if dropped:
            self.get_logger().info(
                f'dropped {len(dropped)} low-confidence detection(s) '
                f'(< {self.min_conf}): '
                f'{[(self._labels[i], self._conf.get(i)) for i in dropped]}')

        img = cv2.cvtColor(self._rgb, cv2.COLOR_RGB2BGR).copy()
        overlay = img.copy()
        for inst_id in kept:
            m = self._seg == inst_id
            if not m.any():
                continue
            overlay[m] = color_for_id(inst_id)
        img = cv2.addWeighted(overlay, self.mask_alpha, img, 1 - self.mask_alpha, 0)

        for inst_id, label in kept.items():
            m = self._seg == inst_id
            if not m.any():
                continue
            ys, xs = np.nonzero(m)
            cx, cy = int(xs.mean()), int(ys.mean())
            conf = self._conf.get(inst_id)
            text = f'{label} {conf * 100:.0f}%' if conf is not None else label
            cv2.putText(img, text, (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, text, (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1, cv2.LINE_AA)

        self._draw_backprojection(img, cv2)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.save_dir, f'perception_{self.ns}_{timestamp}.png')
        cv2.imwrite(path, img)
        self.get_logger().info(f'saved: {path}')

    def _draw_backprojection(self, img, cv2):
        if self._K is None or not self._objects:
            self.get_logger().warn(
                'no camera_info / detected_objects yet -- saved without '
                'back-projected point')
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.cam_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'no tf {self.world_frame}->{self.cam_frame} yet -- saved '
                'without back-projected point')
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])
        fx, fy, cx, cy = self._K
        h, w = img.shape[:2]

        for wx, wy, wz in self._objects:
            p_cam = R.T @ (np.array([wx, wy, wz]) - T)
            if p_cam[2] <= 0.05:
                continue
            u = fx * p_cam[0] / p_cam[2] + cx
            v = fy * p_cam[1] / p_cam[2] + cy
            if not (0 <= u < w and 0 <= v < h):
                continue
            u, v = int(u), int(v)
            cv2.drawMarker(img, (u, v), (0, 255, 255),
                            markerType=cv2.MARKER_CROSS, markerSize=18,
                            thickness=2)
            cv2.circle(img, (u, v), 10, (0, 255, 255), 2)


def main():
    rclpy.init()
    node = FigPerceptionCapture()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print('Press Enter to save a snapshot, Ctrl+C to quit.')
    try:
        while rclpy.ok():
            input()
            node.save_snapshot()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
