"""Route the segmentation source consumed by the perception pipeline.

The whole RGBD pipeline (object_localizer, reachability_cloud, collision_cloud,
object_collision, seg_cloud) consumes a generic instance-segmentation contract
per camera namespace `ns`:

    /<ns>/instance_segmentation         sensor_msgs/Image  (32SC1, id per pixel)
    /<ns>/instance_segmentation_labels  std_msgs/String    (JSON {id: label})

This node republishes that contract on a NEUTRAL topic pair that the consumers
are remapped to (see perception.launch.py):

    /<ns>/seg/instance_segmentation
    /<ns>/seg/instance_segmentation_labels

...sourcing it from one of two producers, switchable AT RUNTIME (no relaunch):

    source == 'isaac'  -> relay Isaac's ground-truth /<ns>/instance_segmentation*
                          (default; downstream behaves exactly as before)
    source == 'yoloe'  -> run open-vocabulary YOLOE segmentation on /<ns>/rgb

Switch live by publishing the source name (and, optionally, the open-vocab
prompt list) on:

    ros2 topic pub -1 /seg_source std_msgs/String "data: yoloe"
    ros2 topic pub -1 /seg_source std_msgs/String "data: isaac"
    ros2 topic pub -1 /seg_prompts std_msgs/String "data: bottle,cup,box"

torch/ultralytics/cv2 are imported LAZILY the first time YOLOE is activated, so
the default 'isaac' relay path adds no heavy dependency. For a real camera, set
source:=yoloe and point camera_namespaces / rgb_topic at the RealSense stream;
the same node then feeds the identical contract downstream.

    ros2 run reachability_gng seg_router
    ros2 run reachability_gng seg_router --ros-args -p source:=yoloe \
        -p prompts:="bottle,cup,box"
"""
from __future__ import annotations

import colorsys
import json

import numpy as np


def name_color(r, g, b):
    """Map an (r,g,b) 0-255 mean color to a basic color name (HSV-based)."""
    h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    if v < 0.20:
        return 'black'
    if s < 0.20:
        return 'white' if v > 0.70 else 'gray'
    deg = h * 360.0
    if deg < 15 or deg >= 345:
        return 'red'
    if deg < 45:
        return 'brown' if v < 0.55 else 'orange'
    if deg < 70:
        return 'yellow'
    if deg < 170:
        return 'green'
    if deg < 200:
        return 'cyan'
    if deg < 260:
        return 'blue'
    if deg < 320:
        return 'purple'
    return 'pink'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SegRouter(Node):
    def __init__(self):
        super().__init__('seg_router')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('source', 'isaac')          # 'isaac' | 'yoloe'
        # Topic templates ({ns} is filled per camera). Leading '/' added if absent.
        self.declare_parameter('rgb_topic', '{ns}/rgb')
        self.declare_parameter('isaac_seg_topic', '{ns}/instance_segmentation')
        self.declare_parameter('isaac_labels_topic',
                               '{ns}/instance_segmentation_labels')
        self.declare_parameter('out_seg_topic', '{ns}/seg/instance_segmentation')
        self.declare_parameter('out_labels_topic',
                               '{ns}/seg/instance_segmentation_labels')
        # Per-instance YOLOE confidence {id: value}, parallel to out_labels_topic
        # (display only -- consumers still key off out_labels_topic).
        self.declare_parameter('out_conf_topic',
                               '{ns}/seg/instance_segmentation_conf')
        # YOLOE settings
        self.declare_parameter('model_path', 'yoloe-11s-seg.pt')
        self.declare_parameter('prompts', 'bottle,cup,box,scissors,mug,bowl')   # comma-separated
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')               # '' -> auto
        self.declare_parameter('max_rate', 10.0)           # inference Hz cap
        # Prefix each object's label with its measured dominant color, e.g.
        # 'yellow box' -> lets HRI/targeting distinguish same-class objects by
        # color deterministically (measured from RGB, not guessed by the model).
        self.declare_parameter('label_color', True)

        self.source = str(self.get_parameter('source').value).strip().lower()
        self.prompts = self._split(str(self.get_parameter('prompts').value))
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = str(self.get_parameter('device').value).strip()
        self.max_rate = float(self.get_parameter('max_rate').value)
        self.label_color = bool(self.get_parameter('label_color').value)
        self.model_path = str(self.get_parameter('model_path').value)
        nss = list(self.get_parameter('camera_namespaces').value)

        self._model = None                 # lazy-loaded YOLOE
        self._classes_dirty = True         # (re)apply prompts on next inference
        self._last_infer = {ns: 0.0 for ns in nss}

        def _t(name):
            v = str(self.get_parameter(name).value)
            return v if v.startswith('/') else '/' + v

        self._seg_pub, self._lbl_pub, self._conf_pub = {}, {}, {}
        for ns in nss:
            self._seg_pub[ns] = self.create_publisher(
                Image, _t('out_seg_topic').format(ns=ns), 1)
            self._lbl_pub[ns] = self.create_publisher(
                String, _t('out_labels_topic').format(ns=ns), 1)
            self._conf_pub[ns] = self.create_publisher(
                String, _t('out_conf_topic').format(ns=ns), 1)
            # Isaac ground-truth relay inputs
            self.create_subscription(
                Image, _t('isaac_seg_topic').format(ns=ns),
                lambda m, ns=ns: self._on_isaac_seg(ns, m), 1)
            self.create_subscription(
                String, _t('isaac_labels_topic').format(ns=ns),
                lambda m, ns=ns: self._on_isaac_labels(ns, m), 1)
            # YOLOE input
            self.create_subscription(
                Image, _t('rgb_topic').format(ns=ns),
                lambda m, ns=ns: self._on_rgb(ns, m), 1)

        # Runtime control
        self.create_subscription(String, '/seg_source', self._on_source, 10)
        self.create_subscription(String, '/seg_prompts', self._on_prompts, 10)
        self.get_logger().info(
            f'seg_router up; source={self.source}, cameras={nss}, '
            f'prompts={self.prompts}')

    # ---- runtime control ----------------------------------------------------
    def _on_source(self, msg):
        s = msg.data.strip().lower()
        if s not in ('isaac', 'yoloe') or s == self.source:
            return
        self.source = s
        self.get_logger().info(f'segmentation source -> {s}')

    @staticmethod
    def _split(s):
        return [p.strip() for p in s.split(',') if p.strip()]

    def _on_prompts(self, msg):
        prompts = self._split(msg.data)
        if not prompts or prompts == self.prompts:
            return
        self.prompts = prompts
        self._classes_dirty = True
        self.get_logger().info(f'YOLOE prompts -> {prompts}')

    # ---- Isaac ground-truth relay ------------------------------------------
    def _on_isaac_seg(self, ns, msg):
        if self.source == 'isaac':
            self._seg_pub[ns].publish(msg)

    def _on_isaac_labels(self, ns, msg):
        if self.source == 'isaac':
            self._lbl_pub[ns].publish(msg)

    # ---- YOLOE open-vocab path ---------------------------------------------
    def _ensure_model(self):
        """Load YOLOE + apply text prompts (lazy; heavy imports live here)."""
        if self._model is None:
            from ultralytics import YOLOE
            if not self.device:
                try:
                    import torch
                    self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
                except ImportError:
                    self.device = 'cpu'
            self.get_logger().info(
                f'loading YOLOE {self.model_path} on {self.device} ...')
            self._model = YOLOE(self.model_path)
        if self._classes_dirty:
            self._model.set_classes(
                self.prompts, self._model.get_text_pe(self.prompts))
            self._classes_dirty = False
        return self._model

    def _on_rgb(self, ns, msg):
        if self.source != 'yoloe':
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_rate > 0 and now - self._last_infer[ns] < 1.0 / self.max_rate:
            return
        self._last_infer[ns] = now
        try:
            self._infer(ns, msg)
        except Exception as e:   # keep the node alive across a bad frame/model
            self.get_logger().warn(f'YOLOE inference failed on {ns}: {e}')

    def _infer(self, ns, msg):
        import cv2
        img = self._rgb_to_bgr(msg)
        model = self._ensure_model()
        res = model.predict(img, conf=self.conf, imgsz=self.imgsz,
                            device=self.device, retina_masks=True,
                            verbose=False)[0]

        h, w = img.shape[:2]
        seg = np.zeros((h, w), np.int32)
        id_labels = {}
        id_conf = {}
        if res.masks is not None and len(res.masks) > 0:
            masks = res.masks.data.cpu().numpy()            # (N, h', w') in [0,1]
            cls = res.boxes.cls.cpu().numpy().astype(int)
            conf = res.boxes.conf.cpu().numpy()              # (N,) in [0,1]
            names = res.names                                # {idx: name}
            for i in range(masks.shape[0]):
                inst_id = 2 + i                              # id>1 == object
                m = masks[i] > 0.5
                if m.shape != (h, w):
                    m = cv2.resize(m.astype(np.uint8), (w, h),
                                   interpolation=cv2.INTER_NEAREST) > 0
                seg[m] = inst_id
                cls_name = str(names.get(int(cls[i]), cls[i]))
                if self.label_color and m.any():
                    b, g, r = img[m].mean(axis=0)   # img is BGR
                    cls_name = f'{name_color(r, g, b)} {cls_name}'
                id_labels[inst_id] = cls_name
                id_conf[inst_id] = round(float(conf[i]), 3)

        self._publish_seg(ns, seg, msg.header)
        lbl = String()
        lbl.data = json.dumps({str(k): v for k, v in id_labels.items()})
        self._lbl_pub[ns].publish(lbl)
        self._conf_pub[ns].publish(
            String(data=json.dumps({str(k): v for k, v in id_conf.items()})))

    @staticmethod
    def _rgb_to_bgr(msg):
        """Decode a color Image message to an (H, W, 3) BGR uint8 array."""
        import cv2
        cols = msg.step // 3
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(
            msg.height, cols, 3)[:, :msg.width, :]
        enc = (msg.encoding or 'rgb8').lower()
        if enc.startswith('rgb'):
            return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        return a.copy()   # already bgr8

    def _publish_seg(self, ns, seg, header):
        """Publish an int32 label image as a 32SC1 Image (Isaac-compatible)."""
        h, w = seg.shape
        out = Image()
        out.header = header
        out.height = h
        out.width = w
        out.encoding = '32SC1'
        out.is_bigendian = 0
        out.step = w * 4
        out.data = np.ascontiguousarray(seg, np.int32).tobytes()
        self._seg_pub[ns].publish(out)


def main():
    rclpy.init()
    node = SegRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
