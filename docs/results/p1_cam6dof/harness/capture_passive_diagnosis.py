#!/usr/bin/env python3
"""Passive diagnosis only: retain sampled raw RGB/depth/mask and all track/label messages.
No publishers, commands, detector changes or replacement of calibration data.
"""
import argparse, collections, json, time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from reachability_gng.object_localizer import deproject, quat_to_R


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def decode(m):
    types = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), '32FC1': ('f4', 1),
             '32SC1': ('i4', 1), '16UC1': ('u2', 1)}
    dtype, channels = types[m.encoding]
    dt = np.dtype(dtype).newbyteorder('>' if m.is_bigendian else '<')
    a = np.frombuffer(bytes(m.data), dt).reshape(m.height, m.step // dt.itemsize)
    return a[:, :m.width * channels].reshape(m.height, m.width, channels).copy() if channels > 1 else a[:, :m.width].copy()


class Recorder(Node):
    def __init__(self, out):
        super().__init__('cam6dof_passive_diagnosis')
        self.out = out
        self.buf = {ns: {k: collections.deque(maxlen=24) for k in ('rgb', 'depth', 'rawdepth', 'debug')} for ns in ('rgbd', 'rgbd2')}
        self.info, self.labels, self.counts, self.last, self.frames = {}, {}, {}, {}, []
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.events = (out / 'events.jsonl').open('w')
        self.active = False
        for ns in self.buf:
            for kind, topic in {'rgb': f'/{ns}/rgb', 'depth': f'/{ns}/depth',
                                'rawdepth': f'/{ns}/{ns}/aligned_depth_to_color/image_raw',
                                'debug': f'/{ns}/seg/debug_image'}.items():
                self.create_subscription(Image, topic, lambda m, n=ns, k=kind: self.on_image(n, k, m), qos_profile_sensor_data)
            self.create_subscription(Image, f'/{ns}/seg/instance_segmentation', lambda m, n=ns: self.on_mask(n, m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/{ns}/camera_info', lambda m, n=ns: self.on_info(n, m), qos_profile_sensor_data)
            for kind, topic in {'labels': f'/{ns}/seg/instance_segmentation_labels',
                                'conf': f'/{ns}/seg/instance_segmentation_conf',
                                'tracks': f'/loc_{ns}/tracks'}.items():
                self.create_subscription(String, topic, lambda m, n=ns, k=kind: self.on_string(n, k, m), qos_profile_sensor_data)

    def event(self, ns, kind, **fields):
        if self.active:
            self.events.write(json.dumps({'receipt': time.time(), 'camera': ns, 'kind': kind, **fields}) + '\n')
            key = ns + '/' + kind; self.counts[key] = self.counts.get(key, 0) + 1

    def on_info(self, ns, m):
        self.info[ns] = {'K': list(m.k), 'D': list(m.d), 'frame': m.header.frame_id, 'width': m.width, 'height': m.height, 'stamp': stamp(m)}

    def on_image(self, ns, kind, m):
        self.buf[ns][kind].append(m)
        self.event(ns, kind, stamp=stamp(m), encoding=m.encoding, width=m.width, height=m.height, step=m.step, frame=m.header.frame_id)

    def on_string(self, ns, kind, m):
        try: data = json.loads(m.data)
        except ValueError: data = m.data
        if kind == 'labels': self.labels[ns] = {'receipt': time.time(), 'payload': data}
        self.event(ns, kind, payload=data)

    def on_mask(self, ns, m):
        self.event(ns, 'mask', stamp=stamp(m), encoding=m.encoding, receipt_lag_s=time.time() - stamp(m))
        if not self.active or time.monotonic() - self.last.get(ns, -100) < .5: return
        self.last[ns] = time.monotonic()
        rec = {'camera': ns, 'mask_stamp': stamp(m), 'mask_receipt': time.time(), 'mask_encoding': m.encoding,
               'labels_latest': self.labels.get(ns), 'label_note': 'String labels have no header; latest receipt association is not exact synchronization.'}
        arrays = {'mask': decode(m)}
        for kind in ('rgb', 'depth', 'rawdepth'):
            if self.buf[ns][kind]:
                msg = min(self.buf[ns][kind], key=lambda v: abs(stamp(v) - stamp(m)))
                rec[kind] = {'stamp': stamp(msg), 'delta_mask_s': stamp(msg) - stamp(m), 'encoding': msg.encoding,
                             'frame': msg.header.frame_id, 'step': msg.step, 'is_bigendian': msg.is_bigendian}
                arrays[kind] = decode(msg)
        rec['camera_info'] = self.info.get(ns)
        try:
            tr = self.tf.lookup_transform('world', ns + '_camera_optical', rclpy.time.Time()).transform
            t = tr.translation; q = tr.rotation
            R = quat_to_R(q.x, q.y, q.z, q.w); T = np.array([t.x, t.y, t.z])
            rec['TF'] = {'translation': T.tolist(), 'quaternion_xyzw': [q.x, q.y, q.z, q.w], 'R': R.tolist()}
        except Exception as e: rec['TF_error'] = str(e)
        rec['instances'] = []
        if 'depth' in arrays and rec.get('camera_info') and rec.get('TF'):
            d, seg = arrays['depth'], arrays['mask']
            if rec['depth']['encoding'] != '32FC1': rec['depth_error'] = 'Metric depth must be 32FC1; no guessed scaling applied'
            elif d.shape != seg.shape: rec['depth_error'] = 'Depth/mask shapes differ'
            else:
                K = np.array(rec['camera_info']['K']).reshape(3, 3)
                for ident in np.unique(seg):
                    if ident <= 1: continue
                    mask = seg == ident; yy, xx = np.nonzero(mask & np.isfinite(d) & (d > .1) & (d < 12.0))
                    item = {'id': int(ident), 'mask_pixels': int(mask.sum()), 'valid_depth_pixels': len(xx)}
                    if len(xx):
                        world = deproject(xx, yy, d[yy, xx], K[0, 0], K[1, 1], K[0, 2], K[1, 2]) @ R.T + T
                        item.update(median_world=np.median(world, axis=0).tolist(), p5_p95_world=np.percentile(world, [5, 95], axis=0).tolist(),
                                    bbox_uv=[int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())],
                                    fraction_world_z_below_surface=float(np.mean(world[:, 2] < .140)),
                                    depth_m_p5_p50_p95=np.percentile(d[yy, xx], [5, 50, 95]).tolist())
                    rec['instances'].append(item)
        name = ns + '_' + str(len(self.frames)).zfill(3)
        np.savez_compressed(self.out / (name + '.npz'), **arrays)
        rec['raw_arrays'] = name + '.npz'
        if 'rgb' in arrays:
            rgb = arrays['rgb']; bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rec['rgb']['encoding'] == 'rgb8' else rgb
            overlay = bgr.copy()
            colors = np.random.default_rng(0).integers(40, 255, (256, 3), dtype=np.uint8)
            for ident in np.unique(arrays['mask']):
                if ident > 1:
                    mm = arrays['mask'] == ident; overlay[mm] = (.55 * overlay[mm] + .45 * colors[int(ident) % 256]).astype(np.uint8)
            cv2.imwrite(str(self.out / (name + '_rgb.jpg')), bgr)
            cv2.imwrite(str(self.out / (name + '_mask.jpg')), overlay)
        self.frames.append(rec)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('output'); ap.add_argument('--seconds', type=float, default=15); a = ap.parse_args()
    out = Path(a.output); out.mkdir(parents=True, exist_ok=True); assert not (out / 'events.jsonl').exists(), 'Never overwrite diagnosis'
    rclpy.init(); node = Recorder(out)
    warm = time.monotonic()
    while time.monotonic() - warm < 3: rclpy.spin_once(node, timeout_sec=.1)
    node.active = True; begin = time.time(); start = time.monotonic()
    while time.monotonic() - start < a.seconds: rclpy.spin_once(node, timeout_sec=.05)
    node.active = False; node.events.close()
    result = {'start': begin, 'end': time.time(), 'requested_seconds': a.seconds, 'counts': node.counts, 'frames': node.frames,
              'purpose': 'Passive F5/D diagnosis; original calibration raw, selectors and scores unchanged',
              'sampling': 'Arrays sampled at most2Hz/camera, nearest retained RGB/depth by header timestamp; actual deltas retained, not claimed exact synchronization',
              'geometry': {'xy': [.6497, -.4779], 'surface_z': .140, 'bottle_H': .165, 'bottle_diameter': .065}}
    (out / 'diagnosis.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'counts': node.counts, 'snapshots': len(node.frames)}, indent=2))
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
