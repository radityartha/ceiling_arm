"""Regressions for the F5/D camera-stop diagnosis; no ROS node or hardware."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from reachability_gng.object_localizer import ObjectLocalizer, fuse


def localizer(cameras=('rgbd',)):
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.seconds = 10.0
    def now():
        nanos = round(node.seconds * 1e9)
        return SimpleNamespace(nanoseconds=nanos, to_msg=lambda: Time(
            sec=nanos // 1000000000, nanosec=nanos % 1000000000))
    node.get_clock = lambda: SimpleNamespace(now=now)
    node._dets = {ns: [] for ns in cameras}
    node._dets_stamp = {ns: None for ns in cameras}
    node._last_pair_stamp = {ns: None for ns in cameras}
    node._K = {ns: (100., 100., 0., 0.) for ns in cameras}
    node._labels = {ns: {2: 'gray bottle'} for ns in cameras}
    tr = SimpleNamespace(translation=SimpleNamespace(x=0., y=0., z=0.), rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *a: SimpleNamespace(transform=tr))
    node.world_frame = 'world'; node.suffix = '_camera_optical'
    node.min_depth = .1; node.max_depth = 12.; node.min_pixels = 20
    node.track_ttl = 1.5; node.confirm_frames = 3; node.fuse_radius = .1
    node.track_match_radius = .12; node.class_cost = .10; node.track_smooth = .5
    node.track_merge_dist = .15; node.vote_decay = .9; node.label_hysteresis_margin = 2
    node._tracks = {}; node._next_tid = 0
    node._label_conf = {}; node.target_label = 'gray bottle'; node.target_id = -1
    node.published = {}
    for name in ('pose_pub', 'target_pub', 'marker_pub', 'tracks_pub', 'box_pub'):
        node.published[name] = []
        setattr(node, name, SimpleNamespace(publish=node.published[name].append))
    return node


def pair(node, stamp, ns='rgbd', empty=False):
    depth, seg = Image(), Image()
    for image in (depth, seg):
        image.height = 1; image.width = 20; image.step = 80
        nanos = round(stamp * 1e9)
        image.header.stamp.sec = nanos // 1000000000
        image.header.stamp.nanosec = nanos % 1000000000
    depth.encoding = '32FC1'; depth.data = np.ones(20, np.float32).tobytes()
    seg.encoding = '32SC1'; seg.data = np.full(20, 0 if empty else 2, np.int32).tobytes()
    node._on_pair(ns, depth, seg)


def tick(node, now):
    node.seconds = now
    node._publish()
    return json.loads(node.published['tracks_pub'][-1].data)


def confirmed(node):
    out = None
    for stamp in (10., 10.5, 11.):
        pair(node, stamp); out = tick(node, stamp)
    assert len(out) == 1
    return out[0]['tid']


def test_single_pair_cannot_be_confirmed_by_repeated_timer_ticks():
    node = localizer(); pair(node, 10.)
    for now in (10., 10.5, 11.):
        assert tick(node, now) == []
    assert node._tracks[0]['hits'] == 1


def test_three_distinct_pairs_confirm_and_preserve_identity():
    node = localizer(); tid = confirmed(node)
    pair(node, 11.5); out = tick(node, 11.5)
    assert out[0]['tid'] == tid
    assert node._tracks[tid]['hits'] == 4


def test_camera_stop_expires_confirmed_track_after_measurement_ttl():
    node = localizer(); tid = confirmed(node)
    assert tick(node, 12.)[0]['tid'] == tid
    assert tick(node, 12.500001) == []
    assert not node._tracks
    assert node.published['pose_pub'][-1].poses == []
    assert json.loads(node.published['box_pub'][-1].data) == []


def test_publication_delay_does_not_extend_measurement_ttl():
    node = localizer()
    for stamp in (10., 10.2, 10.4):
        pair(node, stamp); tick(node, stamp + .4)
    assert node._tracks[0]['last'] == pytest.approx(10.4)
    assert tick(node, 11.900001) == []


def test_duplicate_or_out_of_order_pair_cannot_create_another_hit():
    node = localizer(); pair(node, 10.); tick(node, 10.)
    for stamp in (10., 9.9):
        pair(node, stamp); assert tick(node, 10.5) == []
    assert node._tracks[0]['hits'] == 1


def test_empty_new_pair_does_not_refresh_cached_object():
    node = localizer(); confirmed(node)
    pair(node, 11.5, empty=True); tick(node, 11.5)
    assert node._tracks[0]['last'] == 11.
    assert tick(node, 12.500001) == []


def test_pair_that_aged_out_before_publish_is_not_used():
    node = localizer(); pair(node, 10.)
    assert tick(node, 11.500001) == []
    assert not node._tracks


def test_new_frame_after_ttl_starts_confirmation_over():
    node = localizer(); old = confirmed(node)
    pair(node, 13.); assert tick(node, 13.) == []
    assert old not in node._tracks
    assert node._tracks[old + 1]['hits'] == 1


def test_stale_second_camera_cannot_bias_new_camera_fusion():
    node = localizer(('rgbd', 'rgbd2'))
    node._dets['rgbd'] = [('bottle', np.array([.05, 0., 1.]), 1.)]
    node._dets_stamp['rgbd'] = 10.
    node._dets['rgbd2'] = [('bottle', np.array([0., 0., 1.]), 1.)]
    node._dets_stamp['rgbd2'] = 8.
    out = node._consume_detections()
    assert len(out) == 1
    np.testing.assert_allclose(out[0][1], [.05, 0., 1.])
    assert out[0][3] == 10.
    assert node._consume_detections() == []


def test_later_arriving_second_camera_does_not_rewind_track_age():
    node = localizer()
    node._update_tracks([('bottle', np.zeros(3), 1., 10.)])
    node._update_tracks([('bottle', np.zeros(3), 1., 9.9)])
    assert node._tracks[0]['last'] == 10.


def test_fusion_keeps_latest_contributing_measurement_time():
    dets = [('bottle', np.array([0., 0., 1.]), 1.), ('bottle', np.array([.04, 0., 1.]), 1.2)]
    out = fuse(dets, .1, [10., 10.2])
    np.testing.assert_allclose(out[0][1], [.02, 0., 1.])
    assert out[0][2:] == (1.2, 10.2)
    assert len(fuse(dets, .1)[0]) == 3  # Existing geometry API remains compatible.
