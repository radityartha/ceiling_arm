"""Interactive target picker for reach_fusion (like pick_cli).

Lists live objects by Isaac prim name obj_N (ground truth, via ObjnLocalizer)
with world positions, and sends the chosen obj_N / class label / index to
/reach_fusion/set_target -- reach_fusion switches target without a restart.

    ros2 run reachability_gng reach_fusion     # backend
    ros2 run reachability_gng target_cli        # this menu (own terminal)
"""
from __future__ import annotations

import json
import re
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

from reachability_gng.objn_localizer import ObjnLocalizer
# Reuse the deterministic natural-language parser (no LLM, Rule 5) from pick_cli
# so 'please bring a teddy bear' works here too -- but resolved to a STABLE track
# handle (#tid), not a per-frame label.
from reachability_gng.pick_cli import parse_object, split_color, _alias_group

_OBJN_RE = re.compile(r'^obj_\d+$')
SEARCH_SECS = 10.0      # re-scan window for a requested object before giving up


class TargetCli(Node):
    def __init__(self):
        super().__init__('target_cli')
        self.declare_parameter('set_target_topic', '/reach_fusion/set_target')
        self._lock = threading.Lock()
        self.poses = np.empty((0, 3))
        self.tracks = []            # [{tid,label,x,y,z}] stable-identity handles
        self.last_sent = None
        self.objn = ObjnLocalizer(self)
        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        tq = QoSProfile(depth=1)
        tq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/detected_objects/tracks',
                                 self._on_tracks, tq)
        self.pub = self.create_publisher(
            String, self.get_parameter('set_target_topic').value, 1)
        self.exec_pub = self.create_publisher(Empty, '/reach_fusion/execute', 1)

    def _on_objects(self, msg):
        with self._lock:
            self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                                   for p in msg.poses]) if msg.poses \
                else np.empty((0, 3))

    def _on_tracks(self, msg):
        try:
            arr = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.tracks = arr

    def track_list(self):
        with self._lock:
            return list(self.tracks)

    def tracks_publisher_count(self):
        return self.count_publishers('/detected_objects/tracks')

    def objn_map(self):
        with self._lock:
            poses = self.poses.copy()
        return self.objn.map(poses), len(poses)

    def send(self, value):
        self.pub.publish(String(data=value))
        self.last_sent = value

    def execute(self):
        self.exec_pub.publish(Empty())


def _fetch(node: TargetCli, sentence):
    """Natural-language fetch -> a STABLE track handle (#tid).

    Deterministically parses the object phrase ('please bring a teddy bear' ->
    'teddy bear'), then matches it against the current confirmed tracks by class
    (color optional). One match -> targets its #tid straight away; several
    matches of the same class -> lists the candidate #ids so the user picks the
    exact object (never auto-guesses which of two identical objects). Identity is
    the persistent track, so this works under Isaac GT and YOLOE alike.
    """
    phrase = parse_object(sentence)
    if not phrase:
        print('  ! no object name found in that request')
        return
    color, cls = split_color(phrase)
    aliases = _alias_group(cls or phrase)

    def _match(tracks):
        out = []
        for t in tracks:
            lab = str(t.get('label', '')).lower()
            if lab and any(a in lab for a in aliases) and (not color or color in lab):
                out.append(t)
        return out

    # Perception can miss the object on the FIRST frame (label flicker / a track
    # not confirmed yet), so don't reject on a single snapshot -- re-scan the
    # live tracks for a few seconds before giving up, like pick_cli does. This
    # avoids rejecting a request just because of a transient early-detection gap.
    matches = _match(node.track_list())
    if not matches:
        print(f"  -> looking for '{phrase}' (re-checking tracks for {SEARCH_SECS:g}s)...")
        deadline = time.time() + SEARCH_SECS
        while time.time() < deadline and rclpy.ok():
            time.sleep(0.5)
            matches = _match(node.track_list())
            if matches:
                break
    if not matches:
        seen = sorted({str(t.get('label', '')) for t in node.track_list()})
        print(f"  ! no '{phrase}' among current tracks after {SEARCH_SECS:g}s "
              f"(seen: {seen or '(none)'})")
        return
    if len(matches) == 1:
        t = matches[0]
        node.send(f"#{t['tid']}")
        print(f"  -> understood '{phrase}' = track #{t['tid']} ({t['label']}); "
              "type g to execute the approach")
        return
    # Several identical-class objects match. Default to the highest-confidence
    # one (set it as the current target) so typing g straight away executes it;
    # the user can still override by typing another #id first.
    best = max(matches, key=lambda t: t.get('conf', 0.0))
    node.send(f"#{best['tid']}")
    print(f"  understood '{phrase}', but {len(matches)} objects match -- "
          f"defaulting to highest-confidence #{best['tid']}:")
    for t in matches:
        mark = '  <- default' if t['tid'] == best['tid'] else ''
        print(f"    #{t['tid']}  {t['label']}"
              f"  x={t['x']:+.2f} y={t['y']:+.2f} z={t['z']:+.2f}"
              f"  conf={t.get('conf', 0.0):.2f}{mark}")
    print('  (type g to take the default, or another #id first)')


def _loop(node: TargetCli):
    print('\n=== target_cli ===  (Enter=refresh, g/go=execute approach, q=quit)')
    print('type #<id> (STABLE track, preferred) / obj_N / an index -> set target,')
    print('OR a natural request e.g. "please bring a teddy bear" / "get a can";')
    print('then g (or go) -> move the winning arm to approach it')
    while rclpy.ok():
        tracks = node.track_list()
        objn, n = node.objn_map()
        cur = f'  [current: {node.last_sent}]' if node.last_sent else ''
        print(f'\nDetected objects ({n} total){cur}')
        npub = node.tracks_publisher_count()
        if npub > 1:
            print(f'  !! WARNING: {npub} object_localizer instances publishing '
                  'tracks -- #id may target the WRONG object. Kill the stale '
                  'topo_fusion launch and keep exactly one.')
        # Stable tracks are the source-agnostic handle (work for Isaac GT AND
        # YOLOE); a #<id> follows one physical object through label flicker.
        if tracks:
            print('  stable tracks (target by #id):')
            for t in tracks:
                print(f"    #{t['tid']}  {t['label']:<16}"
                      f"  x={t['x']:+.2f} y={t['y']:+.2f} z={t['z']:+.2f}")
        elif not objn:
            print('  (no tracks/obj_N yet -- is perception up?)')
        for name in sorted(objn):
            i, p = objn[name]
            print(f'  {name}  (index {i})  x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}')
        try:
            raw = input('target> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        low = raw.lower()
        if low in ('q', 'quit', 'exit'):
            break
        if low in ('g', 'go', 'x', 'execute'):
            node.execute()
            print('  -> execute sent (watch reach_fusion for the plan/execute result)')
        elif not raw:
            continue
        elif raw.startswith('#') or _OBJN_RE.match(low) or low.lstrip('-').isdigit():
            # explicit handle: stable track #id, obj_N, or list index -> as-is
            node.send(raw)
            print(f'  -> target set to "{raw}"  (type g to execute)')
        else:
            # free-form request -> resolve to a stable track handle
            _fetch(node, raw)


def main():
    rclpy.init()
    node = TargetCli()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        _loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
