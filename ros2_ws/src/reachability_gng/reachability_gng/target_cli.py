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
STATUS_TIMEOUT = 60.0   # how long to wait for a mission result after 'g' before giving up
STATUS_POLL = 0.3


def _label_matches(label, aliases, color):
    """True if `label` still satisfies an NL match's (aliases, color) criteria.

    Shared by _fetch's current-track matching and the execute-time drift
    re-check in _loop, so 'still the same match' means the same thing in both.
    """
    lab = str(label).lower()
    return bool(lab) and any(a in lab for a in aliases) and (not color or color in lab)


class TargetCli(Node):
    def __init__(self):
        super().__init__('target_cli')
        self.declare_parameter('set_target_topic', '/reach_fusion/set_target')
        self._lock = threading.Lock()
        self.poses = np.empty((0, 3))
        self.tracks = []            # [{tid,label,x,y,z}] stable-identity handles
        self.last_sent = None
        # Set by _fetch() when a target comes from an NL match (aliases/color to
        # re-check), cleared by send() otherwise (explicit #id/obj_N/index has no
        # phrase to drift-check against). See execute-time guard in _loop.
        self.last_match = None
        self.pending_confirm = False
        self.status = None          # last unconsumed /reach_fusion/status message
        self.objn = ObjnLocalizer(self)
        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        tq = QoSProfile(depth=1)
        tq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/detected_objects/tracks',
                                 self._on_tracks, tq)
        self.create_subscription(String, '/reach_fusion/status',
                                 self._on_status, 1)
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
        self.last_match = None
        self.pending_confirm = False

    def execute(self):
        with self._lock:
            self.status = None
        self.exec_pub.publish(Empty())

    def _on_status(self, msg):
        with self._lock:
            self.status = msg.data

    def pop_status(self):
        with self._lock:
            s = self.status
            self.status = None
            return s


def _fetch(node: TargetCli, sentence):
    """Natural-language fetch -> a STABLE track handle (#tid).

    Deterministically parses the object phrase ('please bring a teddy bear' ->
    'teddy bear'), then matches it against the tracks already known right now
    (the same snapshot just printed to the user) by class (color optional).
    Does NOT re-poll perception over a multi-second window: a track that was
    just shown on screen and then re-searched live could churn (drop / get a
    new tid) before the search finished, making a just-visible object look
    like it "disappeared" -- confusing since the user picked exactly what they
    saw. Using the known snapshot directly avoids that. One candidate ->
    targets its #tid; several -> the first, listing all #ids so the user can
    override. Identity is the persistent track (position-anchored), so this
    works under Isaac GT and YOLOE alike.
    """
    phrase = parse_object(sentence)
    if not phrase:
        print('  ! no object name found in that request')
        return
    color, cls = split_color(phrase)
    aliases = _alias_group(cls or phrase)
    tracks = node.track_list()
    matches = [t for t in tracks if _label_matches(t.get('label', ''), aliases, color)]
    if not matches:
        seen = sorted({str(t.get('label', '')) for t in tracks})
        print(f"  ! no '{phrase}' among currently known tracks "
              f"(seen: {seen or '(none)'})")
        return
    if len(matches) == 1:
        t = matches[0]
        node.send(f"#{t['tid']}")
        node.last_match = {'phrase': phrase, 'aliases': aliases, 'color': color,
                           'tid': t['tid'], 'label_at_select': t['label']}
        print(f"  -> understood '{phrase}' = track #{t['tid']} ({t['label']}); "
              "type g to execute the approach")
        return
    # Several tracks currently match (either genuinely distinct objects of the
    # same class, or a duplicate) -- default to the first so typing g straight
    # away executes it; list all so the user can verify and override with
    # another #id.
    best = matches[0]
    node.send(f"#{best['tid']}")
    node.last_match = {'phrase': phrase, 'aliases': aliases, 'color': color,
                       'tid': best['tid'], 'label_at_select': best['label']}
    print(f"  understood '{phrase}', {len(matches)} known tracks match -- "
          f"defaulting to #{best['tid']}:")
    for t in matches:
        mark = '  <- default' if t is best else ''
        print(f"    #{t['tid']}  {t['label']}"
              f"  x={t['x']:+.2f} y={t['y']:+.2f} z={t['z']:+.2f}{mark}")
    print('  (type g to take the default, or another #id first)')


def _loop(node: TargetCli):
    print('\n=== target_cli ===  (Enter=refresh, g/go=execute approach, q=quit)')
    print('type the list number / #<id> (STABLE track, preferred) / obj_N -> set target,')
    print('OR a natural request e.g. "please bring a teddy bear" / "get a can";')
    print('then g (or go) -> move the winning arm to approach it')
    # Once a target is picked, the full list is noise on every prompt -- only
    # dump it again when there's something new to look at: nothing selected
    # yet, the user explicitly asks (blank Enter), or a mission just ended.
    show_list = True
    display_map = {}   # '1'..'N' (as last printed) -> tid, for picking by list number
    while rclpy.ok():
        npub = node.tracks_publisher_count()
        if npub > 1:
            print(f'  !! WARNING: {npub} object_localizer instances publishing '
                  'tracks -- #id may target the WRONG object. Kill the stale '
                  'topo_fusion launch and keep exactly one.')
        if show_list:
            tracks = node.track_list()
            objn, n = node.objn_map()
            cur = f'  [current: {node.last_sent}]' if node.last_sent else ''
            print(f'\nDetected objects ({n} total){cur}')
            # Stable tracks are the source-agnostic handle (work for Isaac GT AND
            # YOLOE); a #<id> follows one physical object through label flicker.
            # tid values can have big gaps/jump around as tracks churn, which
            # reads as "out of order" -- number the printed list 1..N instead so
            # it's always a clean, readable sequence; #<tid> still works too.
            display_map = {str(i): t['tid'] for i, t in enumerate(tracks, start=1)}
            if tracks:
                print('  stable tracks (target by number or #id):')
                for i, t in enumerate(tracks, start=1):
                    print(f"    {i}) #{t['tid']}  {t['label']:<16}"
                          f"  x={t['x']:+.2f} y={t['y']:+.2f} z={t['z']:+.2f}")
            elif not objn:
                print('  (no tracks/obj_N yet -- is perception up?)')
            for name in sorted(objn):
                i, p = objn[name]
                print(f'  {name}  (index {i})  x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}')
        else:
            print(f'\n[current: {node.last_sent}]  (Enter to see the full list)')
        show_list = False
        try:
            raw = input('target> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        low = raw.lower()
        if low in ('q', 'quit', 'exit'):
            break
        if low in ('g', 'go', 'x', 'execute'):
            lm = node.last_match
            if lm and not node.pending_confirm:
                # Re-check the LIVE label right before firing, not the label at
                # selection time -- object_localizer's vote hysteresis can be
                # defeated by a sustained (multi-second) run of misclassified
                # frames (observed live: a majority-vote match can still be
                # stale by execute time). Catching drift here, not just at
                # selection, is what a fixed track id alone cannot guarantee.
                live = next((t for t in node.track_list()
                            if t.get('tid') == lm['tid']), None)
                if live is None:
                    print(f"  ! track #{lm['tid']} ('{lm['label_at_select']}') "
                          "is no longer visible -- pick a target again")
                    continue
                if not _label_matches(live.get('label', ''), lm['aliases'], lm['color']):
                    node.pending_confirm = True
                    print(f"  !! WARNING: '{lm['phrase']}' was matched to track "
                          f"#{lm['tid']} ('{lm['label_at_select']}') at selection "
                          f"time, but it is now labelled '{live['label']}' -- the "
                          "class label may have drifted (YOLOE instability), and "
                          "executing may approach the WRONG object.")
                    print('  type g again to execute anyway, or pick a different target.')
                    continue
            node.execute()
            node.pending_confirm = False
            print('  -> execute sent, waiting for result...')
            # Don't reprint the object list while the approach is in flight --
            # only show it again once the mission actually succeeds or fails,
            # so the list isn't re-dumped on every loop tick during a long move.
            waited, status = 0.0, None
            try:
                while rclpy.ok() and waited < STATUS_TIMEOUT:
                    status = node.pop_status()
                    if status is not None:
                        break
                    time.sleep(STATUS_POLL)
                    waited += STATUS_POLL
            except KeyboardInterrupt:
                break
            if status is not None:
                print(f'  ==> mission {status}')
            else:
                print(f'  !! no result after {STATUS_TIMEOUT:g}s -- '
                      'check reach_fusion log (may still be running)')
            show_list = True   # mission ended (success/failed/timeout) -- show it again
        elif not raw:
            show_list = True   # explicit Enter=refresh
        elif low in display_map:
            # plain number -> the list position from the last printed listing,
            # translated to its real #tid (list order can differ from tid order)
            tid = display_map[low]
            node.send(f"#{tid}")
            print(f'  -> target set to #{tid} (list {low})  (type g to execute)')
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
