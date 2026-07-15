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
from collections import Counter

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
# object_localizer's vote-hysteresis (label_hysteresis_margin/vote_decay) is meant
# to reject 1-frame label flicker, but a SUSTAINED run of wrong YOLOE classifications
# can still out-vote it -- observed LIVE: a track's committed label flipped from
# 'brown teddy bear' to 'red box' (its POSITION stayed anchored on the physical
# teddy bear the whole time -- identity/position tracking is fine, only the class
# label mis-committed). How many publish cycles a flip like that persists for is
# NOT known/bounded, so a natural-language match cannot just trust one snapshot, or
# even trust that a SECOND read shortly after will have reverted. Instead _fetch
# samples the live tracks for VOTE_WINDOW seconds and matches by MAJORITY vote
# across that window: a genuinely, stably labelled object dominates the window
# regardless of how long any one flip lasts, whereas a flip that never becomes the
# majority can never be selected. VOTE_POLL is finer than object_localizer's
# default 0.5 s publish period so the window reliably samples several cycles.
VOTE_WINDOW = 3.0
VOTE_POLL = 0.4


def _label_matches(label, aliases, color):
    """True if `label` still satisfies an NL match's (aliases, color) criteria.

    Shared by _fetch's live sampling and the execute-time drift re-check in
    _loop, so 'still the same match' means the same thing in both places.
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
        self.last_match = None
        self.pending_confirm = False

    def execute(self):
        self.exec_pub.publish(Empty())


def _fetch(node: TargetCli, sentence):
    """Natural-language fetch -> a STABLE track handle (#tid).

    Deterministically parses the object phrase ('please bring a teddy bear' ->
    'teddy bear'), then matches it against the current confirmed tracks by class
    (color optional), sampling over VOTE_WINDOW and picking the MAJORITY-vote tid
    (see module docstring on VOTE_WINDOW for why a single snapshot is not trusted).
    One candidate -> targets its #tid; several -> the most-voted one, listing the
    candidate #ids so the user can override. Identity is the persistent track
    (position-anchored), so this works under Isaac GT and YOLOE alike.
    """
    phrase = parse_object(sentence)
    if not phrase:
        print('  ! no object name found in that request')
        return
    color, cls = split_color(phrase)
    aliases = _alias_group(cls or phrase)

    def _match(tracks):
        return [t for t in tracks if _label_matches(t.get('label', ''), aliases, color)]

    # Perception can miss the object on the FIRST frames (label flicker / a track
    # not confirmed yet), so don't reject on an empty snapshot -- keep sampling for
    # up to SEARCH_SECS before giving up. Once ANY sample matches, collect votes
    # for VOTE_WINDOW seconds (across >= 2 publish cycles) and pick the tid with
    # the most matching samples, not just whichever tid the FIRST or LATEST sample
    # happened to carry -- that is what makes a transient mislabel (a minority of
    # samples) lose to the genuinely, stably labelled object (the majority).
    deadline = time.time() + SEARCH_SECS
    votes = Counter()
    latest = {}   # tid -> most recent matching track dict (for display + position)
    window_end = None
    printed_looking = False
    while rclpy.ok():
        matches = _match(node.track_list())
        for t in matches:
            votes[t['tid']] += 1
            latest[t['tid']] = t
        if matches and window_end is None:
            window_end = time.time() + VOTE_WINDOW
        if window_end is not None and time.time() >= window_end:
            break
        if window_end is None:
            if time.time() >= deadline:
                break
            if not printed_looking:
                print(f"  -> looking for '{phrase}' "
                      f"(re-checking tracks for {SEARCH_SECS:g}s)...")
                printed_looking = True
        time.sleep(VOTE_POLL)
    if not votes:
        seen = sorted({str(t.get('label', '')) for t in node.track_list()})
        print(f"  ! no '{phrase}' among current tracks after {SEARCH_SECS:g}s "
              f"(seen: {seen or '(none)'})")
        return
    total = sum(votes.values())
    ranked = votes.most_common()
    best_tid, best_n = ranked[0]
    # A thin plurality (the winner didn't dominate the window) means the label was
    # genuinely unstable during the sample -- still act on the best evidence
    # available (majority vote beats a single snapshot either way), but say so
    # loudly rather than silently presenting it as a clean match (Rule: fail loud).
    shaky = best_n < total / 2 or (len(ranked) > 1 and best_n - ranked[1][1] <= 1)
    stability = f"{best_n}/{total} samples" + (' -- UNSTABLE, verify below' if shaky else '')
    if len(ranked) == 1:
        t = latest[best_tid]
        node.send(f"#{best_tid}")
        node.last_match = {'phrase': phrase, 'aliases': aliases, 'color': color,
                           'tid': best_tid, 'label_at_select': t['label']}
        print(f"  -> understood '{phrase}' = track #{best_tid} ({t['label']}), "
              f"{stability}; type g to execute the approach")
        return
    # Several candidate tids got votes (either genuinely distinct objects of the
    # same class, or one is a flicker artifact) -- default to the majority-vote
    # one so typing g straight away executes it; list all so the user can verify
    # and override with another #id if the vote was close.
    node.send(f"#{best_tid}")
    node.last_match = {'phrase': phrase, 'aliases': aliases, 'color': color,
                       'tid': best_tid, 'label_at_select': latest[best_tid]['label']}
    print(f"  understood '{phrase}', {len(ranked)} tracks matched over the "
          f"sample window -- defaulting to #{best_tid} ({stability}):")
    for tid, n in ranked:
        t = latest[tid]
        mark = '  <- default' if tid == best_tid else ''
        print(f"    #{tid}  {t['label']}"
              f"  x={t['x']:+.2f} y={t['y']:+.2f} z={t['z']:+.2f}"
              f"  votes={n}/{total}{mark}")
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
