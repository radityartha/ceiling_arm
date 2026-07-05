"""Interactive object picker for the gantry_reach_executor.

A small control terminal: it lists the live objects from /detected_objects
(with labels from /detected_objects/markers) and lets you type an index to fire
a pick -- replacing the repeated `ros2 topic pub .../pick` one-liner.

Selecting an object publishes its LABEL on /grasp_target, which makes
collision_cloud carve that object out of the octomap and object_collision box it
(so the gripper can reach + attach it) while every other object stays an octomap
obstacle; object_localizer then puts it on /target_object for the executor. Then
the index is sent to fire the pick. Type `c` to clear the target (all objects go
back into the octomap).

Each object also shows a CHEAP distance estimate: the straight-line distance
from the object to each arm's current tool frame (via TF), and which arm is
nearer. This is only a geometric proxy -- the executor's true energy J (gantry
travel + hold + manipulability over the GNG maps) is computed at pick time and
printed in its own terminal / CSV, not here.

It also drives the segmentation source (seg_router): type `y`/`i` to switch
between YOLOE open-vocab and Isaac ground truth, and `p` to set the YOLOE class
prompts (which become the object labels you target). So this one terminal both
selects the detector AND fires picks -- no separate `ros2 topic pub` needed.

Natural-language fetch: type a plain request like "please get me a box". The
object phrase is extracted (deterministic keyword parse), YOLOE is pointed at it,
and once it is detected the object is targeted and the pick is fired
automatically -- a minimal human-robot interface on top of the open-vocab
detector.

    ros2 launch reachability_gng gantry_pick.launch.py     # backend (terminal 1)
    ros2 run reachability_gng pick_cli                     # this menu (terminal 2)

Watch the gantry_pick.launch.py terminal for the chosen arm / plan result.
"""
from __future__ import annotations

import math
import re
import threading
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import MarkerArray


class PickCli(Node):
    def __init__(self):
        super().__init__('pick_cli')
        self.declare_parameter('pick_topic', '/gantry_reach_executor/pick')
        # kept in sync with gantry_reach_executor's arm_names / arm_ee_frames
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter('tool_frames',
                               ['t1_a1_tool_frame', 't1_a2_tool_frame'])
        self._topic = self.get_parameter('pick_topic').value
        self.arm_names = list(self.get_parameter('arm_names').value)
        self.tool_frames = list(self.get_parameter('tool_frames').value)

        self._poses = []                 # latest /detected_objects poses
        self._world = 'world'            # /detected_objects header frame
        self._labels = {}                # marker id -> label text
        self._lock = threading.Lock()
        # local echo of what we last commanded seg_router to (it publishes no
        # state topic); starts at the perception.launch.py default so the header
        # is honest before the first y/i/p command.
        self._seg_source = 'yoloe'
        self._seg_prompts = ['box', 'can', 'bottle', 'banana', 'teddy bear']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        self.create_subscription(MarkerArray, '/detected_objects/markers',
                                 self._on_markers, 1)
        self.pick_pub = self.create_publisher(String, self._topic, 1)
        # Selecting an object also announces it as the grasp target so
        # collision_cloud carves it out of the octomap and object_collision boxes
        # it (reachable + attachable); the rest stay octomap obstacles.
        self.target_pub = self.create_publisher(String, '/grasp_target', 1)
        # seg_router controls: pick the detector + its open-vocab classes live.
        self.seg_source_pub = self.create_publisher(String, '/seg_source', 1)
        self.seg_prompts_pub = self.create_publisher(String, '/seg_prompts', 1)

    def _on_objects(self, msg):
        with self._lock:
            self._poses = list(msg.poses)
            if msg.header.frame_id:
                self._world = msg.header.frame_id

    def _on_markers(self, msg):
        with self._lock:
            for m in msg.markers:
                if m.ns == 'labels':
                    self._labels[m.id] = m.text

    def snapshot(self):
        with self._lock:
            return list(self._poses), dict(self._labels), self._world

    def _tool_xyz(self, frame, world):
        """Current tool-frame position in `world`, or None if TF not ready."""
        try:
            t = self.tf_buffer.lookup_transform(world, frame, Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, tr.z)

    def arm_distances(self, pose, world):
        """[(arm_name, distance_m or None)] from object to each arm's tool."""
        p = pose.position
        out = []
        for name, frame in zip(self.arm_names, self.tool_frames):
            xyz = self._tool_xyz(frame, world)
            if xyz is None:
                out.append((name, None))
            else:
                d = math.dist((p.x, p.y, p.z), xyz)
                out.append((name, d))
        return out

    def send_pick(self, idx):
        self.pick_pub.publish(String(data=str(idx)))

    def send_target(self, label):
        """Announce the grasp target (or '' to clear -> all back in octomap).

        Strips a display instance suffix ('yellow bottle 2' -> 'yellow bottle')
        so the target matches the unnumbered seg/track labels downstream.
        """
        self.target_pub.publish(
            String(data=re.sub(r'\s+\d+$', '', label).strip()))

    def set_source(self, source):
        """Switch seg_router between 'isaac' ground truth and 'yoloe'."""
        self.seg_source_pub.publish(String(data=source))
        self._seg_source = source

    def set_prompts(self, prompts):
        """Set YOLOE open-vocab classes (list) -> these become object labels."""
        self.seg_prompts_pub.publish(String(data=','.join(prompts)))
        self._seg_prompts = list(prompts)


# Command/filler words stripped from a natural-language fetch request; whatever
# remains is taken as the object to look for (open-vocab, so any noun works).
_STOP = {
    'please', 'you', 'get', 'me', 'a', 'an', 'the', 'bring', 'grab', 'pick',
    'up', 'hand', 'give', 'i', 'want', 'need', 'like', 'to', 'fetch', 'take',
    'for', 'some', 'that', 'this', 'object', 'thing', 'my', 'of', 'and', 'on',
    'from', 'over', 'there', 'here', 'it', 'pass', 'go', 'find', 'now', 'us',
    'them', 'one',
}
# Modal verbs only strip when leading ('Can you...'); kept elsewhere because
# 'can' is also an object noun ('a soup can').
_LEAD_MODAL = {'can', 'could', 'would', 'will'}


def parse_object(sentence):
    """Extract the target object phrase from a free-form request.

    Deterministic: lowercase, keep alphanumeric tokens, drop command/filler
    words. 'Please get me a box' -> 'box'; 'bring the tomato soup can' ->
    'tomato soup can'; 'Can you hand me a can' -> 'can'. Returns '' when
    nothing is left.
    """
    words = re.findall(r'[a-z0-9]+', sentence.lower())
    if words and words[0] in _LEAD_MODAL:
        words = words[1:]              # drop leading modal, keep object 'can'
    return ' '.join(w for w in words if w not in _STOP).strip()


# Color words seg_router prefixes onto labels (see seg_router.name_color).
_COLORS = {'red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple',
           'pink', 'brown', 'white', 'gray', 'grey', 'black'}


def split_color(phrase):
    """'yellow box' -> ('yellow', 'box'); 'box' -> ('', 'box').

    The color is used to disambiguate objects; the class is what YOLOE is
    pointed at (it detects the shape reliably, seg_router adds the color).
    """
    words = phrase.split()
    color = next((w for w in words if w in _COLORS), '')
    cls = ' '.join(w for w in words if w not in _COLORS).strip()
    return color, cls


# Class aliases: words that name the SAME object for YOLOE. Saying any member
# points YOLOE at the whole group (so a reliably-detected member still fires --
# e.g. YOLOE always reads the plush as 'teddy bear', never 'doll'), and a
# detection labelled as ANY member satisfies the request.
_ALIAS_GROUPS = [{'teddy bear', 'doll'}]


def _alias_group(cls):
    """The alias set containing `cls` (incl. itself), or {cls} when it has none."""
    for g in _ALIAS_GROUPS:
        if cls in g:
            return set(g)
    return {cls}


def _fetch(node, sentence):
    """Understand a request, point YOLOE at the object, then target + pick it.

    Color-aware: 'bring yellow box' -> YOLOE detects the class 'box' (reliable),
    seg_router tags each with its measured color, and we pick the object whose
    label is 'yellow box'. Falls back to any same-class object if the exact
    color is not seen (with a note).
    """
    obj = parse_object(sentence)
    if not obj:
        print('  ! no object name found in that request')
        return
    color, cls = split_color(obj)
    detect_class = cls or obj         # what YOLOE is told to look for
    aliases = _alias_group(detect_class)   # {detect_class}, or its synonym group
    prompts = sorted(aliases)
    print(f"  -> understood: '{obj}'  (YOLOE looking for {prompts})")
    node.set_source('yoloe')
    node.set_prompts(prompts)
    deadline = time.time() + 25.0     # YOLOE warmup + ~0.75 Hz inference
    idx = fallback = None
    while time.time() < deadline and rclpy.ok():
        poses, labels, _ = node.snapshot()
        for i in range(len(poses)):
            lab = labels.get(i, '').lower()
            if not lab:
                continue
            cls_hit = any(a in lab for a in aliases)   # class (or an alias) match
            if cls_hit and (not color or color in lab):
                idx = i               # class matches (+ color if one was asked)
                break
            if cls_hit and fallback is None:
                fallback = i          # same class, different/unknown color
        if idx is not None:
            break
        time.sleep(0.5)
    if idx is None and fallback is not None:
        idx = fallback
        if color:
            print(f"  ! could not confirm a '{color}' one -- "
                  f"picking the nearest '{cls}'")
    if idx is None:
        print(f"  ! '{obj}' not detected within 25 s -- is it in the camera "
              f"view? try rephrasing, or 'p {', '.join(prompts)}' "
              f"then watch the list")
        return
    target = node.snapshot()[1].get(idx, obj)   # full measured label of the pick
    node.send_target(target)          # object_localizer -> /target_object + carve/box
    # Grasp via /target_object (label-based), NOT the positional index: narrowing
    # the prompt changes the /detected_objects length, so an index computed here
    # can be stale by the time the executor reads its list (-> 'index out of
    # range'). 'target' tells the executor to use /target_object instead. Give
    # object_localizer's timer a moment to publish it first.
    time.sleep(1.5)
    node.send_pick('target')
    print(f"  -> fetching '{target}': target set + pick sent via /target_object "
          f"(see executor terminal for arm/plan result)")


def _fmt_dist(node, pose, world):
    dists = node.arm_distances(pose, world)
    known = [(n, d) for n, d in dists if d is not None]
    if not known:
        return 'dist: TF not ready'
    parts = ' '.join(f'{n}={d:.2f}m' for n, d in known)
    nearest = min(known, key=lambda nd: nd[1])
    return f'{parts}  -> nearest {nearest[0]}'


def _print_menu(node, poses, labels, world):
    if not poses:
        print('  (no objects on /detected_objects yet -- '
              'check perception.launch.py)')
        return
    for i, p in enumerate(poses):
        pos = p.position
        label = labels.get(i, '?')
        print(f'  [{i}] {label:<12} '
              f'x={pos.x:+.2f} y={pos.y:+.2f} z={pos.z:+.2f}   '
              f'{_fmt_dist(node, p, world)}')


def _loop(node: PickCli):
    print('\n=== pick_cli ===  (Enter=refresh, c=clear target, q=quit)')
    print('say : type a request -> finds + picks it, e.g. '
          '"please get me a box"')
    print('seg : y=YOLOE  i=Isaac  p[ classes]=set YOLOE prompts '
          '(e.g. "p box,can,bottle,banana")')
    print('pick: type an object number to pick it directly')
    print('dist = straight-line object->current arm tool (estimate, '
          'not energy J)')
    while rclpy.ok():
        poses, labels, world = node.snapshot()
        prompts = ','.join(node._seg_prompts) if node._seg_prompts else '(default)'
        print(f'\n[seg source: {node._seg_source} | YOLOE prompts: {prompts}]')
        print('Detected objects:')
        _print_menu(node, poses, labels, world)
        try:
            raw = input(f'Select object [0-{max(len(poses) - 1, 0)}]: ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        low = raw.lower()
        if low in ('q', 'quit', 'exit'):
            break
        if low == 'c':
            node.send_target('')         # clear -> every object back in octomap
            print('  -> grasp target cleared (all objects back in octomap)')
            continue
        if low == 'i':
            node.set_source('isaac')
            print('  -> seg source = isaac (ground-truth segmentation)')
            continue
        if low == 'y':
            node.set_source('yoloe')
            hint = ('' if node._seg_prompts
                    else '  (set classes with "p box,can,bottle,banana")')
            print(f'  -> seg source = yoloe (open-vocab){hint}')
            continue
        if low == 'p' or low.startswith('p '):
            rest = raw[1:].strip()
            if not rest:
                try:
                    rest = input('  YOLOE classes (comma-separated): ').strip()
                except (EOFError, KeyboardInterrupt):
                    continue
            classes = [c.strip() for c in rest.split(',') if c.strip()]
            if not classes:
                print('  ! no classes given')
                continue
            node.set_prompts(classes)
            print(f'  -> YOLOE prompts = {classes}')
            continue
        if raw == '':
            continue                     # refresh the menu
        if not raw.isdigit():
            _fetch(node, raw)            # natural-language request -> find + pick
            continue
        idx = int(raw)
        if idx >= len(poses):
            print(f'  ! index {idx} out of range '
                  f'(only {len(poses)} objects)')
            continue
        label = labels.get(idx, '')
        if label and label != '?':
            node.send_target(label)      # carve it out + box it before picking
            print(f'  -> grasp target = {label} (carved out of octomap + boxed)')
        else:
            print('  ! no label for this object yet; picking by index only')
        node.send_pick(idx)
        print(f'  -> pick object {idx} sent to {node._topic} '
              f'(see executor terminal for the result)')


def main():
    rclpy.init()
    node = PickCli()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
