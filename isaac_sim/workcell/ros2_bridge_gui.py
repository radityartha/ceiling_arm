"""ROS 2 bridge - step 2 (bidirectional).

Publishes /joint_states (+ /clock) AND subscribes to /joint_command
(sensor_msgs/JointState) -> drives the workcell articulation. Send a command with
joint names + positions for any subset of the 44 joints; only those are driven.

VERIFIED CONFIG (same as step 1): use CycloneDDS on both sides, matching domain,
and world.step(render=True) so the OmniGraph ticks.

    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python ros2_bridge.py

Drive a joint from another shell (system Humble, same two env vars):
    ros2 topic pub -r 20 /joint_command sensor_msgs/msg/JointState \
        "{name: ['t1_a1_joint_2'], position: [0.6]}"
    ros2 topic echo /joint_states            # watch t1_a1_joint_2 move to ~0.6
"""
import os
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
ROBOT = "/World/workcell"

# Two RGBD cameras at DIAGONALLY OPPOSITE corners of the gantry_1 work area (rail
# x: 0..2.0 m; reachable EE extent x ~[-0.66, 2.60]). Both sit at ceiling height
# (z=2.05, the max allowed). EXPERIMENTAL diagonal layout for visual comparison:
# cam1 (rgbd) is mirrored to the +Y side at (+X, +Y) while cam2 (rgbd2) stays at
# (-X, -Y), so their view axes cross diagonally in X and Y for extra azimuthal
# coverage. Trade-off vs the old both-on--Y layout: cam1 now looks across the
# hanging arms (y=+0.36) so expect more arm self-occlusion of the work area. cam2
# is placed symmetric to cam1: its eye (y=-1.2) sits 0.84 m off gantry_2's centre
# (y=-0.36) on -Y, matching cam1's 0.84 m off gantry_1's centre (y=+0.36) on +Y --
# a mirror across y=0. Detections fuse in `world`
# (each is independent ground-truth RGBD, calibrated to `world` by its static TF
# in launch_workcell.sh -> KEEP THOSE TFs IN SYNC if you move a camera).
# To revert to the both-on--Y baseline: cam1 eye -> (2.8, -0.6, 2.05) and its TF
# quat -> (-0.722084, -0.422309, 0.27663, 0.472996).
CAM_RES = (1280, 720)
# rgbd (cam1) eye+target both shifted +1.35 in X to follow polish.py's work
# table move (cx 1.55 -> 2.9, for the wider pick-distance calibration range).
# Shifting eye AND target by the SAME delta leaves the target-eye vector (and
# therefore the look-at orientation / TF quaternion) UNCHANGED -- pure
# translation. Keep launch_workcell.sh's rgbd_camera_optical static TF --x in
# sync (3.0 -> 4.35); its quaternion does not need to change.
CAMERAS = [
    {"prim": "/World/rgbd_camera",  "ns": "rgbd",  "frame": "rgbd_camera_optical",
     "eye": (4.35, 1.2, 2.05),  "target": (2.55, 0.30, 1.25), "focal": 17.25},
    {"prim": "/World/rgbd2_camera", "ns": "rgbd2", "frame": "rgbd2_camera_optical",
     "eye": (-0.6, -1.2, 2.05), "target": (1.2, -0.30, 1.25), "focal": 16.42},
]

import polish  # noqa: E402  (room + lights + work table; same dir)

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT)
wc = world.scene.add(Robot(prim_path=ROBOT, name="workcell"))


def _raise_table_drive_force(maxforce=1.0e7):
    """Set USD drive max-force on the table joints BEFORE physics init.

    The URDF importer bakes the (tiny) URDF effort as the joint drive max-force
    (rotation was 10 N·m) — far too low to turn the rotation bar + 2 arms, so the
    rotation joint stalls mid-move. `Robot` has no `set_max_efforts` in this Isaac
    build, so we set the UsdPhysics.DriveAPI maxForce on the joint prims directly.
    """
    st = get_current_stage()
    nset = 0
    for p in st.Traverse():
        nm = p.GetName()
        if not (nm.endswith("rotation_joint") or nm.endswith("linear_joint")):
            continue
        for sch in p.GetAppliedSchemas():            # e.g. 'PhysicsDriveAPI:angular'
            if sch.startswith("PhysicsDriveAPI:"):
                tok = sch.split(":", 1)[1]
                drv = UsdPhysics.DriveAPI.Get(p, tok)
                if drv:
                    (drv.GetMaxForceAttr() or drv.CreateMaxForceAttr()).Set(maxforce)
                    nset += 1
    print(f">>> raised drive maxForce on {nset} table-joint drives", flush=True)


def _add_rgbd_cameras():
    """Create each RGBD camera prim looking at the corridor it covers.

    USD cameras look down local -Z with +Y up; we build the camera->world matrix
    from an eye/target look-at so the orientation stays in sync with the static
    TF the perception node uses. The prims have no physics, so they survive
    world.reset(). Render products + ROS publishers are added later in a loop over
    CAMERAS (off the same OmniGraph tick as the joint-state publishers).
    """
    from pxr import Gf, UsdGeom
    up = np.array([0.0, 0.0, 1.0])
    stage = get_current_stage()
    for c in CAMERAS:
        eye = np.array(c["eye"], float)
        target = np.array(c["target"], float)
        f = target - eye; f /= np.linalg.norm(f)        # forward = camera -Z
        r = np.cross(f, up); r /= np.linalg.norm(r)     # right   = camera +X
        u = np.cross(r, f)                              # up      = camera +Y
        z = -f                                          # camera +Z
        M = Gf.Matrix4d(1.0)
        M.SetRow(0, Gf.Vec4d(float(r[0]), float(r[1]), float(r[2]), 0.0))
        M.SetRow(1, Gf.Vec4d(float(u[0]), float(u[1]), float(u[2]), 0.0))
        M.SetRow(2, Gf.Vec4d(float(z[0]), float(z[1]), float(z[2]), 0.0))
        M.SetRow(3, Gf.Vec4d(float(eye[0]), float(eye[1]), float(eye[2]), 1.0))

        cam = UsdGeom.Camera.Define(stage, c["prim"])
        cam.AddTransformOp().Set(M)
        # focal -> ~63 deg HFOV covering this camera's half of the corridor;
        # vertical aperture matches CAM_RES aspect so camera_info has square px.
        cam.CreateFocalLengthAttr(c["focal"])
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(20.955 * CAM_RES[1] / CAM_RES[0])
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 12.0))
        print(f">>> added RGBD camera prim {c['prim']} at {c['eye']} "
              f"-> look {c['target']}", flush=True)


_raise_table_drive_force()
_objs = polish.build_room()
polish.add_lights()
for _o in _objs:
    world.scene.add(_o)
world.reset()
polish.recolor_tables()

# Hide gantry_2 + arm_3/arm_4 (all t2_* prims) for the gantry_1-focused GNG view.
# VISIBILITY ONLY — the articulation keeps every DOF, so the bridge still
# publishes/accepts the full /isaac_joint_states (RViz/move_group use a
# matching gantry_1-only model). Set GNG_HIDE_T2=0 to keep them visible.
if os.environ.get("GNG_HIDE_T2", "1") != "0":
    from pxr import UsdGeom  # noqa: E402
    _stage = get_current_stage()
    _hidden = 0
    for _p in _stage.Traverse():
        if "t2_" in _p.GetName():
            _img = UsdGeom.Imageable(_p)
            if _img:
                _img.MakeInvisible()
                _hidden += 1
    print(f">>> GNG view: hid {_hidden} t2_ prims (gantry_2 + arm_3/arm_4)", flush=True)

_add_rgbd_cameras()

ART = next((str(p.GetPath()) for p in get_current_stage().Traverse()
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)), ROBOT)
print(f">>> articulation root prim = {ART}", flush=True)

# Stiff drives so commanded positions are reached and held.
# Arm revolutes: stiff + damped (1e5/1e4) tracks well. The TABLE joints are
# heavy (platform + 2 arms) with low effort limits, so the same big damping
# (kd=1e4 -> ~5000 N at 0.5 m/s, above the ~1000 N rail effort) makes the
# prismatic stick-slip / stutter. Give the table joints stiff position gain but
# MUCH lower damping so the position drive tracks the JTC smoothly.
n = wc.num_dof
names = list(wc.dof_names)
kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
# Table joints get their own gains: stiff position gain + damping near
# critically-damped for their inertia (with the high max-effort below, this
# tracks smoothly AND settles without the inertial coast/overshoot you'd get
# from too-low damping). Tune kd up if it still coasts, down if it stick-slips.
for i, nm in enumerate(names):
    if nm.endswith("linear_joint"):
        kps[i] = 1.0e5; kds[i] = 3.0e3
    elif nm.endswith("rotation_joint"):
        kps[i] = 1.0e5; kds[i] = 1.5e3
wc.get_articulation_controller().set_gains(kps=kps, kds=kds)

# Build the OmniGraph nodes/connections/values for every camera in CAMERAS: one
# render product per camera feeding RGB + depth + camera_info + instance-
# segmentation ROS publishers, all ticking off the same OnPlaybackTick/Context as
# the joint-state publishers. Topic suffix per helper key:
_cam_topics = {"rgb": "rgb", "depth": "depth", "info": "camera_info",
               "seg": "instance_segmentation"}
_cam_nodes, _cam_connects, _cam_setvals = [], [], []
for _c in CAMERAS:
    _ns = _c["ns"]
    _rp = f"RP_{_ns}"
    # key -> ROS2CameraHelper `type`
    _helpers = {"rgb": "rgb", "depth": "depth", "info": "camera_info",
                "seg": "instance_segmentation"}
    _cam_nodes.append((_rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"))
    _cam_connects.append(("OnPlaybackTick.outputs:tick", f"{_rp}.inputs:execIn"))
    _cam_setvals += [
        (f"{_rp}.inputs:cameraPrim", [usdrt.Sdf.Path(_c["prim"])]),
        (f"{_rp}.inputs:width", CAM_RES[0]),
        (f"{_rp}.inputs:height", CAM_RES[1]),
    ]
    for _key, _type in _helpers.items():
        _h = f"Cam_{_ns}_{_key}"
        _cam_nodes.append((_h, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        _cam_connects += [
            (f"{_rp}.outputs:execOut", f"{_h}.inputs:execIn"),
            (f"{_rp}.outputs:renderProductPath", f"{_h}.inputs:renderProductPath"),
            ("Context.outputs:context", f"{_h}.inputs:context"),
        ]
        _cam_setvals += [
            (f"{_h}.inputs:topicName", f"{_ns}/{_cam_topics[_key]}"),
            (f"{_h}.inputs:type", _type),
            (f"{_h}.inputs:frameId", _c["frame"]),
        ]
        if _key == "seg":
            # also publish the instance-id -> semantic-class JSON mapping so the
            # localizer can name each detected object (ground-truth seg now; swap
            # the mask source for an open-vocab detector later).
            _cam_setvals += [
                (f"{_h}.inputs:enableSemanticLabels", True),
                (f"{_h}.inputs:semanticLabelsTopicName",
                 f"{_ns}/instance_segmentation_labels"),
            ]

og.Controller.edit(
    {"graph_path": "/ROS2_Bridge", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            # Separate subscriber + controller for the TABLE so its commands are
            # never starved by the arm/gripper flood on /isaac_joint_commands
            # (single-latest-message subscriber drops the table's setpoints while
            # the arm is active -> the rail/rotation freezes mid-move).
            ("SubscribeTable", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationControllerTable", "isaacsim.core.nodes.IsaacArticulationController"),
        ] + _cam_nodes,
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            # table branch
            ("OnPlaybackTick.outputs:tick", "SubscribeTable.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationControllerTable.inputs:execIn"),
            ("Context.outputs:context", "SubscribeTable.inputs:context"),
            ("SubscribeTable.outputs:jointNames", "ArticulationControllerTable.inputs:jointNames"),
            ("SubscribeTable.outputs:positionCommand", "ArticulationControllerTable.inputs:positionCommand"),
            ("SubscribeTable.outputs:velocityCommand", "ArticulationControllerTable.inputs:velocityCommand"),
            ("SubscribeTable.outputs:effortCommand", "ArticulationControllerTable.inputs:effortCommand"),
        ] + _cam_connects,
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ART)]),
            ("PublishClock.inputs:topicName", "clock"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ArticulationController.inputs:robotPath", ART),
            ("SubscribeTable.inputs:topicName", "isaac_table_commands"),
            ("ArticulationControllerTable.inputs:robotPath", ART),
        ] + _cam_setvals,
    },
)

world.reset()
omni.timeline.get_timeline_interface().play()

# Re-apply drive gains after the final reset/play so they aren't wiped.
wc.get_articulation_controller().set_gains(kps=kps, kds=kds)

# Default start pose: tuck every arm with joint_2 = joint_3 ~= 150 deg so each
# arm starts folded up (out of the camera view / off the work table) instead of
# hanging straight down at the all-zero pose. Set BOTH the articulation state
# (teleport) and the position-drive target so the stiff drive holds it; the
# topic_based ros2_control reads this back as the JTC hold setpoint.
# NOTE: joint_2/joint_3 hard limit is +/-2.61 rad (149.5 deg), so 150 deg is out
# of bounds (MoveIt would reject the start state) -> clamped to 2.60 rad.
_ARM_TUCK = 2.60
_q0 = wc.get_joint_positions().copy()
for _i, _nm in enumerate(names):
    if _nm.endswith("joint_2") or _nm.endswith("joint_3"):
        _q0[_i] = _ARM_TUCK
wc.set_joint_positions(_q0)
wc.get_articulation_controller().apply_action(ArticulationAction(joint_positions=_q0))
print(f">>> default arm pose: joint_2=joint_3=150deg ({_ARM_TUCK:.4f} rad)", flush=True)

# Per-arm kortex ros2_control (topic_based) drives all 16 gripper joints via its mimic
# params, so no extra coupling is needed here.
print(">>> ROS2 bridge: publishing /isaac_joint_states, subscribing /isaac_joint_commands. Ctrl-C to stop.", flush=True)
i = 0
while simulation_app.is_running():
    world.step(render=True)   # render pumps the OmniGraph so nodes tick
    i += 1
    if i % 120 == 0:
        print(f">>> running (step {i})", flush=True)

simulation_app.close()
