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
from pxr import UsdPhysics  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
ROBOT = "/World/workcell"

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


_raise_table_drive_force()
_objs = polish.build_room()
polish.add_lights()
for _o in _objs:
    world.scene.add(_o)
world.reset()
polish.recolor_tables()

# Hide table_2 + arm_3/arm_4 (all t2_* prims) for the table_1-focused GNG view.
# VISIBILITY ONLY — the articulation keeps every DOF, so the bridge still
# publishes/accepts the full /isaac_joint_states (RViz/move_group use a
# matching table_1-only model). Set GNG_HIDE_T2=0 to keep them visible.
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
    print(f">>> GNG view: hid {_hidden} t2_ prims (table_2 + arm_3/arm_4)", flush=True)

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
        ],
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
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ART)]),
            ("PublishClock.inputs:topicName", "clock"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ArticulationController.inputs:robotPath", ART),
            ("SubscribeTable.inputs:topicName", "isaac_table_commands"),
            ("ArticulationControllerTable.inputs:robotPath", ART),
        ],
    },
)

world.reset()
omni.timeline.get_timeline_interface().play()

# Re-apply drive gains after the final reset/play so they aren't wiped.
wc.get_articulation_controller().set_gains(kps=kps, kds=kds)

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
