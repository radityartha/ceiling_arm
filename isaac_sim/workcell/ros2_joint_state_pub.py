"""ROS 2 bridge - step 1 (joint states OUT only).

Loads the workcell, enables the Isaac ROS 2 bridge, builds a minimal OmniGraph that
publishes sensor_msgs/JointState on /joint_states (+ /clock), and runs the sim.
The bridge uses its bundled Humble libraries, so this process needs NO system ROS.

VERIFIED WORKING CONFIG (Isaac<->system DDS interop on this host):
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp on BOTH sides (system default here; UDP,
    no shared-memory). FastDDS gave discovery-only (data blocked by SHM mismatch).
  - ROS_DOMAIN_ID=42 (any value, must match; isolates from other ROS traffic on host).
  - The loop must use world.step(render=True): render pumps the OmniGraph so the
    OnPlaybackTick node actually fires and writes samples (render=False advertises
    the topic but never publishes data).

Launch:
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python ros2_joint_state_pub.py
Verify (another shell, system Humble sourced, same two env vars):
    ros2 topic echo /joint_states --once     # -> 44 joints
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
ROBOT = "/World/workcell"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT)
wc = world.scene.add(Robot(prim_path=ROBOT, name="workcell"))
world.reset()

# The publish node needs the exact prim carrying ArticulationRootAPI.
ART = next((str(p.GetPath()) for p in get_current_stage().Traverse()
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)), ROBOT)
print(f">>> articulation root prim = {ART}", flush=True)

# Minimal publish graph.
og.Controller.edit(
    {"graph_path": "/ROS2_JointStates", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:topicName", "joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ART)]),
            ("PublishClock.inputs:topicName", "clock"),
        ],
    },
)

world.reset()
import omni.timeline  # noqa: E402
omni.timeline.get_timeline_interface().play()

print(">>> ROS2 bridge publishing /joint_states and /clock. Ctrl-C to stop.", flush=True)
i = 0
while simulation_app.is_running():
    world.step(render=True)   # render=True pumps the app/OmniGraph so OnPlaybackTick fires
    i += 1
    if i % 120 == 0:
        print(f">>> still publishing (step {i})", flush=True)

simulation_app.close()
