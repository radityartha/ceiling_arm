"""SINGLE-ARM ROS 2 bridge (kortex topic convention, for MoveIt/ros2_control).

Publishes /isaac_joint_states (+ /clock) AND subscribes to /isaac_joint_commands
(sensor_msgs/JointState) -> drives the single Gen3 Lite articulation. These are the
exact topics topic_based_ros2_control/TopicBasedSystem uses when kortex is built with
sim_isaac:=true, so ros2_control + MoveIt drive Isaac through these directly.

VERIFIED CONFIG: CycloneDDS on both sides, matching ROS_DOMAIN_ID, and
world.step(render=True) so the OmniGraph ticks. Isaac-half verified standalone:
commanding joint_2 -> 0.5 moved it to 0.501.

    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python ros2_bridge.py

Manual test from another shell (system Humble, same two env vars):
    ros2 topic pub -r 20 /isaac_joint_commands sensor_msgs/msg/JointState \
        "{name: ['joint_2'], position: [0.5]}"
    ros2 topic echo /isaac_joint_states
"""
import os
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

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
USD_PATH = os.path.join(HERE, "gen3_lite.usd")
ROBOT = "/World/arm"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path=ROBOT)
wc = world.scene.add(Robot(prim_path=ROBOT, name="gen3_lite"))
world.reset()

ART = next((str(p.GetPath()) for p in get_current_stage().Traverse()
            if p.HasAPI(UsdPhysics.ArticulationRootAPI)), ROBOT)
print(f">>> articulation root prim = {ART}", flush=True)

# Stiff drives so commanded positions are reached and held.
n = wc.num_dof
kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
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
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ART)]),
            ("PublishClock.inputs:topicName", "clock"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ArticulationController.inputs:robotPath", ART),
        ],
    },
)

world.reset()
omni.timeline.get_timeline_interface().play()

print(">>> ROS2 bridge: publishing /joint_states, subscribing /joint_command. Ctrl-C to stop.", flush=True)
i = 0
while simulation_app.is_running():
    world.step(render=True)   # render pumps the OmniGraph so nodes tick
    i += 1
    if i % 120 == 0:
        print(f">>> running (step {i})", flush=True)

simulation_app.close()
