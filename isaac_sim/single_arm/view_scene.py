"""GUI viewer: load the imported arm into a scene and render to a window
(for viewing over VNC/noVNC). Keeps the arm holding home pose and slowly
waving joint_2 so motion is visible. Close the window or Ctrl-C to exit.

    DISPLAY=:22380 python view_scene.py
"""
import os
import math
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "gen3_lite.usd")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/arm")
arm = world.scene.add(Robot(prim_path="/World/arm", name="arm"))
world.reset()

ctrl = arm.get_articulation_controller()
ctrl.set_gains(kps=np.full(arm.num_dof, 1.0e5), kds=np.full(arm.num_dof, 1.0e4))
home = arm.get_joint_positions().copy()

print(">>> viewer running; open noVNC to watch. Ctrl-C to stop.", flush=True)
t = 0.0
while simulation_app.is_running():
    target = home.copy()
    target[1] = 0.5 * math.sin(t)      # wave joint_2
    ctrl.apply_action(ArticulationAction(joint_positions=target))
    world.step(render=True)
    t += 0.01

simulation_app.close()
