"""GUI viewer for the full workcell over VNC/noVNC.
Gently animates each arm's joint_2 and slides both tables so motion is visible.

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
USD_PATH = os.path.join(HERE, "workcell.usd")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
world.reset()

n = wc.num_dof
names = list(wc.dof_names)
ctrl = wc.get_articulation_controller()
ctrl.set_gains(kps=np.full(n, 1.0e5), kds=np.full(n, 1.0e4))
home = wc.get_joint_positions().copy()

arm_j2 = [i for i, nm in enumerate(names) if nm.endswith("_joint_2")]
lin = [i for i, nm in enumerate(names) if nm.endswith("linear_joint")]

print(">>> workcell viewer running; open noVNC to watch. Ctrl-C to stop.", flush=True)
t = 0.0
while simulation_app.is_running():
    target = home.copy()
    for i in arm_j2:
        target[i] = home[i] + 0.4 * math.sin(t)
    for i in lin:
        target[i] = home[i] + 0.08 * math.sin(0.5 * t)
    ctrl.apply_action(ArticulationAction(joint_positions=target))
    world.step(render=True)
    t += 0.01

simulation_app.close()
