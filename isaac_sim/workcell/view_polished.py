"""Polished workcell viewer for noVNC: room (floor + walls + lights), grey tables,
a work table with objects, and the 4 arms gently moving. Ctrl-C to stop.

    DISPLAY=:22380 python view_polished.py
"""
import os
import sys
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from gripper import gripper_indices, apply_gripper, OPEN  # noqa: E402
import polish  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))

# scene polish
objs = polish.build_room()
polish.add_lights()
for o in objs:
    world.scene.add(o)
world.reset()
polish.recolor_tables()

names = list(wc.dof_names)
n = wc.num_dof
groups = gripper_indices(names)
kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
gj = [groups[p]["master"] for p in groups] + [i for p in groups for i, _, _ in groups[p]["mimics"]]
for i in gj:
    kps[i] = 2.0e3; kds[i] = 1.0e2
ctrl = wc.get_articulation_controller()
ctrl.set_gains(kps=kps, kds=kds)
home = wc.get_joint_positions().copy()

arm_j2 = [i for i, nm in enumerate(names) if nm.endswith("_joint_2")]
lin = [i for i, nm in enumerate(names) if nm.endswith("linear_joint")]

print(">>> polished workcell viewer running. Press F in the viewport to frame. Ctrl-C to stop.", flush=True)
t = 0.0
while simulation_app.is_running():
    tgt = home.copy()
    apply_gripper(tgt, groups, OPEN)
    for i in arm_j2:
        tgt[i] = home[i] + 0.35 * math.sin(t)
    for i in lin:
        tgt[i] = home[i] + 0.06 * math.sin(0.5 * t)
    ctrl.apply_action(ArticulationAction(joint_positions=tgt))
    world.step(render=True)
    t += 0.01

simulation_app.close()
