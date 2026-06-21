"""Headless validation of polish.py: build the room/materials/objects and step,
catching any errors before launching the GUI viewer.  -> polish_check.txt
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
import polish  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
log = []
def out(m):
    print(m); log.append(str(m))

try:
    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
    wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
    objs = polish.build_room()
    polish.add_lights()
    for o in objs:
        world.scene.add(o)
    world.reset()
    bound = polish.recolor_tables()
    for _ in range(60):
        world.step(render=False)
    out(f"table material bindings applied : {bound}")
    out(f"objects on work table          : {len(objs)}")
    zs = [float(o.get_world_pose()[0][2]) for o in objs]
    out(f"object z after settle          : {[round(z,3) for z in zs]} (table top ~0.805)")
    out("POLISH CHECK: PASS")
except Exception as e:
    import traceback
    out("POLISH CHECK: FAIL\n" + traceback.format_exc())

with open(os.path.join(HERE, "polish_check.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")
simulation_app.close()
