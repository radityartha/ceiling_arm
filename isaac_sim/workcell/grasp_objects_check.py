"""Headless validation of polish.py's GRASP_OBJECTS=1 path: build the room with
all 8 YCB objects made DYNAMIC (RigidBodyEnabled + mass + convexDecomposition +
friction), step, and confirm every object settles in place instead of sliding
off its table or free-falling (has_phys objects flip RigidBodyEnabled off->on;
visual-only objects gain a fresh RigidBodyAPI -- either path could go wrong).

    GRASP_OBJECTS=1 python grasp_objects_check.py   ->   grasp_objects_check.txt

Pass bar (Step 1 of the isaac-grasping plan): every object's horizontal (xy)
drift after settling < 5 mm, and no object's z drops more than a few cm (i.e.
it did not fall through/off its table). Numbers are reported as-measured even
if they miss the bar -- see Rule 12.
"""
import os
os.environ.setdefault("GRASP_OBJECTS", "1")  # must be set BEFORE `import polish`

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
    out(f"polish.GRASP_OBJECTS = {polish.GRASP_OBJECTS} (expect True)")
    assert polish.GRASP_OBJECTS, "GRASP_OBJECTS env var was not read as enabled"

    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
    wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
    objs = polish.build_room()  # returns the 8 obj_i wrappers (no pick_cube/coalition_box here)
    polish.add_lights()
    for o in objs:
        world.scene.add(o)
    world.reset()

    labels = [spec[0] for spec in [
        ("cracker_box",), ("sugar_box",), ("tomato_soup_can",), ("mustard_bottle",),
        ("teddy_bear",), ("mug",), ("banana",), ("beaker",),
    ]]
    p0 = [np.array(o.get_world_pose()[0], dtype=float) for o in objs]
    out(f"initial xyz: {[list(np.round(p, 3)) for p in p0]}")

    STEPS = 300  # ~5 s at 60 Hz -- long enough to settle, short enough to catch a slide
    for _ in range(STEPS):
        world.step(render=False)

    p1 = [np.array(o.get_world_pose()[0], dtype=float) for o in objs]
    out(f"final   xyz: {[list(np.round(p, 3)) for p in p1]}")

    all_ok = True
    for label, a, b in zip(labels, p0, p1):
        xy_drift = float(np.linalg.norm(b[:2] - a[:2]))
        z_drop = float(a[2] - b[2])
        ok = xy_drift < 0.005 and z_drop < 0.03
        all_ok &= ok
        out(f"  {label:16s} xy_drift={xy_drift:.4f} m  z_drop={z_drop:+.4f} m  "
            f"{'PASS' if ok else 'FAIL'}")

    out(f"\nGRASP_OBJECTS SETTLE CHECK: {'PASS' if all_ok else 'FAIL (see per-object above)'}")
except Exception as e:
    import traceback
    out("GRASP_OBJECTS SETTLE CHECK: FAIL (exception)\n" + traceback.format_exc())

with open(os.path.join(HERE, "grasp_objects_check.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")
simulation_app.close()
