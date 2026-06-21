"""Workcell grasp test: confirm gripper.py drives all 4 grippers as 1-DOF, and
perform a grasp with one ceiling arm (t1_a1_, which points down -> top-down pick).

    python grasp_test.py   ->   grasp_verify.txt
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.core.api.materials import PhysicsMaterial  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from gripper import gripper_indices, apply_gripper, OPEN, CLOSED  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
ARM = "t1_a1_"   # which gripper to grasp with

log = []
def out(m):
    print(m); log.append(str(m))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
world.reset()

names = list(wc.dof_names)
n = wc.num_dof
groups = gripper_indices(names)
out(f"gripper groups found: {sorted(groups.keys())}  (expect 4)")
assert len(groups) == 4, "expected 4 grippers"

kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
gjoints = [groups[p]["master"] for p in groups] + [i for p in groups for i, _, _ in groups[p]["mimics"]]
for i in gjoints:
    kps[i] = 2.0e3; kds[i] = 1.0e2
ctrl = wc.get_articulation_controller()
ctrl.set_gains(kps=kps, kds=kds)
home = wc.get_joint_positions().copy()

# open all grippers, settle
tgt = home.copy()
apply_gripper(tgt, groups, OPEN)
for _ in range(80):
    ctrl.apply_action(ArticulationAction(joint_positions=tgt)); world.step(render=False)

stage = get_current_stage()
def find(nm):
    for p in stage.Traverse():
        if p.GetName() == nm:
            return str(p.GetPath())
    return None
rp = SingleXFormPrim(find(ARM + "right_finger_dist_link")).get_world_pose()[0]
lp = SingleXFormPrim(find(ARM + "left_finger_dist_link")).get_world_pose()[0]
base = SingleXFormPrim(find(ARM + "gripper_base_link")).get_world_pose()[0]
mid = (np.array(rp) + np.array(lp)) / 2.0
gap = float(np.linalg.norm(np.array(rp) - np.array(lp)))
# move grasp point ~1.2 cm deeper into the jaw (toward gripper base) for a solid pinch
to_base = np.array(base) - mid
to_base = to_base / (np.linalg.norm(to_base) + 1e-9)
mid = mid + 0.012 * to_base
out(f"{ARM} pad gap = {gap:.3f} m, grasp point (deep) = {np.round(mid,3)}")

size = float(min(0.015, 0.7 * gap))
mat = PhysicsMaterial(prim_path="/World/cube_mat", static_friction=1.2,
                      dynamic_friction=1.2, restitution=0.0)
cube = world.scene.add(DynamicCuboid(prim_path="/World/cube", name="cube",
                                     position=mid, size=size,
                                     color=np.array([1.0, 0.2, 0.2]), mass=0.02))
cube.apply_physics_material(mat)
world.reset(); ctrl.set_gains(kps=kps, kds=kds)
mid_z0 = float(mid[2])

# close ONLY this arm's gripper
tgt = home.copy()
apply_gripper(tgt, groups, {ARM: CLOSED})
for _ in range(300):
    ctrl.apply_action(ArticulationAction(joint_positions=tgt)); world.step(render=False)

# diagnostics
q = wc.get_joint_positions()
m_idx = groups[ARM]["master"]
rp2 = SingleXFormPrim(find(ARM + "right_finger_dist_link")).get_world_pose()[0]
lp2 = SingleXFormPrim(find(ARM + "left_finger_dist_link")).get_world_pose()[0]
gap2 = float(np.linalg.norm(np.array(rp2) - np.array(lp2)))
cpos = cube.get_world_pose()[0]
out(f"[diag] master cmd={CLOSED:.2f} actual={q[m_idx]:.3f}  pad gap open->close: {gap:.3f}->{gap2:.3f}")
out(f"[diag] cube final xyz = {np.round(cpos,3)} (grasp xy was {np.round(mid[:2],3)})")

cz = float(cpos[2])
drop = mid_z0 - cz
held = (cz > size / 2.0 + 0.1) and (drop < 0.08)
out(f"\ncube start z = {mid_z0:.3f}, final z = {cz:.3f}, drop = {drop:.3f} m")
out(f"GRASP ({ARM}): {'PASS (held)' if held else 'FAIL (dropped)'}")

with open(os.path.join(HERE, "grasp_verify.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")
simulation_app.close()
