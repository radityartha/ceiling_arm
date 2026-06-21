"""Grasp test: spawn a cube between the gripper pads, command ONLY the master
(1-DOF, via software coupling), and confirm the cube is held against gravity.

PASS: after closing, the cube stays near the grasp point (didn't fall to the floor).

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
USD_PATH = os.path.join(HERE, "gen3_lite.usd")

log = []
def out(m):
    print(m); log.append(str(m))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/arm")
arm = world.scene.add(Robot(prim_path="/World/arm", name="arm"))
world.reset()

names = list(arm.dof_names)
n = arm.num_dof
groups = gripper_indices(names)
out(f"gripper groups: { {k: v['master'] for k, v in groups.items()} }")

# Gains: arm stiff; gripper joints moderate (apply grip force on contact).
kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
gjoints = [groups[p]["master"] for p in groups] + [i for p in groups for i, _, _ in groups[p]["mimics"]]
for i in gjoints:
    kps[i] = 1.0e3; kds[i] = 1.0e2
ctrl = arm.get_articulation_controller()
ctrl.set_gains(kps=kps, kds=kds)

home = arm.get_joint_positions().copy()

# Open and settle.
tgt = home.copy()
apply_gripper(tgt, groups, OPEN)
for _ in range(80):
    ctrl.apply_action(ArticulationAction(joint_positions=tgt)); world.step(render=False)

# Locate finger pads.
stage = get_current_stage()
def find(name):
    for p in stage.Traverse():
        if p.GetName() == name:
            return str(p.GetPath())
    return None
rp = SingleXFormPrim(find("right_finger_dist_link")).get_world_pose()[0]
lp = SingleXFormPrim(find("left_finger_dist_link")).get_world_pose()[0]
mid = (np.array(rp) + np.array(lp)) / 2.0
gap = float(np.linalg.norm(np.array(rp) - np.array(lp)))
out(f"pad gap (open) = {gap:.3f} m, grasp point = {np.round(mid,3)}")

# Spawn a light cube at the grasp point, with friction.
size = float(min(0.02, 0.7 * gap))
mat = PhysicsMaterial(prim_path="/World/cube_mat", static_friction=1.2,
                      dynamic_friction=1.2, restitution=0.0)
cube = world.scene.add(DynamicCuboid(
    prim_path="/World/cube", name="cube", position=mid, size=size,
    color=np.array([1.0, 0.2, 0.2]), mass=0.02))
cube.apply_physics_material(mat)
world.reset()  # registers the new cube in physics
ctrl.set_gains(kps=kps, kds=kds)
mid_z0 = mid[2]

# Close the gripper (single command) and hold.
tgt = home.copy()
apply_gripper(tgt, groups, CLOSED)
for _ in range(300):
    ctrl.apply_action(ArticulationAction(joint_positions=tgt)); world.step(render=False)

cube_pos = cube.get_world_pose()[0]
drop = mid_z0 - float(cube_pos[2])
floor_z = size / 2.0
held = (float(cube_pos[2]) > floor_z + 0.03) and (drop < 0.06)
out(f"\ncube start z = {mid_z0:.3f}, final z = {float(cube_pos[2]):.3f}, drop = {drop:.3f} m")
out(f"floor would be z ~ {floor_z:.3f}")
out(f"\nGRASP: {'PASS (held)' if held else 'FAIL (dropped)'}")

with open(os.path.join(HERE, "grasp_verify.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")

simulation_app.close()
