"""noVNC grasp demo (single arm): open the gripper, drop a cube into it, close to
grasp (1-DOF master command), then swing the base joint so the held cube travels
with the gripper -- visual proof the grasp holds. Ctrl-C to stop.

    DISPLAY=:22380 python view_grasp.py
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
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.core.api.materials import PhysicsMaterial  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from gripper import gripper_indices, apply_gripper, OPEN, CLOSED  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "gen3_lite.usd")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/arm")
arm = world.scene.add(Robot(prim_path="/World/arm", name="arm"))
world.reset()

names = list(arm.dof_names)
n = arm.num_dof
groups = gripper_indices(names)
kps = np.full(n, 1.0e5); kds = np.full(n, 1.0e4)
gj = [groups[p]["master"] for p in groups] + [i for p in groups for i, _, _ in groups[p]["mimics"]]
for i in gj:
    kps[i] = 2.0e3; kds[i] = 1.0e2
ctrl = arm.get_articulation_controller()
ctrl.set_gains(kps=kps, kds=kds)
home = arm.get_joint_positions().copy()

def hold(action_pos, steps):
    for _ in range(steps):
        ctrl.apply_action(ArticulationAction(joint_positions=action_pos)); world.step(render=True)

# open and settle
tgt = home.copy(); apply_gripper(tgt, groups, OPEN); hold(tgt, 90)

# spawn cube at the open grasp point
stage = get_current_stage()
def find(nm):
    return next((str(p.GetPath()) for p in stage.Traverse() if p.GetName() == nm), None)
rp = SingleXFormPrim(find("right_finger_dist_link")).get_world_pose()[0]
lp = SingleXFormPrim(find("left_finger_dist_link")).get_world_pose()[0]
mid = (np.array(rp) + np.array(lp)) / 2.0
mat = PhysicsMaterial(prim_path="/World/cube_mat", static_friction=1.2, dynamic_friction=1.2, restitution=0.0)
cube = world.scene.add(DynamicCuboid(prim_path="/World/cube", name="cube", position=mid,
                                     size=0.02, color=np.array([1.0, 0.2, 0.2]), mass=0.02))
cube.apply_physics_material(mat)
world.reset(); ctrl.set_gains(kps=kps, kds=kds)

# grasp
tgt = home.copy(); apply_gripper(tgt, groups, CLOSED); hold(tgt, 120)

print(">>> grasp demo running: swinging the held cube. Ctrl-C to stop.", flush=True)
j1 = names.index("joint_1")
t = 0.0
while simulation_app.is_running():
    tgt = home.copy()
    apply_gripper(tgt, groups, CLOSED)      # keep holding
    tgt[j1] = home[j1] + 0.6 * math.sin(t)  # swing base so cube travels with gripper
    ctrl.apply_action(ArticulationAction(joint_positions=tgt))
    world.step(render=True)
    t += 0.01

simulation_app.close()
