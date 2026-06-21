"""Workcell scene test: load into physics scene, hold pose (stability),
then command one arm joint + one table linear joint (controllability).

    python test_scene.py   ->   phase23_verify.txt
"""
import os
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")

log = []
def out(m):
    print(m); log.append(str(m))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
world.reset()

n = wc.num_dof
names = list(wc.dof_names)
out(f"num_dof : {n}")
out(f"dof_names: {names}")

ctrl = wc.get_articulation_controller()
# revolute joints stiff; prismatic (table linear, meters) also stiff
ctrl.set_gains(kps=np.full(n, 1.0e5), kds=np.full(n, 1.0e4))

home = wc.get_joint_positions().copy()
ctrl.apply_action(ArticulationAction(joint_positions=home))
for _ in range(300):
    world.step(render=False)
drift = np.max(np.abs(wc.get_joint_positions() - home))
stable = drift < 0.05
out(f"\n[STABILITY] max drift = {drift:.4f}  -> {'PASS' if stable else 'FAIL'}")

# pick test joints by name
arm_j = next((i for i, nm in enumerate(names) if nm == "t1_a1_joint_2"), None)
lin_j = next((i for i, nm in enumerate(names) if nm == "t1_linear_joint"), None)
out(f"\ntest indices: t1_a1_joint_2={arm_j}  t1_linear_joint={lin_j}")

target = home.copy()
if arm_j is not None:
    target[arm_j] += 0.4
if lin_j is not None:
    target[lin_j] += 0.10   # 10 cm table slide
ctrl.apply_action(ArticulationAction(joint_positions=target))
for _ in range(500):
    world.step(render=False)
reached = wc.get_joint_positions()

ok = True
if arm_j is not None:
    e = abs(reached[arm_j] - target[arm_j]); ok &= e < 0.05
    out(f"[CONTROL] t1_a1_joint_2: target {target[arm_j]:.3f} reached {reached[arm_j]:.3f} err {e:.4f}")
if lin_j is not None:
    e = abs(reached[lin_j] - target[lin_j]); ok &= e < 0.02
    out(f"[CONTROL] t1_linear_joint: target {target[lin_j]:.3f} reached {reached[lin_j]:.3f} err {e:.4f}")

out(f"\nOVERALL: {'PASS' if (stable and ok) else 'FAIL'}")
with open(os.path.join(HERE, "phase23_verify.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")

simulation_app.close()
