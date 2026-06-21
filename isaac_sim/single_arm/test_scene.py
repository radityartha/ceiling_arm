"""Phase 2+3: load the imported arm into a minimal physics scene, hold pose
under gravity (stability), then command a joint move (controllability).

Run with the Isaac Sim 4.5 python (venv active):
    python test_scene.py

Writes results to phase23_verify.txt. PASS criteria:
  - stability: max joint drift while holding default pose < 0.05 rad over 300 steps
  - control : commanded joint reaches target within 0.05 rad over 400 steps
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
USD_PATH = os.path.join(HERE, "gen3_lite.usd")

log = []
def out(msg):
    print(msg)
    log.append(str(msg))

# ---- Scene ----
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/arm")
arm = world.scene.add(Robot(prim_path="/World/arm", name="arm"))
world.reset()

n = arm.num_dof
out(f"num_dof   : {n}")
out(f"dof_names : {arm.dof_names}")

# Deterministic position-drive gains so the arm holds pose under gravity.
ctrl = arm.get_articulation_controller()
ctrl.set_gains(kps=np.full(n, 1.0e5), kds=np.full(n, 1.0e4))

# ---- Stability: hold the default pose under gravity ----
home = arm.get_joint_positions().copy()
ctrl.apply_action(ArticulationAction(joint_positions=home))
for _ in range(300):
    world.step(render=False)
drift = np.max(np.abs(arm.get_joint_positions() - home))
stable = drift < 0.05
out(f"\n[STABILITY] home pose      : {np.round(home, 3)}")
out(f"[STABILITY] max drift (rad) : {drift:.4f}  -> {'PASS' if stable else 'FAIL'}")

# ---- Control: command joint_2 (index 1) to move +0.4 rad ----
target = home.copy()
target[1] += 0.4
ctrl.apply_action(ArticulationAction(joint_positions=target))
for _ in range(400):
    world.step(render=False)
reached = arm.get_joint_positions()
err = np.abs(reached[1] - target[1])
controllable = err < 0.05
out(f"\n[CONTROL] joint_2 target    : {target[1]:.3f} rad")
out(f"[CONTROL] joint_2 reached   : {reached[1]:.3f} rad")
out(f"[CONTROL] error (rad)       : {err:.4f}  -> {'PASS' if controllable else 'FAIL'}")

out(f"\nOVERALL: {'PASS' if (stable and controllable) else 'FAIL'}")

with open(os.path.join(HERE, "phase23_verify.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")

simulation_app.close()
