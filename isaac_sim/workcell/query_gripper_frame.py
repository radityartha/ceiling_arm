"""One-off query: t1_a1 gripper link world poses, and the same points expressed
in gripper_base_link's OWN local frame -- so a wrist-camera mount offset can be
picked from measured numbers instead of guessed ones.

    python query_gripper_frame.py   ->   stdout + query_gripper_frame.txt
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402
from isaacsim.core.utils.rotations import quat_to_rot_matrix  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
USD_PATH = os.path.join(HERE, "workcell.usd")
log = []
def out(m):
    print(m); log.append(str(m))

world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/workcell")
wc = world.scene.add(Robot(prim_path="/World/workcell", name="workcell"))
world.reset()

stage = get_current_stage()
def find(nm):
    for p in stage.Traverse():
        if p.GetName() == nm:
            return str(p.GetPath())
    return None

ARM = "t1_a1_"
LINKS = ["gripper_base_link", "right_finger_dist_link", "left_finger_dist_link",
         "right_finger_prox_link", "left_finger_prox_link", "end_effector_link",
         "lower_wrist_link"]

poses = {}
for lk in LINKS:
    path = find(ARM + lk)
    if path is None:
        out(f"{lk}: NOT FOUND (path={ARM+lk})")
        continue
    pos, quat = SingleXFormPrim(path).get_world_pose()
    poses[lk] = (np.array(pos, dtype=float), np.array(quat, dtype=float))
    out(f"{lk}: world pos={np.round(pos,4)} quat(wxyz)={np.round(quat,4)}")

if "gripper_base_link" in poses:
    gp, gq = poses["gripper_base_link"]
    Rg = quat_to_rot_matrix(gq)  # world <- gripper_base_link local
    out("\n--- same points in gripper_base_link's LOCAL frame ---")
    for lk, (p, q) in poses.items():
        local = Rg.T @ (p - gp)
        out(f"{lk}: local offset from gripper_base_link = {np.round(local, 4)}")

    if "right_finger_dist_link" in poses and "left_finger_dist_link" in poses:
        rp = poses["right_finger_dist_link"][0]
        lp = poses["left_finger_dist_link"][0]
        mid_world = (rp + lp) / 2.0
        mid_local = Rg.T @ (mid_world - gp)
        pad_gap = float(np.linalg.norm(rp - lp))
        out(f"\nfinger midpoint (grasp point), world = {np.round(mid_world,4)}")
        out(f"finger midpoint, gripper_base_link-local = {np.round(mid_local,4)}")
        out(f"pad gap (open) = {pad_gap:.4f} m")

with open(os.path.join(HERE, "query_gripper_frame.txt"), "w") as fh:
    fh.write("\n".join(log) + "\n")
simulation_app.close()
