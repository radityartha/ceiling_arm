"""Phase 1: headless import of the single Gen3 Lite arm URDF into a USD asset.

Run with the Isaac Sim 4.5 python (venv active):
    python import_urdf.py

Produces gen3_lite.usd next to this script and prints a verification summary.
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdPhysics, PhysxSchema  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "gen3_lite.urdf")
USD_PATH = os.path.join(HERE, "gen3_lite.usd")

# Fresh stage.
omni.usd.get_context().new_stage()

# Import configuration.
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True      # collapse fixed joints (gripper base, tool frames)
import_config.fix_base = True                # anchor `world` link (ceiling mount)
import_config.make_default_prim = True
import_config.distance_scale = 1.0           # URDF is in meters; stage is meters
import_config.import_inertia_tensor = True   # use inertia from the URDF
import_config.create_physics_scene = True
import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_position_drive_damping = 1e3
import_config.parse_mimic = False            # PhysX hard-mimic unreliable here; couple in software (see gripper.py)

print(">>> importing:", URDF_PATH)
result = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=USD_PATH,
)
print(">>> import command result:", result)

# ---- Verification: reopen the written USD and summarize ----
stage = Usd.Stage.Open(USD_PATH)
prims = list(stage.Traverse())
joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
revolute = [p for p in prims if p.GetTypeName() == "PhysicsRevoluteJoint"]
prismatic = [p for p in prims if p.GetTypeName() == "PhysicsPrismaticJoint"]
meshes = [p for p in prims if p.GetTypeName() == "Mesh"]
art_root = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]

lines = [
    "========== USD VERIFICATION ==========",
    f"default prim       : {stage.GetDefaultPrim().GetPath()}",
    f"total prims        : {len(prims)}",
    f"mesh prims         : {len(meshes)}",
    f"physics joints     : {len(joints)} (revolute: {len(revolute)} prismatic: {len(prismatic)})",
    f"revolute names     : {[p.GetName() for p in revolute]}",
    f"prismatic names    : {[p.GetName() for p in prismatic]}",
    f"mimic joints       : {[p.GetName() for p in prims if p.HasAPI(PhysxSchema.PhysxMimicJointAPI)]}",
    f"articulation roots : {[str(p.GetPath()) for p in art_root]}",
    f"usd written to     : {USD_PATH} exists: {os.path.exists(USD_PATH)}",
    "======================================",
]
report = "\n".join(lines)
print(report)
with open(os.path.join(HERE, "verify.txt"), "w") as fh:
    fh.write(report + "\n")

simulation_app.close()
