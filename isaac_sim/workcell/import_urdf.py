"""Import the full workcell URDF (4 arms + 2 tables) into a USD asset.

    python import_urdf.py
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "workcell.urdf")
USD_PATH = os.path.join(HERE, "workcell.usd")

omni.usd.get_context().new_stage()

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True
import_config.fix_base = True                 # anchor `world` (ceiling)
import_config.make_default_prim = True
import_config.distance_scale = 1.0
import_config.import_inertia_tensor = True
import_config.create_physics_scene = True
import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_position_drive_damping = 1e3

result = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    dest_path=USD_PATH,
)

stage = Usd.Stage.Open(USD_PATH)
prims = list(stage.Traverse())
rev = [p for p in prims if p.GetTypeName() == "PhysicsRevoluteJoint"]
pris = [p for p in prims if p.GetTypeName() == "PhysicsPrismaticJoint"]
art = [p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]

lines = [
    "========== WORKCELL USD VERIFICATION ==========",
    f"default prim       : {stage.GetDefaultPrim().GetPath()}",
    f"total prims        : {len(prims)}",
    f"revolute joints    : {len(rev)}",
    f"prismatic joints   : {len(pris)}  {[p.GetName() for p in pris]}",
    f"articulation roots : {[str(p.GetPath()) for p in art]}",
    f"usd written        : {os.path.exists(USD_PATH)}",
    "===============================================",
]
report = "\n".join(lines)
print(report)
with open(os.path.join(HERE, "verify.txt"), "w") as fh:
    fh.write(report + "\n")

simulation_app.close()
