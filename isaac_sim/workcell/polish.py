"""Scene-polish helpers for the workcell viewer: a room (floor + walls + lights),
a work surface with objects, and grey materials on the otherwise-uncolored tables.
The arm links already carry colors from the URDF, so only the 8 table links need it.
"""
import numpy as np

from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.semantics import add_update_semantics

TABLE_LINKS = [
    "t1_platform_link", "t1_rotation_link", "t1_mount_plate_left", "t1_mount_plate_right",
    "t2_platform_link", "t2_rotation_link", "t2_mount_plate_left", "t2_mount_plate_right",
]


def recolor_tables(root="/World/workcell", color=(0.30, 0.32, 0.36)):
    """Bind a grey material to the 8 table links (override imported default)."""
    from pxr import UsdShade
    stage = get_current_stage()
    grey = OmniPBR(prim_path="/World/Looks/TableGrey", name="table_grey",
                   color=np.array(color))
    mat = UsdShade.Material(stage.GetPrimAtPath(grey.prim_path))
    bound = 0
    for p in stage.Traverse():
        name = p.GetName()
        # bind on the link prim and on its visual mesh prims
        if any(name == lk for lk in TABLE_LINKS) or \
           any(("/" + lk + "/") in str(p.GetPath()) and p.GetTypeName() == "Mesh" for lk in TABLE_LINKS):
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            bound += 1
    return bound


def add_lights(intensity_dome=900.0, intensity_key=2500.0):
    from pxr import UsdLux, Gf, UsdGeom
    stage = get_current_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(intensity_dome)
    dome.CreateColorAttr(Gf.Vec3f(0.9, 0.93, 1.0))
    key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(intensity_key)
    key.CreateAngleAttr(1.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 10.0, 0.0))


def _pbr(path, color):
    return OmniPBR(prim_path=path, name=path.split("/")[-1], color=np.array(color))


def build_room():
    """Floor + 2 side walls (back/front open for the rail-end cameras) + a work
    table with a few objects."""
    floor_mat = _pbr("/World/Looks/Floor", (0.18, 0.19, 0.21))
    wall_mat = _pbr("/World/Looks/Wall", (0.55, 0.56, 0.60))
    top_mat = _pbr("/World/Looks/TableTop", (0.45, 0.30, 0.18))
    leg_mat = _pbr("/World/Looks/Leg", (0.12, 0.12, 0.13))
    cab_mat = _pbr("/World/Looks/Cabinet", (0.35, 0.40, 0.48))

    FixedCuboid(prim_path="/World/room/floor", name="floor",
                position=np.array([0, 0, -0.05]), scale=np.array([12, 12, 0.1]),
                visual_material=floor_mat)
    # Two side walls (y=+-2.5). The back (-x) and front (+x) walls are OMITTED so
    # nothing occludes the two RGBD cameras, which sit near the rail ends
    # (rgbd2 at x=-0.6, rgbd at x=2.8) looking in under the arms.
    FixedCuboid(prim_path="/World/room/wall_left", name="wall_left",
                position=np.array([0, 2.5, 2.0]), scale=np.array([6, 0.1, 4.0]),
                visual_material=wall_mat)
    FixedCuboid(prim_path="/World/room/wall_right", name="wall_right",
                position=np.array([0, -2.5, 2.0]), scale=np.array([6, 0.1, 4.0]),
                visual_material=wall_mat)

    # Work table under the ceiling arms. The arms hang from z~2.05 and their
    # reachable shell (GNG cloud) floors out at z~1.14, so a floor-height table
    # (old top_z=0.78) put objects BELOW reach. Raise the top to z=1.30 so
    # objects rest at z~1.35 -- comfortably inside the shell (not at its lower
    # edge, where manipulability is poor). Legs + obj_z derive from top_z.
    # Footprint sized to the 6-object cluster (~0.55 x 0.38 m) with a small
    # margin rather than an oversized 1.3 x 1.3 slab, and nudged +0.25 m along
    # +X (center 1.5 -> 1.75) to push the objects further from the arm bases
    # while staying inside the gantry's reachable range (rail travels world +X
    # 0..3.0 m, EE reach ~3.6 m). Objects are shifted by the same +0.25 m below
    # so they stay on the top. Legs + obj_z derive from top_z.
    cx, cy, top_z, th = 1.75, 0.0, 1.30, 0.05
    FixedCuboid(prim_path="/World/work_table/top", name="wt_top",
                position=np.array([cx, cy, top_z]), scale=np.array([0.9, 0.7, th]),
                visual_material=top_mat)
    for i, (dx, dy) in enumerate([(0.40, 0.30), (0.40, -0.30), (-0.40, 0.30), (-0.40, -0.30)]):
        FixedCuboid(prim_path=f"/World/work_table/leg_{i}", name=f"wt_leg_{i}",
                    position=np.array([cx + dx, cy + dy, top_z / 2]),
                    scale=np.array([0.05, 0.05, top_z]), visual_material=leg_mat)

    # Second work table on the +Y side ("left" of the arms), slightly SHORTER than
    # table 1 (top_z 1.22 vs 1.30) -- kept just above the arms' reachable z-floor
    # (~1.22 m, measured from the GNG map) so its near objects stay reachable. It
    # is a long bench extended in +Y ON PURPOSE: gantry_1's reachable cloud only
    # spans y up to ~1.08 m, so to host an object that is GENUINELY out of reach
    # (the banana, below) the top has to extend well past that. Near end (small y)
    # carries the reachable objects, far end (+Y) the unreachable one. ~0.15 m gap
    # in Y from table 1 (table 1 +Y edge 0.35, table 2 -Y edge cy2 - sy2/2 = 0.50)
    # so the two tables don't touch.
    cx2, cy2, top_z2, sx2, sy2 = 1.75, 1.25, 1.22, 0.75, 1.5
    FixedCuboid(prim_path="/World/work_table2/top", name="wt2_top",
                position=np.array([cx2, cy2, top_z2]), scale=np.array([sx2, sy2, th]),
                visual_material=top_mat)
    lx, ly = sx2 / 2 - 0.05, sy2 / 2 - 0.05
    for i, (dx, dy) in enumerate([(lx, ly), (lx, -ly), (-lx, ly), (-lx, -ly)]):
        FixedCuboid(prim_path=f"/World/work_table2/leg_{i}", name=f"wt2_leg_{i}",
                    position=np.array([cx2 + dx, cy2 + dy, top_z2 / 2]),
                    scale=np.array([0.05, 0.05, top_z2]), visual_material=leg_mat)

    # A cabinet standing beside the table on the -X side (i.e. "before" the table,
    # toward the rail origin) as a static obstacle for collision testing -- the arm
    # planner must route around it. It is a solid box from the floor up, made
    # SLIGHTLY taller than the table top (top at z=cab_h=1.45 vs the table's
    # ~1.325) so it pokes into the lower workspace. No semantic label on purpose:
    # it stays "background" (seg<=1) so the cameras feed it into the MoveIt
    # collision octomap as environment, not as a graspable object. Depth along X,
    # width matched to the table in Y, placed just off the table's -X edge.
    cab_dx, cab_wy, cab_h = 0.4, 0.35, 1.45
    cab_x = cx - 0.45 - 0.07 - cab_dx / 2         # just before the -X edge of the top
    FixedCuboid(prim_path="/World/cabinet/body", name="cabinet",
                position=np.array([cab_x, cy, cab_h / 2]),
                scale=np.array([cab_dx, cab_wy, cab_h]), visual_material=cab_mat)

    # Real YCB objects (cans / bottle / boxes / banana) instead of primitives, so
    # segmentation + localization + reachability run against realistic geometry.
    # Two asset variants are mixed: 4 ship as *physics* USDs (RigidBodyAPI +
    # ConvexHull collision baked in), the other 3 are visual-only meshes. We turn
    # ALL of them into STATIC colliders -- they never fall, never move, and carry
    # no rigid-body dynamics, so World.reset() never tries to zero a velocity on
    # them (that was the "PxRigidDynamic::setLinearVelocity: Body must be
    # non-kinematic" spam). Static is also what the perception/reachability task
    # wants: fixed objects at known poses. Split: physics objects on table 1
    # (surf1), visual-only on table 2 (surf2). (x, y) are spaced >0.25 m apart so
    # no two objects touch.
    from pxr import Usd, UsdGeom, UsdPhysics
    root = get_assets_root_path()
    if root is None:
        root = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/4.5"
    ycb = f"{root}/Isaac/Props/YCB"
    surf1 = top_z + th / 2
    surf2 = top_z2 + th / 2
    # label, usd (relative to ycb), (x, y, surface_z), has_physics_variant
    specs = [
        ("cracker_box",     "Axis_Aligned_Physics/003_cracker_box.usd",     (1.62, -0.16, surf1), True),
        ("sugar_box",       "Axis_Aligned_Physics/004_sugar_box.usd",       (1.62,  0.16, surf1), True),
        ("tomato_soup_can", "Axis_Aligned_Physics/005_tomato_soup_can.usd", (1.90, -0.16, surf1), True),
        ("mustard_bottle",  "Axis_Aligned_Physics/006_mustard_bottle.usd",  (1.90,  0.16, surf1), True),
        ("tuna_fish_can",   "Axis_Aligned/007_tuna_fish_can.usd",           (1.60,  0.70, surf2), False),
        ("potted_meat_can", "Axis_Aligned/010_potted_meat_can.usd",         (1.92,  0.72, surf2), False),
        # banana sits at the far +Y end (y=1.85) -- ~0.77 m past the reachable
        # cloud's y-max (1.08 m), beyond the ~0.73 m pool radius, so the reachability
        # check finds NO candidate nodes and reports it UNREACHABLE on purpose.
        ("banana",          "Axis_Aligned/011_banana.usd",                  (1.75,  1.85, surf2), False),
    ]
    objs = []
    stage = get_current_stage()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for i, (label, rel, (ox, oy, surf), has_phys) in enumerate(specs):
        prim_path = f"/World/objects/obj_{i}"
        add_reference_to_stage(usd_path=f"{ycb}/{rel}", prim_path=prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        # Measure the asset's own bounds (still at identity) and seat it BEFORE
        # building the wrapper: bottom exactly on the table top, centered on
        # (ox, oy). These YCB meshes are center-origin, so placing the origin on
        # the surface would bury half the body. The seated pose must be passed to
        # the constructor (not set afterwards) because World.reset() restores each
        # scene object to its construction-time default pose -- a later
        # set_world_pose would be reverted, which is what left them sunk before.
        rng = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            pos = np.array([ox, oy, surf])      # fallback: origin on the surface
        else:
            mn, mx = rng.GetMin(), rng.GetMax()
            pos = np.array([ox - 0.5 * (mn[0] + mx[0]),
                            oy - 0.5 * (mn[1] + mx[1]),
                            surf - mn[2]])
        if has_phys:
            # disable the asset's baked rigid-body dynamics -> static collider.
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        else:
            # visual-only mesh -> add a static convex-hull collider on each mesh.
            for d in Usd.PrimRange(prim):
                if d.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(d)
                    UsdPhysics.MeshCollisionAPI.Apply(d).CreateApproximationAttr("convexHull")
        obj = SingleXFormPrim(prim_path=prim_path, name=f"obj_{i}", position=pos)
        objs.append(obj)
        # Semantic class label -> the cameras' instance_segmentation publishes a
        # labeled mask + an id->class JSON, so the localizer can name each
        # detected object. Ground-truth seg now; swap source for open-vocab later.
        add_update_semantics(prim, label)
    return objs
