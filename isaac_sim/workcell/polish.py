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
    # Two side walls, 2.4 m apart (y=+-1.2, narrowed from +-2.5). The front (+x)
    # wall is still OMITTED so nothing occludes the rgbd camera (x=4.35) looking
    # in under the arms; the back (-x) side is now CLOSED by wall_back below
    # (rgbd2 at x=-0.6 sits inside the room, between wall_back and the work
    # tables, so it keeps its view). X length kept at 10 (span -5..5) so the
    # walls still run past every table/camera along the rail.
    FixedCuboid(prim_path="/World/room/wall_left", name="wall_left",
                position=np.array([0, 1.2, 2.0]), scale=np.array([10, 0.1, 4.0]),
                visual_material=wall_mat)
    FixedCuboid(prim_path="/World/room/wall_right", name="wall_right",
                position=np.array([0, -1.2, 2.0]), scale=np.array([10, 0.1, 4.0]),
                visual_material=wall_mat)
    # Back wall, 2 m behind the gantry bases (world x~0, per CLAUDE.md) closing
    # the previously-open -X end. Spans slightly past +-1.2 (2.6 vs the 2.4 m
    # room width) so it seals the two side-wall corners with no gap.
    FixedCuboid(prim_path="/World/room/wall_back", name="wall_back",
                position=np.array([-2.0, 0, 2.0]), scale=np.array([0.1, 2.6, 4.0]),
                visual_material=wall_mat)

    # Work table under the ceiling arms. The arms hang from z~2.05 and their
    # reachable GNG cloud floors out at z~1.22. top_z=1.05 puts the top ~0.17 m
    # below that floor, but the 0.727 m pool radius (reachability heuristic)
    # bridges the gap, so objects at z~1.10 still find candidate nodes and stay
    # reachable. Legs + obj_z derive from top_z. Footprint sized to the object
    # cluster with a small margin. Rail travels world +X 0..3.0 m, EE reach
    # ~3.6 m, so this stays inside the gantry's X range.
    # Center pushed to x=2.9 (was 1.55, delta +1.35) to widen the pick-distance
    # range for J/energy calibration -- the far object (tomato_soup_can, was
    # 1.70) lands at ~3.05, ~0.25 m inside the ~3.6 m reach cap. Object X specs
    # below and the cabinet (cab_x, derived from cx) shift with it.
    # cy pushed to -0.80 (was 0.0) so the table's -Y edge sits flush against
    # wall_right (inner face at y=-1.2+0.05=-1.15): cy = -1.15 + sy/2 = -0.80.
    # The cabinet shares this `cy` (see below), so it moves to the same wall too.
    cx, cy, top_z, th = 2.9, -0.80, 1.05, 0.05
    FixedCuboid(prim_path="/World/work_table/top", name="wt_top",
                position=np.array([cx, cy, top_z]), scale=np.array([0.9, 0.7, th]),
                visual_material=top_mat)
    for i, (dx, dy) in enumerate([(0.40, 0.30), (0.40, -0.30), (-0.40, 0.30), (-0.40, -0.30)]):
        FixedCuboid(prim_path=f"/World/work_table/leg_{i}", name=f"wt_leg_{i}",
                    position=np.array([cx + dx, cy + dy, top_z / 2]),
                    scale=np.array([0.05, 0.05, top_z]), visual_material=leg_mat)

    # Second work table on the +Y side ("left" of the arms), slightly lower than
    # table 1 (top_z2 0.97 vs 1.05). Rotated 90 deg (sx2/sy2 swapped from the
    # original 0.75/1.30) and pushed flush against wall_left: wall_left's inner
    # face is at y=1.2-0.05=1.15, so cy2 = 1.15 - sy2/2 = 0.775.
    # cx2 advanced +0.55 (0.75 -> 1.30) so the bench clears work_table3 below,
    # which now cuts straight across the room at x~0 (table3's +X edge is 0.375,
    # so 0.65 near-edge clearance = 0.275 m gap).
    # cx2 advanced another +0.50 (requested; 1.30 -> 1.80). New x span is
    # 1.15..2.45, touching table1's -X edge (2.45) exactly but in a disjoint Y
    # band (table2 y:[0.40,1.15] vs table1 y:[-1.15,-0.45]), so no collision.
    cx2, cy2, top_z2, sx2, sy2 = 1.80, 0.775, 0.97, 1.30, 0.75
    FixedCuboid(prim_path="/World/work_table2/top", name="wt2_top",
                position=np.array([cx2, cy2, top_z2]), scale=np.array([sx2, sy2, th]),
                visual_material=top_mat)
    lx, ly = sx2 / 2 - 0.05, sy2 / 2 - 0.05
    for i, (dx, dy) in enumerate([(lx, ly), (lx, -ly), (-lx, ly), (-lx, -ly)]):
        FixedCuboid(prim_path=f"/World/work_table2/leg_{i}", name=f"wt2_leg_{i}",
                    position=np.array([cx2 + dx, cy2 + dy, top_z2 / 2]),
                    scale=np.array([0.05, 0.05, top_z2]), visual_material=leg_mat)

    # Third work table: sits under the gantry's rail-start (x=0, the origin of
    # the 0..2.0 m rail travel) and spans the full room width, wall_left to
    # wall_right (inner faces at y=+-1.15) -- a cross-bench rather than a
    # wall-side one. sy3=2.28 leaves a 0.01 m gap to each wall so it doesn't
    # z-fight the wall geometry. Shallow in X (sx3=0.75, like the other
    # benches) and centered on x=0 so it clears work_table2 (whose near edge
    # is now at cx2-sx2/2 = 0.65, a 0.275 m gap from table3's +X edge at 0.375).
    cx3, cy3, top_z3, sx3, sy3 = 0.0, 0.0, 0.97, 0.75, 2.28
    FixedCuboid(prim_path="/World/work_table3/top", name="wt3_top",
                position=np.array([cx3, cy3, top_z3]), scale=np.array([sx3, sy3, th]),
                visual_material=top_mat)
    lx3, ly3 = sx3 / 2 - 0.05, sy3 / 2 - 0.05
    for i, (dx, dy) in enumerate([(lx3, ly3), (lx3, -ly3), (-lx3, ly3), (-lx3, -ly3)]):
        FixedCuboid(prim_path=f"/World/work_table3/leg_{i}", name=f"wt3_leg_{i}",
                    position=np.array([cx3 + dx, cy3 + dy, top_z3 / 2]),
                    scale=np.array([0.05, 0.05, top_z3]), visual_material=leg_mat)

    # A cabinet standing beside the table on the -X side (i.e. "before" the table,
    # toward the rail origin) as a static obstacle for collision testing -- the arm
    # planner must route around it. It is a solid box from the floor up, its top at
    # z=cab_h=1.15 -- above the lowered table top (~1.075) -- so it pokes into the
    # lower workspace. No semantic label on purpose:
    # it stays "background" (seg<=1) so the cameras feed it into the MoveIt
    # collision octomap as environment, not as a graspable object. Depth along X,
    # width matched to the table in Y, placed just off the table's -X edge.
    cab_dx, cab_wy, cab_h = 0.4, 0.35, 1.15
    cab_x = cx - 0.45 - 0.07 - cab_dx / 2 - 0.40  # shifted -0.40 m along X (moved back)
    # Pushed flush against wall_right (own y, decoupled from table1's cy): inner
    # face at -1.15, so cab_cy = -1.15 + cab_wy/2 = -0.975.
    cab_cy = -1.15 + cab_wy / 2
    FixedCuboid(prim_path="/World/cabinet/body", name="cabinet",
                position=np.array([cab_x, cab_cy, cab_h / 2]),
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
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    root = get_assets_root_path()
    if root is None:
        root = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/4.5"
    ycb = f"{root}/Isaac/Props/YCB"
    surf1 = top_z + th / 2
    surf2 = top_z2 + th / 2
    surf3 = top_z3 + th / 2
    # label, usd (relative to ycb), (x, y, surface_z), has_physics_variant
    # table-1 object X shifted +1.35 (was 1.42/1.52/1.70) to follow cx's move
    # to 2.9 -- these are literal coords, NOT derived from cx, so they must be
    # kept in sync with it by hand. Y shifted -0.80 to follow table1's cy move
    # (flush to wall_right): -0.16-0.80=-0.96, 0.16-0.80=-0.64.
    # sugar_box removed (previous request); scissors added in its old slot.
    specs = [
        ("cracker_box",     "Axis_Aligned_Physics/003_cracker_box.usd",     (2.77, -0.96, surf1), True),
        # obj_1: scissors, in sugar_box's old slot (requested). Only the
        # visual-only variant exists (verified via HTTP HEAD; Axis_Aligned_
        # Physics/037_scissors.usd -> 404), so has_phys=False.
        ("scissors",        "Axis_Aligned/037_scissors.usd",                (2.87, -0.64, surf1), False),
        # obj_2 shifted +0.20 x / +0.30 y (requested): (3.05,-0.96) -> (3.25,-0.66).
        ("tomato_soup_can", "Axis_Aligned_Physics/005_tomato_soup_can.usd", (3.25, -0.66, surf1), True),
        # obj_3 seated on TOP of the cabinet (cab_x, cab_cy, cab top = cab_h).
        ("mustard_bottle",  "Axis_Aligned_Physics/006_mustard_bottle.usd",  (cab_x, cab_cy, cab_h), True),
        # obj_4/5 repositioned to fit work_table2's new footprint (x:[1.15,2.45]
        # after the +0.55 then +0.50 advances, y:[0.40,1.15] -- see
        # cx2/cy2/sx2/sy2 above). X shifted +1.05 total to follow table2's cx
        # move; relative layout (from the earlier rotation remap) is unchanged.
        # obj_4: IsaacLab teddy bear (not YCB) -> full URL, resolved directly below.
        ("teddy_bear",      f"{root}/Isaac/IsaacLab/Objects/Teddy_Bear/teddy_bear.usd", (1.45,  0.59, surf2), False),
        # potted_meat_can removed (previous request).
        ("banana",          "Axis_Aligned/011_banana.usd",                  (2.01,  0.60, surf2), False),
        # obj_6: a second mug on work_table3, replacing the earlier glass
        # primitive placeholder with the real Isaac/YCB mug asset (requested).
        # NOTE: the physics variant doesn't exist for this asset (verified via
        # HTTP HEAD against the Nucleus/S3 fallback: Axis_Aligned_Physics/
        # 025_mug.usd -> 404) -- that 404 was why the mug rendered "empty" in
        # the sim. Only Axis_Aligned/025_mug.usd (visual-only, 200 OK) exists,
        # so has_phys=False here (static convex-hull collider path, like banana).
        ("mug",             "Axis_Aligned/025_mug.usd",                     (cx3, cy3, surf3), False),
        # obj_7: bowl on work_table3, near gantry_1's side (gantry_1_base is
        # at world y=+0.36, per CLAUDE.md) -- placed at (cx3, 0.36), a 0.36 m
        # separation from the table3 mug at (cx3, cy3=0.0), clear of the
        # >0.25 m object-spacing convention. Same asset family as the mug:
        # only the visual-only Axis_Aligned variant exists (verified via HTTP
        # HEAD; Axis_Aligned_Physics/024_bowl.usd -> 404), so has_phys=False.
        ("bowl",            "Axis_Aligned/024_bowl.usd",                    (cx3, 0.36, surf3), False),
    ]
    objs = []
    stage = get_current_stage()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    # Cans/bottle ship lying on their side; a +90 deg rotation about X stands them
    # upright (long axis -> +Z). quat is (w, x, y, z), scalar-first. Flip to about-Y
    # [0.7071, 0, 0.7071, 0] if an object ends up lying the other way.
    _stand = np.array([0.70710678, 0.70710678, 0.0, 0.0])       # +90 deg about X
    # mustard bottle came out upside down at +90; -90 (i.e. +180 more) stands it right way up.
    _stand_flip = np.array([0.70710678, -0.70710678, 0.0, 0.0])  # -90 deg about X
    _yaw90 = np.array([0.70710678, 0.0, 0.0, 0.70710678])         # +90 deg about Z (in-plane)
    # obj_0 cracker_box (-90), obj_2 tomato_soup_can (+90),
    # obj_3 mustard_bottle (-90), obj_5 banana (+90 yaw). obj_1 scissors lies
    # flat as-is (no rotation needed). obj_6/7 mug/bowl came out face-down at
    # +90 (requested fix) -> switched to _stand_flip (-90, i.e. +180 more).
    stand_rot = {0: _stand_flip, 2: _stand, 3: _stand_flip, 5: _yaw90, 6: _stand_flip, 7: _stand_flip}
    for i, (label, rel, (ox, oy, surf), has_phys) in enumerate(specs):
        prim_path = f"/World/objects/obj_{i}"
        usd_path = rel if "://" in rel else f"{ycb}/{rel}"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        # Measure the asset's own bounds (still at identity) and seat it BEFORE
        # building the wrapper: bottom exactly on the table top, centered on
        # (ox, oy). These YCB meshes are center-origin, so placing the origin on
        # the surface would bury half the body. The seated pose must be passed to
        # the constructor (not set afterwards) because World.reset() restores each
        # scene object to its construction-time default pose -- a later
        # set_world_pose would be reverted, which is what left them sunk before.
        rng = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        quat = stand_rot.get(i)                 # (w,x,y,z) to stand a can upright, or None
        if rng.IsEmpty():
            pos = np.array([ox, oy, surf])      # fallback: origin on the surface
        else:
            mn, mx = rng.GetMin(), rng.GetMax()
            if quat is not None:
                # Seat against the STOOD-UP extents: rotate the 8 local-bbox
                # corners by the same quat so the bottom (not the side) lands on
                # the table and the body stays centered on (ox, oy).
                w, x, y, z = quat
                R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                              [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                              [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
                corners = np.array([[a, b, c] for a in (mn[0], mx[0])
                                    for b in (mn[1], mx[1])
                                    for c in (mn[2], mx[2])]) @ R.T
                mn, mx = corners.min(axis=0), corners.max(axis=0)
            pos = np.array([ox - 0.5 * (mn[0] + mx[0]),
                            oy - 0.5 * (mn[1] + mx[1]),
                            surf - mn[2]])
        if has_phys:
            # disable the asset's baked rigid-body dynamics -> static collider.
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        else:
            # visual-only mesh -> add a static convex-hull collider on each mesh.
            for d in Usd.PrimRange(prim):
                # The IsaacLab teddy bear ships as a DEFORMABLE (soft) body, which
                # PhysX only supports on GPU ("enable GPU dynamics flag" warning).
                # We want a static collider like every other object, so strip any
                # deformable-body API before adding the convex hull below.
                for s in list(d.GetAppliedSchemas()):
                    if "Deformable" in s:
                        d.RemoveAppliedSchema(s)
                if d.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(d)
                    UsdPhysics.MeshCollisionAPI.Apply(d).CreateApproximationAttr("convexHull")
        obj = SingleXFormPrim(prim_path=prim_path, name=f"obj_{i}",
                              position=pos, orientation=quat)
        objs.append(obj)
        # Semantic class label -> the cameras' instance_segmentation publishes a
        # labeled mask + an id->class JSON, so the localizer can name each
        # detected object. Ground-truth seg now; swap source for open-vocab later.
        add_update_semantics(prim, label)
        if i == 6:
            # obj_6 (table3 mug): override the asset's baked texture with the
            # brightest possible color (pure white) as requested. Bound on the
            # asset root with strongerThanDescendants so it wins over any
            # material binding on the mesh below it (same pattern as
            # recolor_tables).
            bright = _pbr("/World/Looks/MugBright", (1.0, 1.0, 1.0))
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(bright.prim_path)),
                bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    return objs
