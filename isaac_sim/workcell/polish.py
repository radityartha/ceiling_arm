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
    # Two side walls, now 2.4 m apart: wall_left at y=+1.2, wall_right at y=-1.2
    # (inner faces at +-1.15 given the 0.1 m thickness). The back (-x) and front
    # (+x) walls are OMITTED so nothing occludes the two RGBD cameras, which sit
    # near the rail ends (rgbd2 at x=-0.6, rgbd at x=4.35) looking in under the
    # arms. X length stays 10 (span -5..5) so the far object (tomato_soup_can,
    # x=3.05) and the rgbd camera (x=4.35) stay inside the wall span (no light
    # leak / glare on the unshaded objects).
    FixedCuboid(prim_path="/World/room/wall_left", name="wall_left",
                position=np.array([0, 1.2, 2.0]), scale=np.array([10, 0.1, 4.0]),
                visual_material=wall_mat)
    FixedCuboid(prim_path="/World/room/wall_right", name="wall_right",
                position=np.array([0, -1.2, 2.0]), scale=np.array([10, 0.1, 4.0]),
                visual_material=wall_mat)

    # Work table under the ceiling arms. The arms hang from z~2.05 and their
    # reachable GNG cloud floors out at z~1.22. top_z=1.05 puts the top ~0.17 m
    # below that floor, but the 0.727 m pool radius (reachability heuristic)
    # bridges the gap, so objects at z~1.10 still find candidate nodes and stay
    # reachable. Legs + obj_z derive from top_z. Footprint sized to the object
    # cluster with a small margin. Rail travels world +X 0..3.0 m, EE reach
    # ~3.6 m, so this stays inside the gantry's X range.
    # Center at x=2.85 so the table's +X edge (cx+0.45=3.30) slides under obj_2
    # (tomato_soup_can at x=3.25), the unreachable-object case shoved past the reach
    # fringe -- the table supports it there instead of it floating. obj_0/obj_1/obj_2
    # X specs below are carried by the same -0.1 shift so they stay on the table;
    # the cabinet (cab_x, derived from cx) rides along too, keeping it just off the
    # table's -X edge.
    # cy pushed to -0.80 so the table sits flush against wall_right: its -Y edge
    # (cy - 0.7/2 = -1.15) touches the wall inner face. Table-1 object Y specs
    # below are shifted by the same -0.80 (they are literal, not derived from cy).
    cx, cy, top_z, th = 2.85, -0.80, 1.05, 0.05
    FixedCuboid(prim_path="/World/work_table/top", name="wt_top",
                position=np.array([cx, cy, top_z]), scale=np.array([0.9, 0.7, th]),
                visual_material=top_mat)
    for i, (dx, dy) in enumerate([(0.40, 0.30), (0.40, -0.30), (-0.40, 0.30), (-0.40, -0.30)]):
        FixedCuboid(prim_path=f"/World/work_table/leg_{i}", name=f"wt_leg_{i}",
                    position=np.array([cx + dx, cy + dy, top_z / 2]),
                    scale=np.array([0.05, 0.05, top_z]), visual_material=leg_mat)

    # Second work table on the +Y side ("left" of the arms), slightly lower than
    # table 1 (top_z2 0.97 vs 1.05). Rotated 90 deg vs before so it now runs LONG
    # along X (sx2=1.30, sy2=0.75, was 0.75x1.30) and sits flush against wall_left:
    # its +Y edge (cy2 + 0.75/2 = 1.15) touches the wall inner face. Centered at
    # cx2=1.6 so it spans x 0.95..2.25, inside the gantry X range. Its 2 objects
    # (teddy_bear, mug) are lined up along X at y=cy2 (see specs below). NOTE: the old per-object
    # reachable/unreachable design (banana reachable, potted_meat_can out of reach
    # at y=1.60) no longer holds under this layout and would need re-tuning.
    cx2, cy2, top_z2, sx2, sy2 = 1.6, 0.775, 0.97, 1.30, 0.75
    FixedCuboid(prim_path="/World/work_table2/top", name="wt2_top",
                position=np.array([cx2, cy2, top_z2]), scale=np.array([sx2, sy2, th]),
                visual_material=top_mat)
    lx, ly = sx2 / 2 - 0.05, sy2 / 2 - 0.05
    for i, (dx, dy) in enumerate([(lx, ly), (lx, -ly), (-lx, ly), (-lx, -ly)]):
        FixedCuboid(prim_path=f"/World/work_table2/leg_{i}", name=f"wt2_leg_{i}",
                    position=np.array([cx2 + dx, cy2 + dy, top_z2 / 2]),
                    scale=np.array([0.05, 0.05, top_z2]), visual_material=leg_mat)

    # Third work table spanning the full corridor WIDTH (wall to wall) near the
    # gantry START point (rail x=0), nudged to cx3=0.2. sy3=2.3 makes its Y edges
    # touch both wall inner faces (+-1.15); sx3=0.9 deep in X, top matched to
    # table 1 (1.05). Holds the banana (moved off table 2) + a beaker (see specs).
    cx3, cy3, top_z3, sx3, sy3 = 0.2, 0.0, 1.05, 0.9, 2.3
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
    # placed just off the table's -X edge. cab_cy=-0.975 sets it flush against
    # wall_right too (its -Y edge cab_cy - 0.35/2 = -1.15 touches the wall).
    cab_dx, cab_wy, cab_h = 0.4, 0.35, 1.15
    cab_x = cx - 0.45 - 0.07 - cab_dx / 2 - 0.40  # shifted -0.40 m along X (moved back)
    cab_cy = -0.975
    FixedCuboid(prim_path="/World/cabinet/body", name="cabinet",
                position=np.array([cab_x, cab_cy, cab_h / 2]),
                scale=np.array([cab_dx, cab_wy, cab_h]), visual_material=cab_mat)

    # Real YCB objects (cans / bottle / boxes / banana) instead of primitives, so
    # segmentation + localization + reachability run against realistic geometry.
    # Two asset variants are mixed: 4 ship as *physics* USDs (RigidBodyAPI +
    # ConvexHull collision baked in), the other 4 are visual-only meshes. We turn
    # ALL of them into STATIC colliders -- they never fall, never move, and carry
    # no rigid-body dynamics, so World.reset() never tries to zero a velocity on
    # them (that was the "PxRigidDynamic::setLinearVelocity: Body must be
    # non-kinematic" spam). Static is also what the perception/reachability task
    # wants: fixed objects at known poses. Split: physics objects on table 1
    # (surf1), visual-only on tables 2 & 3 (surf2/surf3). (x, y) are spaced >0.25 m
    # apart so no two objects touch.
    from pxr import Usd, UsdGeom, UsdPhysics
    root = get_assets_root_path()
    if root is None:
        root = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/4.5"
    ycb = f"{root}/Isaac/Props/YCB"
    surf1 = top_z + th / 2
    surf2 = top_z2 + th / 2
    surf3 = top_z3 + th / 2
    # label, usd (relative to ycb), (x, y, surface_z), has_physics_variant
    # table-1 objects sit flush-side with the table: their Y was shifted by -0.80
    # (to follow cy) so they land near y=-0.96/-0.64, inside the table's -1.15..-0.45
    # span. These are literal coords, NOT derived from cx/cy, so keep them in sync
    # with the table by hand.
    specs = [
        ("cracker_box",     "Axis_Aligned_Physics/003_cracker_box.usd",     (2.72, -0.96, surf1), True),
        ("sugar_box",       "Axis_Aligned_Physics/004_sugar_box.usd",       (2.82, -0.64, surf1), True),
        # obj_2 deliberately shoved to x=3.25 -- past the ~3.06 m reach-map fringe of
        # gantry_1, so it sits OUTSIDE the arm capability map (unreachable-object
        # test case). Table 1 (cx=2.85) reaches it: its +X edge (3.30) supports obj_2
        # rather than leaving it floating.
        ("tomato_soup_can", "Axis_Aligned_Physics/005_tomato_soup_can.usd", (3.25, -0.66, surf1), True),
        # obj_3 seated on TOP of the cabinet (cab_x, cab_cy, cab top = cab_h).
        ("mustard_bottle",  "Axis_Aligned_Physics/006_mustard_bottle.usd",  (cab_x, cab_cy, cab_h), True),
        # obj_4/5 lined up along X on the rotated table 2 at y=cy2 (0.775), spaced
        # ~0.9 m apart, inside its x 0.95..2.25 span.
        # obj_4: IsaacLab teddy bear (not YCB) -> full URL, resolved directly below.
        ("teddy_bear",      f"{root}/Isaac/IsaacLab/Objects/Teddy_Bear/teddy_bear.usd", (1.15, 0.775, surf2), False),
        # obj_5: Isaac Props mug (not YCB) -> full URL. Ships upright (Z up), so it
        # needs NO stand rotation (see stand_rot below).
        ("mug",             f"{root}/Isaac/Props/Mugs/SM_Mug_A2.usd",        (2.05, 0.775, surf2), False),
        # obj_6/7 on table 3 (surf3), spread in Y near the two gantries at x=cx3=0.2.
        ("banana",          "Axis_Aligned/011_banana.usd",                  (0.2, 0.4, surf3), False),
        # obj_7: Isaac Props 500 ml beaker (not YCB) -> full URL. Ships upright (Z up).
        ("beaker",          f"{root}/Isaac/Props/Beaker/beaker_500ml.usd",  (0.2, -0.4, surf3), False),
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
    # obj_0/obj_1 boxes (-90), obj_2 tomato_soup_can (+90),
    # obj_3 mustard_bottle (-90), obj_6 banana (+90 yaw). obj_5 (mug), obj_7 (beaker)
    # ship upright -> no entry.
    stand_rot = {0: _stand_flip, 1: _stand_flip, 2: _stand, 3: _stand_flip, 6: _yaw90}
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

    # A standing human beside the workcell as scene context / static obstacle. It
    # gets NO semantic label (like the cabinet), so it stays background: the cameras
    # feed it into the MoveIt collision octomap as environment, never as a graspable
    # object. NVIDIA's People characters are authored Z-up with the origin at the
    # feet, so z=0 puts the soles on the floor -- no bbox-seating needed like the
    # center-origin YCB meshes. Note the asset naming quirk: the folder carries an
    # "original_" prefix that the .usd filename does NOT. Placed near wall_left
    # (y=1.0, back to the +Y wall) at x=0.5, clear of rotated table 2 (x 0.95..2.25).
    #
    # Every People/DH character in the Isaac library ships in a T-pose (arms out) --
    # that is the skeleton's rest pose, and there is no static natural-pose human
    # asset. A relaxed idle normally needs the omni.anim.people retarget +
    # AnimationGraph (heavyweight, interactive; the character's 101-joint Reallusion
    # skeleton does not even match the 81-joint Isaac animation clips). Instead we
    # drop the arms into a static A-pose with a tiny 2-joint UsdSkel animation that
    # rotates only the shoulders (L/R_Upperarm) from horizontal to ~68 deg down and
    # slightly outward. The local rotations below were computed offline from this
    # rig's bind transforms (verified: arm dir (+-1,0,0) -> (+-0.38,0.01,-0.93)) and
    # are reused across every People character since they share the RL skeleton. To
    # raise/lower the arms, regenerate the two quats at a different down-angle.
    from pxr import UsdSkel, Gf, Vt
    person_char = "original_male_adult_construction_05/male_adult_construction_05.usd"
    person_usd = f"{root}/Isaac/People/Characters/{person_char}"
    person_path = "/World/person/body"
    add_reference_to_stage(usd_path=person_usd, prim_path=person_path)
    person_prim = stage.GetPrimAtPath(person_path)
    L_ARM = "RL_BoneRoot/Hip/Waist/Spine01/Spine02/L_Clavicle/L_Upperarm"
    R_ARM = "RL_BoneRoot/Hip/Waist/Spine01/Spine02/R_Clavicle/R_Upperarm"
    skel = next((p for p in Usd.PrimRange(person_prim)
                 if p.GetTypeName() == "Skeleton"), None)
    sj = list(UsdSkel.Skeleton(skel).GetJointsAttr().Get() or []) if skel else []
    if L_ARM in sj and R_ARM in sj:
        # Sparse animation: only the two shoulders move; UsdSkel keeps every other
        # joint at its rest transform. Translations are the rest bone offsets (only
        # rotation changes). quat is (w, x, y, z), scalar-first.
        anim = UsdSkel.Animation.Define(stage, f"{person_path}/ArmsDown")
        anim.CreateJointsAttr(Vt.TokenArray([L_ARM, R_ARM]))
        anim.CreateTranslationsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.14946, 0.0),
                                                   Gf.Vec3f(0.0, 0.14953, 0.0)]))
        anim.CreateRotationsAttr(Vt.QuatfArray([
            Gf.Quatf(0.826161, 0.121839, 0.083670, -0.543703),    # L_Upperarm
            Gf.Quatf(0.825552, 0.121207, -0.083080, 0.544859)]))  # R_Upperarm
        anim.CreateScalesAttr(Vt.Vec3hArray([Gf.Vec3h(1, 1, 1), Gf.Vec3h(1, 1, 1)]))
        UsdSkel.BindingAPI.Apply(skel).CreateAnimationSourceRel().SetTargets(
            [anim.GetPrim().GetPath()])
    # The character's own forward (foot->toe) is -Y. A -90 deg rotation about Z
    # swings that forward from -Y to -X, so the person faces "backward" toward the
    # rail origin (-X). The person is scene geometry; MoveIt sees it, if at all,
    # through the camera->octomap path, like the cabinet. quat is (w,x,y,z),
    # scalar-first.
    SingleXFormPrim(prim_path=person_path, name="person",
                    position=np.array([3.7, 0.0, 0.0]),
                    orientation=np.array([0.70710678, 0.0, 0.0, -0.70710678]))
    return objs
