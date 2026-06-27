"""Scene-polish helpers for the workcell viewer: a room (floor + walls + lights),
a work surface with objects, and grey materials on the otherwise-uncolored tables.
The arm links already carry colors from the URDF, so only the 8 table links need it.
"""
import numpy as np

from isaacsim.core.api.objects import (FixedCuboid, DynamicCuboid,
                                        DynamicCylinder, DynamicSphere)
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.utils.stage import get_current_stage
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
    cx, cy, top_z, th = 1.5, 0.0, 1.30, 0.05
    FixedCuboid(prim_path="/World/work_table/top", name="wt_top",
                position=np.array([cx, cy, top_z]), scale=np.array([1.3, 1.3, th]),
                visual_material=top_mat)
    for i, (dx, dy) in enumerate([(0.58, 0.58), (0.58, -0.58), (-0.58, 0.58), (-0.58, -0.58)]):
        FixedCuboid(prim_path=f"/World/work_table/leg_{i}", name=f"wt_leg_{i}",
                    position=np.array([cx + dx, cy + dy, top_z / 2]),
                    scale=np.array([0.05, 0.05, top_z]), visual_material=leg_mat)

    # A mix of object shapes resting on the table (cube / can / bottle / ball)
    # to test that segmentation + localization + reachability handle arbitrary
    # geometry, not just cubes. (x, y) are world coords; each object is placed so
    # it RESTS on the table top (surface_z + half its height). All spots kept
    # clear (>0.3 m) of the arm hang columns (~x=+-0.4, y=0.36) so the penetrating
    # home-pose arm doesn't knock one off, and within the table footprint.
    #   kind: cube=size | cylinder=(radius,height) | sphere=radius
    surface_z = top_z + th / 2
    specs = [
        ("red_box",   "cube",     (1.35, -0.10), (0.85, 0.15, 0.15), 0.05),
        ("green_box", "cube",     (1.60, -0.20), (0.15, 0.70, 0.20), 0.05),
        ("blue_box",  "cube",     (1.25, -0.30), (0.15, 0.30, 0.85), 0.05),
        ("can",       "cylinder", (1.47,  0.02), (0.80, 0.55, 0.10), (0.033, 0.12)),
        ("bottle",    "cylinder", (1.67, -0.05), (0.10, 0.55, 0.65), (0.035, 0.22)),
        ("ball",      "sphere",   (1.19, -0.06), (0.85, 0.75, 0.15), 0.035),
    ]
    objs = []
    stage = get_current_stage()
    for i, (label, kind, (ox, oy), col, dims) in enumerate(specs):
        prim_path = f"/World/objects/obj_{i}"
        color = np.array(col)
        if kind == "cube":
            half = dims / 2
            obj = DynamicCuboid(prim_path=prim_path, name=f"obj_{i}",
                                position=np.array([ox, oy, surface_z + half]),
                                size=dims, color=color, mass=0.05)
        elif kind == "cylinder":
            radius, height = dims
            obj = DynamicCylinder(prim_path=prim_path, name=f"obj_{i}",
                                  position=np.array([ox, oy, surface_z + height / 2]),
                                  radius=radius, height=height, color=color, mass=0.05)
        elif kind == "sphere":
            obj = DynamicSphere(prim_path=prim_path, name=f"obj_{i}",
                                position=np.array([ox, oy, surface_z + dims]),
                                radius=dims, color=color, mass=0.05)
        else:
            raise ValueError(kind)
        objs.append(obj)
        # Semantic class label -> the cameras' instance_segmentation publishes a
        # labeled mask + an id->class JSON, so the localizer can name each
        # detected object. Ground-truth seg now; swap source for open-vocab later.
        add_update_semantics(stage.GetPrimAtPath(prim_path), label)
    return objs
