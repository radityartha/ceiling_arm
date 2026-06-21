"""Scene-polish helpers for the workcell viewer: a room (floor + walls + lights),
a work surface with objects, and grey materials on the otherwise-uncolored tables.
The arm links already carry colors from the URDF, so only the 8 table links need it.
"""
import numpy as np

from isaacsim.core.api.objects import FixedCuboid, DynamicCuboid
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.utils.stage import get_current_stage

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
    """Floor + 3 walls (front open for camera) + a work table with a few objects."""
    floor_mat = _pbr("/World/Looks/Floor", (0.18, 0.19, 0.21))
    wall_mat = _pbr("/World/Looks/Wall", (0.55, 0.56, 0.60))
    top_mat = _pbr("/World/Looks/TableTop", (0.45, 0.30, 0.18))
    leg_mat = _pbr("/World/Looks/Leg", (0.12, 0.12, 0.13))

    FixedCuboid(prim_path="/World/room/floor", name="floor",
                position=np.array([0, 0, -0.05]), scale=np.array([12, 12, 0.1]),
                visual_material=floor_mat)
    # back wall (x=-2.5) and two side walls (y=+-2.5); front left open
    FixedCuboid(prim_path="/World/room/wall_back", name="wall_back",
                position=np.array([-2.5, 0, 2.0]), scale=np.array([0.1, 6, 4.0]),
                visual_material=wall_mat)
    FixedCuboid(prim_path="/World/room/wall_left", name="wall_left",
                position=np.array([0, 2.5, 2.0]), scale=np.array([6, 0.1, 4.0]),
                visual_material=wall_mat)
    FixedCuboid(prim_path="/World/room/wall_right", name="wall_right",
                position=np.array([0, -2.5, 2.0]), scale=np.array([6, 0.1, 4.0]),
                visual_material=wall_mat)

    # work table under the arms (top ~z=0.78)
    cx, cy, top_z, th = 0.45, 0.0, 0.78, 0.05
    FixedCuboid(prim_path="/World/work_table/top", name="wt_top",
                position=np.array([cx, cy, top_z]), scale=np.array([1.3, 1.3, th]),
                visual_material=top_mat)
    for i, (dx, dy) in enumerate([(0.58, 0.58), (0.58, -0.58), (-0.58, 0.58), (-0.58, -0.58)]):
        FixedCuboid(prim_path=f"/World/work_table/leg_{i}", name=f"wt_leg_{i}",
                    position=np.array([cx + dx, cy + dy, top_z / 2]),
                    scale=np.array([0.05, 0.05, top_z]), visual_material=leg_mat)

    # a few objects resting on the table
    obj_z = top_z + th / 2 + 0.025
    colors = [(0.85, 0.15, 0.15), (0.15, 0.7, 0.2), (0.15, 0.3, 0.85)]
    spots = [(0.30, 0.25), (0.55, -0.2), (0.2, -0.3)]
    objs = []
    for i, ((ox, oy), c) in enumerate(zip(spots, colors)):
        objs.append(DynamicCuboid(prim_path=f"/World/objects/obj_{i}", name=f"obj_{i}",
                    position=np.array([cx + ox - 0.45, cy + oy, obj_z]),
                    size=0.05, color=np.array(c), mass=0.05))
    return objs
