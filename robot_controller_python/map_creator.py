import yaml
import numpy as np
import ifcopenshell
import ifcopenshell.geom
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, Sdf
from omni.physx.scripts import physicsUtils




def load_map(map_path):
    if map_path is None:
        return (0., 0., 0.), (0., 0., 0., 1.)

    with open(map_path, "r") as f:
        data = yaml.safe_load(f)

    robot_pos = tuple(data["robot"]["position"])
    robot_ori = tuple(data["robot"]["orientation"])

    stage = omni.usd.get_context().get_stage()
    terrain = data.get("terrain", {})
    wall_thickness = terrain.get("wall_thickness", 0.2)
    floor_thickness = terrain.get("floor_thickness", 0.1)


    for i, wall_data in enumerate(terrain.get("walls", [])):
        _create_wall_segment(stage, wall_data["start"], wall_data["end"], i, wall_thickness)

    for i, floor_data in enumerate(terrain.get("floors", [])):
        _create_floor_segment(stage, floor_data["start"], floor_data["end"], i, floor_thickness)

    return robot_pos, robot_ori


def _create_wall_segment(stage, start, end, index, thickness):
    s = np.array(start, dtype=float)
    e = np.array(end, dtype=float)

    # Horizontal extent in XY plane; Z encodes bottom (start) and top (end)
    horiz = e[:2] - s[:2]
    length = float(np.linalg.norm(horiz))
    if length < 1e-6:
        return

    unit = horiz / length
    height = float(e[2] - s[2])

    # Extend each end by thickness/2 along the wall direction so corners are
    # fully filled when two perpendicular walls meet.
    extension = np.array([unit[0], unit[1], 0.0]) * (thickness / 2.0)
    s = s - extension
    e = e + extension
    length += float(thickness)

    mid = (s + e) / 2.0

    # Rotate around Z axis to align the cube's X axis with the wall direction
    angle_z = float(np.degrees(np.arctan2(unit[1], unit[0])))

    path = Sdf.Path(f"/World/Wall_{index}")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    xform_api = UsdGeom.XformCommonAPI(cube)
    xform_api.SetTranslate(Gf.Vec3d(float(mid[0]), float(mid[1]), float(mid[2])))
    xform_api.SetRotate(Gf.Vec3f(0.0, 0.0, angle_z), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    xform_api.SetScale(Gf.Vec3f(length, float(thickness), height))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _create_floor_segment(stage, start, end, index, thickness):
    s = np.array(start, dtype=float)
    e = np.array(end, dtype=float)

    mid_x = float((s[0] + e[0]) / 2.0)
    mid_y = float((s[1] + e[1]) / 2.0)
    size_x = float(abs(e[0] - s[0]))
    size_y = float(abs(e[1] - s[1]))

    path = Sdf.Path(f"/World/Floor_{index}")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    xform_api = UsdGeom.XformCommonAPI(cube)
    xform_api.SetTranslate(Gf.Vec3d(mid_x, mid_y, float(s[2]) - float(thickness) / 2.0))
    xform_api.SetScale(Gf.Vec3f(size_x, size_y, float(thickness)))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def create_wall(wall):
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    shape = ifcopenshell.geom.create_shape(settings, wall)

    verts = shape.geometry.verts
    faces = shape.geometry.faces

    points = [(verts[i], verts[i+1], verts[i+2])
              for i in range(0, len(verts), 3)]

    stage = omni.usd.get_context().get_stage()
    path = f"/World/Wall_{wall.id()}"

    mesh = UsdGeom.Mesh.Define(stage, path)

    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateFaceVertexCountsAttr([3] * (len(faces) // 3))

    physicsUtils.add_collision(mesh.GetPrim())
