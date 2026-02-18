import ifcopenshell
import ifcopenshell.geom
import omni.usd
from pxr import UsdGeom, Gf, Sdf
from omni.physx.scripts import physicsUtils

def load_map(map_path):    
    model = ifcopenshell.open(map_path)
    
    for wall in model.by_type("IfcWall"):
        create_wall(wall)

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
    
