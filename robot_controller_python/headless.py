from isaacsim.core.api.world import World
from pxr import UsdLux, UsdGeom, Sdf, Gf
from omni.physx import get_physx_scene_query_interface
import omni.usd
import omni.kit.app
import carb
from .robot_controller import RobotController
from .map_creator import load_map
import time


class HeadlessRunner:
    SPAWN_CLEARANCE = {"spot": 0.6, "go1": 0.4, "go2": 0.4}
    DEFAULT_CLEARANCE = 0.4
    
    def __init__(self, physics_dt=1/60., rendering_dt=1/60.):
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
        )
        self.controller = RobotController()
        self.robot = None

    def setup_scene(self, robot_name, use_sgraphs=False, map_path=None, sgraphs_kargs={},
                    base_frame="base_link", lidar_frame="lidar", pointcloud_topic="/sim/point_cloud",
                    publish_lidar=False):
        pos, ori = load_map(map_path)
        self.robot = self.controller.load_robot(
            robot_name, use_sgraphs, pos, ori, sgraphs_kargs,
            base_frame=base_frame, lidar_frame=lidar_frame, pointcloud_topic=pointcloud_topic,
            publish_lidar=publish_lidar,
        )
        self.world.scene.add(self.robot)
        self.world.reset()

    def step(self, render=True):
        self.controller.update(self.world.get_physics_dt())
        self.world.step(render=render)

    def add_ground(self):
        self.world.scene.add_default_ground_plane()

    def add_light(self):
        stage = omni.usd.get_context().get_stage()
        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
        light.CreateIntensityAttr(3000.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

    def correct_robot_position(self, clearance=None, start_offset=10.0, max_distance=50.0):
        if self.robot is None: return

        robot_name = self.controller.robot_name
        position, orientation = self.robot.get_world_pose()
        x, y, z = float(position[0]), float(position[1]), float(position[2])

        surface_z = self._surface_below(
            x, y, z, ignore_prefix=f"/{robot_name}",
            start_offset=start_offset, max_distance=max_distance
        )

        if surface_z is None: return

        if clearance is None:
            clearance = self.SPAWN_CLEARANCE.get(robot_name, self.DEFAULT_CLEARANCE)

        new_pos = [x, y, surface_z + clearance]
        self.robot.set_world_pose(position=new_pos, orientation=orientation)
        self.robot.set_default_state(position=new_pos, orientation=orientation)

    def _surface_below(self, x, y, z, ignore_prefix=None, start_offset=10.0, max_distance=50.0):
        origin = carb.Float3(float(x), float(y), float(z + start_offset))
        direction = carb.Float3(0.0, 0.0, -1.0)

        best = {"z": None, "dist": None}

        def report(hit):
            if ignore_prefix and hit.collision.startswith(ignore_prefix):
                return True
            if best["dist"] is None or hit.distance < best["dist"]:
                best["dist"] = hit.distance
                best["z"] = float(hit.position[2])

            return True

        get_physx_scene_query_interface().raycast_all(
            origin, direction, float(max_distance), report
        )
        return best["z"]

    def run(self, duration_s, render=True, fps_limit=None):
        n = int(duration_s / self.world.get_physics_dt())
        self.controller.play()

        last_frame = time.time()
        
        for _ in range(n):
            self.step(render=render)

            if fps_limit is not None:
                time.sleep(max(0, (1 / fps_limit) - (time.time() - last_frame)))
                last_frame = time.time()
                
        self.controller.pause()

    def shutdown(self):
        self.controller.shutdown()
