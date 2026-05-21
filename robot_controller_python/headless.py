from isaacsim.core.api.world import World
from .robot_controller import RobotController
from .map_creator import load_map


class ExperimentRunner:
    def __init__(self, physics_dt=1/60., rendering_dt=1/60.):
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
        )
        self.controller = RobotController()
        self.robot = None

    def setup_scene(self, robot_name, use_sgraphs=False, map_path=None):
        pos, ori = load_map(map_path)
        self.robot = self.controller.load_robot(robot_name, use_sgraphs, pos, ori)
        self.world.scene.add(self.robot)
        self.world.reset()

    def step(self, render=True):
        self.controller.update(self.world.get_physics_dt())
        self.world.step(render=render)

    def run(self, duration_s, render=True):
        n = int(duration_s / self.world.get_physics_dt())
        self.controller.play()
      
        for _ in range(n):
            self.step(render=render)
          
        self.controller.pause()

    def shutdown(self):
        self.controller.shutdown()
