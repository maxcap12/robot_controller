## This is an extension for isaac sim 5.1 to control quadruped robot using ros2 Twist messages

## GUI usage

#### 1. Enable extension

window -> extensions -> robot_controller

#### 2. Open extension menu 

#### 3. Configuration

- select desired robot (only works for spot for now)
- enable sgraphs if desired
- select a map if desired

#### 4. Click add to load the scene

#### 5. Use the run, pause and reset button from the extension, not the application ones

## Script usage

#### 1. Initialize the app
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})
```

#### 2. Enable the necessary extensions
```python
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.examples.extension")
```

#### 3. Import the headless module
```python
EXTENSION_ROOT = "/isaac-sim/exts/robot_controller"
if EXTENSION_ROOT not in sys.path:
    sys.path.insert(0, EXTENSION_ROOT)
from robot_controller_python.headless import HeadlessRunner
```

#### 4. Initialize extension
```python
runner = HeadlessRunner(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
```

#### 5. Setup the scene with desired config
```py
runner.setup_scene(robot_name="spot", use_sgraphs=True, map_path=None, sgraphs_kargs={"lidar_topic": "/sim/point_cloud", "compute_odom": "True"})
```

#### 6. Optional: add a ground, light and correct vertical position of the robot
```py
runner.add_ground()
runner.add_light()
runner.correct_robot_position()
```

#### 7. Start simulation
```py
runner.run(300, True, 60) (duration in s, headless mode, fps limit)
```

### Notes

To create your own map, refer to this example

When enabling sgraphs, the point clouds are published to /sim/point_cloud by default

When this extension is used for the first time it takes a bit of time to initialize