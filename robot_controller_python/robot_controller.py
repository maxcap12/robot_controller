from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import PhysxSchema, Gf, UsdGeom
from omni.physx.scripts.physicsUtils import set_or_add_translate_op, set_or_add_orient_op
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep

import rclpy
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
import subprocess
import os
import signal

from .joint_node import JointNode

class RobotController:
    robot_paths = {
        "spot": "/Isaac/Robots/BostonDynamics/spot/spot.usd",
        "go1": "/Isaac/Robots/Unitree/Go1/go1.usd",
        "go2": "/Isaac/Robots/Unitree/Go2/go2.usd",
    }

    def __init__(self):
        self.robot_name = None
        self.articulation = None
        self.lidar = None

        if not rclpy.ok():
            rclpy.init()
            
        self.node = JointNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

        self.mpc_process = None
        self.sgraphs_process = None
        self.use_sgraphs = False
        self.sgraphs_kargs = {}

    def _kill_process(self, process):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass

    def shutdown(self):
        self._kill_process(self.mpc_process)
        self.mpc_process = None
        self._kill_process(self.sgraphs_process)
        self.sgraphs_process = None

    def launch_mpc_node(self):
        self._kill_process(self.mpc_process)

        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        
        self.mpc_process = subprocess.Popen(
            "yes | /opt/conda/bin/conda run --no-capture-output -n venv bash -c "
            "'export LD_LIBRARY_PATH=/ws/src/Quadruped-PyMPC/quadruped_pympc/acados/lib:$LD_LIBRARY_PATH && "
            f"python3 /ws/src/Quadruped-PyMPC/robot_controller.py {self.robot_name}'",
            shell=True,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )

        print("process started with PID:", self.mpc_process.pid)
        print("status:", self.mpc_process.poll())

    def launch_sgraphs(self):
        self._kill_process(self.sgraphs_process)

        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)

        self.sgraphs_process = subprocess.Popen(
            [
                "bash -c "
                "'source /ros_ws/install/setup.bash && "
                "ros2 launch lidar_situational_graphs s_graphs_launch.py "
                f"{' '.join([f'{key}:={value}' for key, value in self.sgraphs_kargs.items()])}'"
            ],
            shell=True,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )

    def create_graph(self, base_prim_path, lidar_prim_path, lidar_frame, pointcloud_topic):
        sensor_prim_path = f"{lidar_prim_path}/sensor"

        self._lidar_render_product = rep.create.render_product(
            sensor_prim_path,
            resolution=(1, 1),
            render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        )

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": f"{lidar_prim_path}/pc_publisher", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Ros2Helper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ("TfPublisher", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("Ros2Helper.inputs:renderProductPath", self._lidar_render_product.path),
                    ("Ros2Helper.inputs:topicName", pointcloud_topic),
                    ("Ros2Helper.inputs:frameId", lidar_frame),
                    ("Ros2Helper.inputs:type", "point_cloud"),
                    ("Ros2Helper.inputs:nodeNamespace", ""),
                    ("Ros2Helper.inputs:fullScan", True),
                    ("TfPublisher.inputs:topicName", "/tf"),
                    ("TfPublisher.inputs:nodeNamespace", ""),
                    ("TfPublisher.inputs:parentPrim", [base_prim_path]),
                    ("TfPublisher.inputs:targetPrims", [lidar_prim_path]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Ros2Helper.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "TfPublisher.inputs:execIn"),
                    ("SimTime.outputs:simulationTime", "TfPublisher.inputs:timeStamp"),
                ],
            }
        )

    def load_robot(self, robot, use_sgraphs, pos, ori, sgraphs_kargs={},
                   base_frame="base_link", lidar_frame="lidar", pointcloud_topic="/sim/point_cloud",
                   publish_lidar=False):
        self.use_sgraphs = use_sgraphs
        self.sgraphs_kargs = sgraphs_kargs
        stage = omni.usd.get_context().get_stage()
        
        if self.robot_name != robot:
            stage.RemovePrim(f"/{self.robot_name}")
            
            self.robot_name = robot
            robot_prim_path = f"/{robot}"
            path_to_robot_usd = get_assets_root_path() + self.robot_paths[robot]

            add_reference_to_stage(path_to_robot_usd, robot_prim_path)

            prim = stage.GetPrimAtPath(robot_prim_path)

            xformable = UsdGeom.Xformable(prim)
            set_or_add_translate_op(xformable, Gf.Vec3f(pos[0], pos[1], pos[2]))
            set_or_add_orient_op(xformable, Gf.Quatf(ori[3], ori[0], ori[1], ori[2]))

            physx_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            physx_api.GetSolverPositionIterationCountAttr().Set(32)
            physx_api.GetSolverVelocityIterationCountAttr().Set(1)

            self.articulation = Articulation(robot_prim_path)

        if use_sgraphs or publish_lidar:
            lidar_path = get_assets_root_path() + "/Isaac/Sensors/Ouster/OS1/OS1.usd"
            base_prim_path = f"/{self.robot_name}/body/{base_frame}"
            lidar_prim_path = f"{base_prim_path}/{lidar_frame}"

            UsdGeom.Xform.Define(stage, base_prim_path)
            add_reference_to_stage(lidar_path, lidar_prim_path)
            self.lidar = stage.GetPrimAtPath(lidar_prim_path)

            xform = UsdGeom.Xformable(self.lidar)

            if robot == "spot":
                xform.GetTranslateOp().Set(Gf.Vec3d(-0.11036, 0.0, 0.08087))
                xform.GetRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

            self.create_graph(base_prim_path, lidar_prim_path, lidar_frame, pointcloud_topic)

        if use_sgraphs:
            self.sgraphs_kargs["lidar_topic"] = pointcloud_topic
            self.sgraphs_kargs["base_frame"] = base_frame
            self.launch_sgraphs()

        self.launch_mpc_node()
        return self.articulation

    def update(self, step):
        if self.mpc_process is None or self.mpc_process.poll() is not None:
            print("Restarting MPC node...", self.mpc_process, self.mpc_process.poll())
            
            out, err = self.mpc_process.communicate()
            print("STDOUT:", out)
            print("STDERR:", err)
            print("Return code:", self.mpc_process.returncode)

            self.launch_mpc_node()

        angles, velocities = self.node.get_joint_attributes()

        if self.use_sgraphs and (self.sgraphs_process is None or self.sgraphs_process.poll() is not None):
            print("Restarting S-Graphs node...", self.sgraphs_process, self.sgraphs_process.poll())

            out, err = self.sgraphs_process.communicate()
            print("STDOUT:", out)
            print("STDERR:", err)
            print("Return code:", self.sgraphs_process.returncode)

            self.launch_sgraphs()
            
        if angles is None or velocities is None: 
            print("aborting update, no joint data yet")
            return
        
        if not self.articulation.handles_initialized:
            print("Initializing articulation...")
            self.articulation.initialize()
        
        action = ArticulationAction(
            joint_positions=angles,
        )
        
        self.articulation.apply_action(action)

    def reset(self):
        self.launch_mpc_node()
        
        if self.use_sgraphs:
            self.launch_sgraphs()

    def play(self):
        self.node.play_pause()

    def pause(self):
        self.node.play_pause()
