from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import PhysxSchema, Gf, UsdGeom
import omni.usd
import omni.graph.core as og

import rclpy
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

        rclpy.init()
        self.node = JointNode()
        self.thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

        self.mpc_process = None
        self.sgraphs_process = None
        self.use_sgraphs = False

    def launch_mpc_node(self):
        if self.mpc_process is not None:
            if self.mpc_process.poll() is None:
                print("problem", self.mpc_process.poll())
        
            os.killpg(os.getpgid(self.mpc_process.pid), signal.SIGTERM)

        self.mpc_process = subprocess.Popen(
            [
                "conda", "run", "-n", "quadruped_pympc_ros2_jazzy_env",
                "python3", "/Quadruped-PyMPC/robot_controller.py",
                self.robot_name
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )

        print("process started with PID:", self.mpc_process.pid)
        print("status:", self.mpc_process.poll())

    def launch_sgraphs(self):
        if self.sgraphs_process is not None:
            os.killpg(os.getpgid(self.sgraphs_process.pid), signal.SIGTERM)
                        
        self.sgraphs_process = subprocess.Popen(
            [
                "conda", "run", "-n", "quadruped_pympc_ros2_jazzy_env",
                "bash", "-c",
                "source /s_graphs/install/setup.bash && "
                "ros2 launch lidar_situational_graphs s_graphs_launch.py "
                "compute_odom:=true lidar_topic:=/sim/point_cloud"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )

    def create_graph(self, lidar_prim_path):
        keys = og.Controller.Keys

        graph_handle, node_list, _, _ = og.Controller.edit(
            {"graph_path": f"{lidar_prim_path}/pc_publisher", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("Ros2Helper", "isaacsim.ros2.bridge.ROS2RTXLidarHelper")
                ],

                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "Ros2Helper.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "Ros2Helper.inputs:renderProductPath"),
                ],

                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", f"{lidar_prim_path}/sensor"),
                    ("Ros2Helper.inputs:topicName", "/sim/point_cloud"),
                    ("Ros2Helper.inputs:frameId", "map"),
                    ("Ros2Helper.inputs:type", "point_cloud"),
                    ("Ros2Helper.inputs:nodeNamespace", ""),
                    ("Ros2Helper.inputs:fullScan", True),
                ]
            }
        )

    def load_robot(self, robot, use_sgraphs):
        self.use_sgraphs = use_sgraphs
        
        if self.robot_name != robot:
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim(f"/{self.robot_name}")
            
            self.robot_name = robot
            robot_prim_path = f"/{robot}"
            path_to_robot_usd = get_assets_root_path() + self.robot_paths[robot]

            add_reference_to_stage(path_to_robot_usd, robot_prim_path)

            prim = stage.GetPrimAtPath(robot_prim_path)

            physx_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            physx_api.GetSolverPositionIterationCountAttr().Set(32)
            physx_api.GetSolverVelocityIterationCountAttr().Set(1)

            self.articulation = Articulation(robot_prim_path)

        if use_sgraphs:
            lidar_path = get_assets_root_path() + "/Isaac/Sensors/Ouster/OS1/OS1.usd"
            lidar_prim_path = f"/{self.robot_name}/body/lidar"
            add_reference_to_stage(lidar_path, lidar_prim_path)
            self.lidar = stage.GetPrimAtPath(lidar_prim_path)

            xform = UsdGeom.Xformable(self.lidar)

            if robot == "spot":
                xform.GetTranslateOp().Set(Gf.Vec3d(-0.11036, 0.0, 0.08087))
                xform.GetRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

            self.create_graph(lidar_prim_path)
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
        
        if angles is None or velocities is None: 
            print("aborting update, no joint data yet")
            return
        
        if not self.articulation.handles_initialized:
            print("Initializing articulation...")
            self.articulation.initialize()
        
        action = ArticulationAction(
            joint_positions=angles,
            # joint_velocities=velocities
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
