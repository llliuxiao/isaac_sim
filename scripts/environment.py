# This is an Isaac Sim Connection Scripts for single Carter-v1 robot
# Multi Robots simulation can inherit Class IsaacConnection
import angles
import tf2_ros
from omni.isaac.kit import SimulationApp
import os

linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
config = {
    "headless": False
}
simulation_app = SimulationApp(config)

# utils
import sys
import numpy as np
from enum import Enum
import carb
import time

# isaac
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.prims import XFormPrim, GeometryPrim
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.cloner import GridCloner
from omni.isaac.core.articulations import ArticulationView, Articulation

# ros
import rospy
from std_srvs.srv import Empty, EmptyResponse
import rosgraph
from isaac_sim.srv import InitPose, InitPoseResponse
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix, quaternion_matrix
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros import Buffer
from rosgraph_msgs.msg import Clock


class SimulationState(Enum):
    RESET = 0
    PAUSE = 1
    UNPAUSE = 2
    NORMAL = 4
    CLOSE = 8


class IsaacSimConnection:
    def __init__(self, training_scene="warehouse"):
        self.training_scene = training_scene
        self.setup_ros()
        self.setup_scene()
        self.state = SimulationState.NORMAL

    def setup_ros(self):
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)
        self.reset_pose = (1.5, 1.5, 0.5)
        self.time = None
        self.pause_server = rospy.Service("/pause", Empty, self._pause_callback)
        self.unpause_server = rospy.Service("/unpause", Empty, self._unpause_callback)
        self.reset_server = rospy.Service("/reset", InitPose, self._reset_callback)
        self.close_server = rospy.Service("/close", Empty, self._close_callback)
        self.clock_sub = rospy.Subscriber("/clock", Clock, self._clock_callback, queue_size=1)
        # self.tf_buffer = Buffer()
        # self.broadcaster = TransformBroadcaster()
        # self.listener = TransformListener(self.tf_buffer)

    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        if self.training_scene != "warehouse":
            self.world.scene.add_default_ground_plane()
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            print("Could not find Isaac Sim assets folder")
            sys.exit(-1)
        self.robots = []
        self.obstacles = []
        self._add_robot()
        self._add_env()
        self._add_action_graph()

    def cycle(self):
        self.world.reset()
        self.world.initialize_physics()
        simulation_app.update()
        self.world.play()
        simulation_app.update()
        robot = Articulation(prim_path="/World/Carter/chassis_link", name="carter")
        self.world.scene.add(robot)
        while simulation_app.is_running:
            # pos, ori = robot.get_world_pose()
            # self._pub_robot_pose(pos[0], pos[1], ori)
            if self.state == SimulationState.NORMAL:
                self.world.step()
            elif self.state == SimulationState.PAUSE:
                self.world.pause()
            elif self.state == SimulationState.UNPAUSE:
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.RESET:
                self.world.pause()
                self.robot.set_world_pose(
                    position=np.array([self.reset_pose[0], self.reset_pose[1], 0]),
                    orientation=np.array([np.cos(self.reset_pose[2] / 2), 0.0, 0.0, np.sin(self.reset_pose[2] / 2)])
                )
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.CLOSE:
                break
        print("simulation app is out of running")
        self.world.stop()
        simulation_app.close()

    def _add_robot(self):
        wheel_dof_names = ["left_wheel", "right_wheel"]
        self.robot = self.world.scene.add(
            WheeledRobot(
                prim_path="/World/Carter",
                name="Carter",
                wheel_dof_names=wheel_dof_names,
                create_robot=True,
                usd_path=CARTER_USD_PATH,
                position=np.array([self.reset_pose[0], self.reset_pose[1], 0]),
                orientation=np.array([np.cos(self.reset_pose[2] / 2), 0.0, 0.0, np.sin(self.reset_pose[2] / 2)])
            )
        )
        self.robots.append(self.robot)
        self.robot_controller = DifferentialController(name="simple_control", wheel_radius=0.0325, wheel_base=0.1125)

    def _add_env(self):
        ENV_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/{self.training_scene}.usd"
        print(ENV_USD_PATH)
        add_reference_to_stage(usd_path=ENV_USD_PATH, prim_path="/World/Env")
        self.world.scene.add(
            GeometryPrim(
                prim_path="/World/Env",
                name="Env",
                collision=True,
                position=np.array([0.0, 0.0, 0.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0])
            )
        )

    def _add_action_graph(self):
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/clock_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ClockPub", "omni.isaac.ros_bridge.ROS1PublishClock"),
                ],
                keys.SET_VALUES: [
                    ("ClockPub.inputs:topicName", "clock"),
                ],
                keys.CONNECT: [
                    ("SimTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                    ("OnTick.outputs:tick", "ClockPub.inputs:execIn"),
                ]
            }
        )

    def _pause_callback(self, msg):
        self.state = SimulationState.PAUSE
        return EmptyResponse()

    def _unpause_callback(self, msg):
        self.state = SimulationState.UNPAUSE
        return EmptyResponse()

    def _reset_callback(self, msg):
        self.state = SimulationState.RESET
        self.reset_pose = (msg.x, msg.y, msg.yaw)
        return InitPoseResponse()

    def _close_callback(self, msg):
        self.state = SimulationState.CLOSE
        return EmptyResponse()

    def _clock_callback(self, msg: Clock):
        self.time = msg.clock

    # Notice that in ROS, the quaternion is organized as [x, y, z, w], but in isaac sim, it is [w, x, y, z]
    def _pub_robot_pose(self, x, y, quaternion):
        if self.time is None:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame="odom",
                source_frame="base_link",
                time=rospy.Time()
            )
        except tf2_ros.LookupException:
            return
        transform = TransformStamped()
        transform.header.stamp = self.time
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = x - trans.transform.translation.x
        transform.transform.translation.y = y - trans.transform.translation.y
        transform.transform.translation.z = 0.0
        _, _, yaw1 = euler_from_quaternion([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        _, _, yaw2 = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y,
                                            trans.transform.rotation.z, trans.transform.rotation.w])
        print(x, y, yaw1)
        print(transform.transform.translation.x, transform.transform.translation.y)
        q = quaternion_from_euler(0.0, 0.0, angles.normalize_angle(yaw1 - yaw2))
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    if len(sys.argv) > 1:
        scene = sys.argv[1]
        assert scene in ["env", "warehouse"]
        connection = IsaacSimConnection(scene)
    else:
        connection = IsaacSimConnection()
    connection.cycle()
