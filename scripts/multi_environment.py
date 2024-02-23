# This is an Isaac Sim Connection Scripts for single Carter-v1 robot
# Multi Robots simulation can inherit Class IsaacConnection

from omni.isaac.kit import SimulationApp
import os

linux_user = os.getlogin()
CARTER_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/carter.usd"
ENV_USD_PATH = f"/home/{linux_user}/isaac_sim_ws/src/isaac_sim/isaac/env.usd"
config = {
    "headless": False,
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

# ros
import rospy
from std_srvs.srv import Empty, EmptyResponse
import rosgraph


class SimulationState(Enum):
    RESET = 0
    PAUSE = 1
    UNPAUSE = 2
    NORMAL = 4
    CLOSE = 8


class IsaacSimConnection:
    interval = 1.0

    def __init__(self):
        self.setup_ros()
        self.setup_scene()
        self.state = SimulationState.NORMAL

    def setup_ros(self):
        enable_extension("omni.isaac.ros_bridge")
        while not rosgraph.is_master_online():
            carb.log_error("Please run roscore before executing this script")
            time.sleep(2.0)
        self.pause_sub = rospy.Service("/pause", Empty, self._pause_callback)
        self.unpause_sub = rospy.Service("/unpause", Empty, self._unpause_callback)
        self.reset_sub = rospy.Service("/reset", Empty, self._reset_callback)
        self.close_sub = rospy.Service("/close", Empty, self._close_callback)
        # while not rospy.has_param("robots_rows"):
        #     rospy.sleep(1)
        self.robots_rows = rospy.get_param("/robots_rows", 2)
        self.robots_columns = rospy.get_param("/robots_columns", 2)
        self.map_height = rospy.get_param("/map_height", 12)
        self.map_width = rospy.get_param("/map_width", 12)

    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            print("Could not find Isaac Sim assets folder")
            sys.exit(-1)
        self.robots = []
        self._add_action_graph()
        for row in range(self.robots_rows):
            for column in range(self.robots_columns):
                prefix = row * self.robots_columns + column
                x = self.map_width * column + column * self.interval
                y = self.map_height * row + row * self.interval
                self._add_env(prefix=prefix, position=(x, y, 0))
                self._add_robot(position=(x + 1.5, y + 1.5, 0), prefix=prefix)
                self.world.reset()
                self._set_namespace(prefix=prefix)
    #
    # self.world.reset()
    #     for prefix in range(self.robots_rows * self.robots_columns):
    #         self._set_namespace(prefix=prefix)

    def cycle(self):
        self.world.reset()
        self.world.initialize_physics()
        simulation_app.update()
        self.world.play()
        self.world.pause()
        self.world.reset()
        simulation_app.update()
        while simulation_app.is_running:
            if self.state == SimulationState.NORMAL:
                self.world.step()
            elif self.state == SimulationState.PAUSE:
                self.world.pause()
            elif self.state == SimulationState.UNPAUSE:
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.RESET:
                self.world.play()
                self.world.step()
                self.state = SimulationState.NORMAL
            elif self.state == SimulationState.CLOSE:
                break
        print("simulation app is out of running")
        self.world.stop()
        simulation_app.close()

    def _add_robot(self, position, orientation=(1.0, 0.0, 0.0, 0.0), prefix=0):
        wheel_dof_names = ["left_wheel", "right_wheel"]
        robot = self.world.scene.add(
            WheeledRobot(
                prim_path=f"/World/Carter{prefix}",
                name=f"Carter{prefix}",
                wheel_dof_names=wheel_dof_names,
                create_robot=True,
                usd_path=CARTER_USD_PATH,
                position=np.array(position),
                orientation=np.array(orientation),
            )
        )
        self.robots.append(robot)

    def _add_env(self, position, prefix=0):
        add_reference_to_stage(usd_path=ENV_USD_PATH, prim_path=f"/World/Env{prefix}")
        self.world.scene.add(
            GeometryPrim(
                prim_path=f"/World/Env{prefix}",
                name=f"Env{prefix}",
                position=np.array(position),
                collision=True
            )
        )

    @staticmethod
    def _add_action_graph():
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

    def _set_namespace(self, prefix=0):
        graph = og.Controller.graph(f"/World/Carter{prefix}/Carter_Control_Graph")
        og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value=f"Carter{prefix}")
        graph = og.Controller.graph(f"/World/Carter{prefix}/Carter_Sensor_Graph")
        og.GraphController.set_variable_default_value(variable_id=(graph, "namespace"), value=f"Carter{prefix}")

    def _pause_callback(self, msg):
        self.state = SimulationState.PAUSE
        return EmptyResponse()

    def _unpause_callback(self, msg):
        self.state = SimulationState.UNPAUSE
        return EmptyResponse()

    def _reset_callback(self, msg):
        self.state = SimulationState.RESET
        return EmptyResponse()

    def _close_callback(self, msg):
        self.state = SimulationState.CLOSE
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("IsaacSimConnection")
    connection = IsaacSimConnection()
    connection.cycle()
