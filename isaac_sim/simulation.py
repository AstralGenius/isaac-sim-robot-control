"""
Isaac Sim Standalone Simulation

Loads the Jetbot in Isaac Sim and bridges it to ROS 2 using
Isaac Sim's built-in ROS 2 bridge (compatible with the bundled Python).

Run with:
    ~/isaacsim/python.sh isaac_sim/simulation.py
"""
from isaacsim import SimulationApp

# Launch Isaac Sim with the ROS 2 bridge enabled
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
})

# Enable the ROS 2 bridge extension
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# Now we can import rclpy — it uses Isaac Sim's bundled version
simulation_app.update()

import carb
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage


WHEEL_RADIUS = 0.03
WHEEL_SEPARATION = 0.1125


class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Isaac Sim ROS 2 bridge started')

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def get_wheel_velocities(self):
        v_left = (self.linear_x - self.angular_z * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        v_right = (self.linear_x + self.angular_z * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        return v_left, v_right


def main():
    rclpy.init()
    bridge = IsaacSimBridge()

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Could not find Isaac Sim assets path")
        simulation_app.close()
        return

    jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    add_reference_to_stage(usd_path=jetbot_usd, prim_path="/World/Jetbot")
    jetbot = world.scene.add(Robot(prim_path="/World/Jetbot", name="jetbot"))

    world.reset()
    articulation_controller = jetbot.get_articulation_controller()

    bridge.get_logger().info('Jetbot loaded. Waiting for /cmd_vel commands...')

    try:
        while simulation_app.is_running():
            world.step(render=True)
            rclpy.spin_once(bridge, timeout_sec=0.0)

            v_left, v_right = bridge.get_wheel_velocities()
            articulation_controller.apply_action(
                ArticulationAction(
                    joint_positions=None,
                    joint_efforts=None,
                    joint_velocities=np.array([v_left, v_right])
                )
            )

    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == '__main__':
    main()