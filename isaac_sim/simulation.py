"""
Isaac Sim Standalone Simulation with Odometry

Loads the Jetbot in Isaac Sim and bridges it to ROS 2:
- Subscribes to /cmd_vel (from controller)
- Publishes /odom (the robot's actual position and velocity)

Run with:
    ./isaac_sim/run_simulation.sh
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import carb
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage


# Jetbot physical parameters (from the USD file)
WHEEL_RADIUS = 0.03      # meters
WHEEL_SEPARATION = 0.1125  # meters between wheels


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def get_yaw_from_quaternion(q):
    """Extract yaw (rotation around Z) from a quaternion (w, x, y, z) — Isaac Sim format."""
    # Isaac Sim returns quaternions as (w, x, y, z)
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


class IsaacSimBridge(Node):
    """ROS 2 node embedded inside the Isaac Sim process.

    Subscribes to /cmd_vel and publishes /odom.
    """

    def __init__(self):
        super().__init__('isaac_sim_bridge')

        # Subscribe to velocity commands from the controller
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Publish odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Latest commanded velocities (default = stopped)
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Isaac Sim ROS 2 bridge started')
        self.get_logger().info('Subscribed: /cmd_vel | Published: /odom')

    def cmd_vel_callback(self, msg: Twist):
        """Called whenever a new Twist message arrives."""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def get_wheel_velocities(self):
        """Convert linear/angular velocity to left/right wheel velocities (rad/s)."""
        v_left = (self.linear_x - self.angular_z * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        v_right = (self.linear_x + self.angular_z * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        return v_left, v_right

    def publish_odometry(self, position, orientation_yaw, linear_vel, angular_vel):
        """Publish the robot's current state as a nav_msgs/Odometry message."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Position (in the world frame)
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])

        # Orientation as quaternion (yaw only — robot is on flat ground)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, orientation_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Velocities (in the robot's local frame)
        msg.twist.twist.linear.x = float(linear_vel[0])
        msg.twist.twist.linear.y = float(linear_vel[1])
        msg.twist.twist.linear.z = float(linear_vel[2])
        msg.twist.twist.angular.x = float(angular_vel[0])
        msg.twist.twist.angular.y = float(angular_vel[1])
        msg.twist.twist.angular.z = float(angular_vel[2])

        self.odom_publisher.publish(msg)


def main():
    rclpy.init()
    bridge = IsaacSimBridge()

    # Create the Isaac Sim world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Load the Jetbot
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Could not find Isaac Sim assets path")
        simulation_app.close()
        return

    jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    add_reference_to_stage(usd_path=jetbot_usd, prim_path="/World/Jetbot")
    jetbot = world.scene.add(Robot(prim_path="/World/Jetbot", name="jetbot"))

    # Reset the world to initialize physics
    world.reset()
    articulation_controller = jetbot.get_articulation_controller()

    bridge.get_logger().info('Jetbot loaded. Publishing /odom and listening on /cmd_vel...')

    # Main simulation loop
    try:
        while simulation_app.is_running():
            # Step Isaac Sim physics
            world.step(render=True)

            # Process any pending ROS 2 messages (non-blocking)
            rclpy.spin_once(bridge, timeout_sec=0.0)

            # Read robot state from Isaac Sim
            position, orientation = jetbot.get_world_pose()
            linear_vel = jetbot.get_linear_velocity()
            angular_vel = jetbot.get_angular_velocity()
            yaw = get_yaw_from_quaternion(orientation)

            # Publish odometry
            bridge.publish_odometry(position, yaw, linear_vel, angular_vel)

            # Convert latest cmd_vel to wheel velocities
            v_left, v_right = bridge.get_wheel_velocities()

            # Apply to the Jetbot
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