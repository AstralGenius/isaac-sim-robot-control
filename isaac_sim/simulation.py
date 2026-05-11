"""
Isaac Sim Standalone Simulation — Entry Point

Bootstraps Isaac Sim, wires the ROS 2 bridge to the simulated robot,
and runs the main physics + communication loop.

This file is intentionally short. Detailed responsibilities live in:
- config.py        — tunable parameters
- kinematics.py    — pure math (no Isaac Sim deps)
- robot_bridge.py  — ROS 2 publisher/subscriber node
- scene_setup.py   — Isaac Sim world and robot loading

Run with:
    ./isaac_sim/run_simulation.sh
"""
# Isaac Sim must launch before any omni/isaacsim modules are imported.
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
})

# Enable the ROS 2 bridge (provides Python 3.11-compatible rclpy).
# Must happen after SimulationApp() but before importing rclpy.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Now safe to import everything else.
import numpy as np
import rclpy

from isaacsim.core.utils.types import ArticulationAction

from isaac_sim.kinematics import yaw_from_isaac_quaternion
from isaac_sim.robot_bridge import IsaacSimBridge
from isaac_sim.scene_setup import setup_scene


def run_simulation_loop(simulation_app, world, robot, articulation_controller, bridge):
    """Main physics + communication loop.

    Each iteration:
        1. Step Isaac Sim physics (and render).
        2. Service pending ROS 2 messages without blocking.
        3. Read robot state from physics and publish /odom.
        4. Apply the latest /cmd_vel as wheel velocities.

    Exits cleanly when the Isaac Sim window is closed.
    """
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(bridge, timeout_sec=0.0)

        # Read current state from Isaac Sim physics
        position, orientation = robot.get_world_pose()
        linear_velocity = robot.get_linear_velocity()
        angular_velocity = robot.get_angular_velocity()
        yaw = yaw_from_isaac_quaternion(orientation)

        # Publish /odom for the controller to consume
        bridge.publish_odometry(
            position=position,
            yaw=yaw,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )

        # Apply the latest velocity command to the wheels
        v_left, v_right = bridge.get_wheel_velocities()
        articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=None,
                joint_efforts=None,
                joint_velocities=np.array([v_left, v_right]),
            )
        )


def main():
    """Program entry point."""
    rclpy.init()
    bridge = IsaacSimBridge()

    world, robot, articulation_controller = setup_scene()
    bridge.get_logger().info('Jetbot loaded — entering simulation loop.')

    try:
        run_simulation_loop(
            simulation_app=simulation_app,
            world=world,
            robot=robot,
            articulation_controller=articulation_controller,
            bridge=bridge,
        )
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == '__main__':
    main()