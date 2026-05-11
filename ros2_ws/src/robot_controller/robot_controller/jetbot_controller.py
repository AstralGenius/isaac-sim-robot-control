"""
Jetbot Waypoint Controller — ROS 2 Node Entry Point

Subscribes to /odom and publishes /cmd_vel, driving the Jetbot
through a sequence of waypoints using closed-loop proportional control.

Architecture:
    /odom (Odometry)  ──►  WaypointController node  ──►  /cmd_vel (Twist)
                                  │
                                  ├─ pose_utils       (pure: quaternion → yaw, angle wrap)
                                  ├─ state_machine    (pure: ROTATE/DRIVE logic)
                                  └─ config           (tunable parameters)

The control logic in state_machine.py is sim-agnostic — the same
controller would drive a real Jetbot publishing /odom and receiving
/cmd_vel over an actual network.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_controller import config
from robot_controller.pose_utils import yaw_from_ros_quaternion
from robot_controller.state_machine import (
    ControllerState,
    ControlGains,
    VelocityCommand,
    step,
)


class WaypointController(Node):
    """ROS 2 node that drives a differential-drive robot through waypoints.

    Subscribes: /odom (nav_msgs/Odometry)
    Publishes: /cmd_vel (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__(config.NODE_NAME)

        # ROS 2 I/O
        self._cmd_vel_publisher = self.create_publisher(
            Twist, config.TOPIC_CMD_VEL, 10
        )
        self._odom_subscription = self.create_subscription(
            Odometry, config.TOPIC_ODOM, self._on_odom, 10
        )
        self._control_timer = self.create_timer(
            config.CONTROL_LOOP_PERIOD, self._control_loop
        )

        # Pose state (updated from /odom)
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._odom_received = False

        # Waypoint state
        self._waypoints = list(config.DEFAULT_WAYPOINTS)
        self._waypoint_index = 0
        self._state = ControllerState.ROTATING
        self._done_logged = False

        # Gains bundle (from config) — passed into the pure state machine
        self._gains = ControlGains(
            kp_linear=config.KP_LINEAR,
            kp_angular=config.KP_ANGULAR,
            max_linear_speed=config.MAX_LINEAR_SPEED,
            max_angular_speed=config.MAX_ANGULAR_SPEED,
            distance_tolerance=config.DISTANCE_TOLERANCE,
            heading_tolerance=config.HEADING_TOLERANCE,
        )

        self.get_logger().info('Waypoint controller started')
        self.get_logger().info(f'Waypoints: {self._waypoints}')

    # --------------------------------------------------------
    # ROS 2 callbacks
    # --------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        """Cache the latest pose from /odom messages."""
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._current_yaw = yaw_from_ros_quaternion(msg.pose.pose.orientation)
        self._odom_received = True

    def _control_loop(self) -> None:
        """Periodic control tick (runs at config.CONTROL_LOOP_PERIOD)."""
        if not self._odom_received:
            return

        if self._state == ControllerState.DONE:
            self._publish_stop()
            if not self._done_logged:
                self.get_logger().info('All waypoints reached. Stopping.')
                self._done_logged = True
            return

        # Current target waypoint
        target_x, target_y = self._waypoints[self._waypoint_index]

        # Pure logic — compute next state and command
        next_state, command, waypoint_reached = step(
            state=self._state,
            current_x=self._current_x,
            current_y=self._current_y,
            current_yaw=self._current_yaw,
            target_x=target_x,
            target_y=target_y,
            gains=self._gains,
        )

        # Log transitions
        if next_state != self._state:
            if next_state == ControllerState.DRIVING:
                self.get_logger().info(
                    f'Aligned with waypoint {self._waypoint_index + 1}. Driving forward.'
                )

        # Advance to next waypoint if we just reached one
        if waypoint_reached:
            self.get_logger().info(
                f'Reached waypoint {self._waypoint_index + 1}: ({target_x}, {target_y})'
            )
            self._waypoint_index += 1
            if self._waypoint_index >= len(self._waypoints):
                next_state = ControllerState.DONE

        self._state = next_state
        self._publish(command)

    # --------------------------------------------------------
    # Publishing helpers
    # --------------------------------------------------------

    def _publish(self, command: VelocityCommand) -> None:
        """Convert a VelocityCommand to a Twist message and publish."""
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self._cmd_vel_publisher.publish(msg)

    def _publish_stop(self) -> None:
        """Publish a zero-velocity command to halt the robot."""
        self._publish(VelocityCommand())


# --------------------------------------------------------
# Entry point for `ros2 run robot_controller jetbot_controller`
# --------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    controller = WaypointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()