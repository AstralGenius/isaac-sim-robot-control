"""
Jetbot Waypoint Controller Node

Closed-loop controller that drives the Jetbot through a sequence of waypoints
using odometry feedback.

Behavior:
- For each waypoint, first rotates to face it, then drives to it
- Uses simple proportional control on heading and distance errors
- Publishes /cmd_vel based on current pose from /odom

This node is sim-agnostic — same code would drive a real differential-drive robot.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WaypointController(Node):
    def __init__(self):
        super().__init__('jetbot_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry feedback
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Control loop runs at 20 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Robot's current pose (updated from /odom)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Waypoints (a 2x2 meter square loop)
        self.waypoints = [
            (0.5, 0.0),   # waypoint 1
            (0.5, 0.5),   # waypoint 2
            (0.0, 0.5),   # waypoint 3
            (0.0, 0.0),   # waypoint 4
        ]
        self.current_waypoint_index = 0

        # State machine: ROTATING -> DRIVING -> ROTATING -> DRIVING ...
        self.state = 'ROTATING'

        # Tolerances
        self.distance_tolerance = 0.1   # meters — how close is "arrived"
        self.heading_tolerance = 0.05   # radians (~3°)

        # Control gains (tuned for Jetbot)
        self.kp_linear = 0.5            # forward speed gain
        self.kp_angular = 1.5           # rotation gain
        self.max_linear_speed = 0.3     # m/s — Jetbot's max safe speed
        self.max_angular_speed = 1.5    # rad/s

        self.get_logger().info('Waypoint controller started')
        self.get_logger().info(f'Waypoints: {self.waypoints}')

    def odom_callback(self, msg: Odometry):
        """Update current pose from /odom messages."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def control_loop(self):
        """Main control loop — runs at 20 Hz."""
        # Wait until we've received at least one odom message
        if not self.odom_received:
            return

        # All waypoints completed?
        if self.current_waypoint_index >= len(self.waypoints):
            self._publish_stop()
            if not getattr(self, '_done_logged', False):
                self.get_logger().info('All waypoints reached. Stopping.')
                self._done_logged = True
            return

        # Compute errors to current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_error = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_heading - self.current_yaw)

        # Build the velocity command
        msg = Twist()

        if self.state == 'ROTATING':
            # Rotate in place until we're facing the waypoint
            if abs(heading_error) < self.heading_tolerance:
                self.state = 'DRIVING'
                self.get_logger().info(
                    f'Aligned with waypoint {self.current_waypoint_index + 1}. Driving forward.'
                )
            else:
                msg.angular.z = self._clamp(
                    self.kp_angular * heading_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )

        elif self.state == 'DRIVING':
            # Drive forward toward the waypoint
            if distance_error < self.distance_tolerance:
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_index + 1}: '
                    f'({target_x}, {target_y})'
                )
                self.current_waypoint_index += 1
                self.state = 'ROTATING'
            else:
                # Forward speed proportional to distance, but corrected for heading drift
                msg.linear.x = self._clamp(
                    self.kp_linear * distance_error,
                    0.0,
                    self.max_linear_speed,
                )
                # Small heading correction while driving
                msg.angular.z = self._clamp(
                    self.kp_angular * heading_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )

        self.cmd_vel_publisher.publish(msg)

    def _publish_stop(self):
        """Publish a zero-velocity Twist."""
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _normalize_angle(angle):
        """Wrap an angle into [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


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