"""
Jetbot Controller Node

Implements a simple state machine that drives the Jetbot:
- 3 seconds forward
- 1 second turn in place
- Repeat

Publishes Twist messages on /cmd_vel.
This node is sim-agnostic — it would work identically with a real Jetbot.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JetbotController(Node):
    def __init__(self):
        super().__init__('jetbot_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10  # queue size
        )

        # Control loop runs at 20 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # State tracking
        self.start_time = self.get_clock().now()

        # Behavior parameters
        self.forward_duration = 3.0  # seconds
        self.turn_duration = 1.0     # seconds
        self.cycle_duration = self.forward_duration + self.turn_duration

        self.linear_speed = 0.2      # m/s — forward speed
        self.angular_speed = 1.0     # rad/s — turning speed

        self.get_logger().info('Jetbot controller started')

    def control_loop(self):
        # How long since we started?
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Where are we in the current cycle?
        cycle_position = elapsed % self.cycle_duration

        # Build the velocity command
        msg = Twist()

        if cycle_position < self.forward_duration:
            # Phase 1: Drive forward
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
        else:
            # Phase 2: Turn in place
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = JetbotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()