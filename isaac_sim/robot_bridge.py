"""
ROS 2 Bridge for Isaac Sim

Implements the IsaacSimBridge node that mediates between
Isaac Sim physics and ROS 2 topics.

Responsibilities:
- Subscribe to /cmd_vel (Twist) for velocity commands
- Publish /odom (Odometry) with the robot's pose and velocity
- Hold latest velocity command in a thread-safe-enough way for the main loop

Pure math lives in kinematics.py; this module only handles ROS 2 I/O.
"""
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from isaac_sim import config
from isaac_sim.kinematics import quaternion_from_euler, twist_to_wheel_velocities


class IsaacSimBridge(Node):
    """ROS 2 node embedded in the Isaac Sim process.

    Acts as the bridge between simulation state and ROS 2 messages.
    Subscribes to /cmd_vel and publishes /odom.
    """

    def __init__(self):
        super().__init__(config.NODE_NAME)

        # Subscriber: latest velocity command from the controller
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            config.TOPIC_CMD_VEL,
            self._on_cmd_vel,
            10,
        )

        # Publisher: robot's current pose and velocity
        self.odom_publisher = self.create_publisher(
            Odometry,
            config.TOPIC_ODOM,
            10,
        )

        # Cached velocity command — default to stopped
        self._linear_x = 0.0
        self._angular_z = 0.0

        self.get_logger().info('Isaac Sim ROS 2 bridge started')
        self.get_logger().info(
            f'Subscribed: {config.TOPIC_CMD_VEL} | '
            f'Published: {config.TOPIC_ODOM}'
        )

    # --------------------------------------------------------
    # ROS 2 callbacks
    # --------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Cache the latest velocity command for the simulation loop to read."""
        self._linear_x = msg.linear.x
        self._angular_z = msg.angular.z

    # --------------------------------------------------------
    # Public API used by the main simulation loop
    # --------------------------------------------------------

    def get_wheel_velocities(self) -> tuple:
        """Return the wheel velocities corresponding to the latest cmd_vel."""
        return twist_to_wheel_velocities(
            linear_x=self._linear_x,
            angular_z=self._angular_z,
            wheel_radius=config.WHEEL_RADIUS,
            wheel_separation=config.WHEEL_SEPARATION,
        )

    def publish_odometry(
        self,
        position,
        yaw: float,
        linear_velocity,
        angular_velocity,
    ) -> None:
        """Build and publish an Odometry message from the robot's current state.

        Args:
            position: iterable of 3 floats (x, y, z) in world frame
            yaw: rotation around Z axis in radians
            linear_velocity: iterable of 3 floats (vx, vy, vz)
            angular_velocity: iterable of 3 floats (wx, wy, wz)
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = config.ODOM_FRAME_ID
        msg.child_frame_id = config.ROBOT_FRAME_ID

        # Position (world frame)
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])

        # Orientation as quaternion (yaw-only — robot on flat ground)
        qx, qy, qz, qw = quaternion_from_euler(roll=0.0, pitch=0.0, yaw=yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Linear velocity
        msg.twist.twist.linear.x = float(linear_velocity[0])
        msg.twist.twist.linear.y = float(linear_velocity[1])
        msg.twist.twist.linear.z = float(linear_velocity[2])

        # Angular velocity
        msg.twist.twist.angular.x = float(angular_velocity[0])
        msg.twist.twist.angular.y = float(angular_velocity[1])
        msg.twist.twist.angular.z = float(angular_velocity[2])

        self.odom_publisher.publish(msg)