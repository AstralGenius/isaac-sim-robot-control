"""
Pose and Angle Utilities

Pure functions for working with poses and angles in the ROS 2 controller.
No rclpy dependencies — just math.
"""
import math


def yaw_from_ros_quaternion(q) -> float:
    """Extract yaw (Z-axis rotation) from a ROS 2 quaternion.

    ROS 2 uses (x, y, z, w) order — accessed as q.x, q.y, q.z, q.w
    on a geometry_msgs/Quaternion object.

    Args:
        q: A quaternion-like object exposing .x, .y, .z, .w attributes
           (e.g. geometry_msgs.msg.Quaternion).

    Returns:
        Yaw angle in radians, range [-pi, pi].
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap an angle into the range [-pi, pi].

    Used for computing the shortest angular distance between
    two headings — without normalization, a robot might choose
    to turn 270 degrees clockwise instead of 90 degrees counterclockwise.

    Args:
        angle: Angle in radians (any value).

    Returns:
        Equivalent angle in [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle