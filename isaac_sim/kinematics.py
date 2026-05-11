"""
Kinematics and Pose Math

Pure functions for robot kinematics and pose conversions.
No Isaac Sim or ROS 2 dependencies — just numpy.

This separation enables fast unit testing without starting Isaac Sim.
"""
import numpy as np


# ============================================================
# Quaternion Conversions
# ============================================================

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles (radians) to quaternion in ROS 2 convention.

    ROS 2 uses (x, y, z, w) order.

    Args:
        roll: rotation around X axis (radians)
        pitch: rotation around Y axis (radians)
        yaw: rotation around Z axis (radians)

    Returns:
        Tuple (x, y, z, w) representing the quaternion.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return (x, y, z, w)


def yaw_from_isaac_quaternion(q) -> float:
    """Extract yaw (Z-axis rotation) from an Isaac Sim quaternion.

    Important: Isaac Sim returns quaternions in (w, x, y, z) order,
    which differs from ROS 2's (x, y, z, w) order.

    Args:
        q: Iterable of 4 floats in (w, x, y, z) order.

    Returns:
        Yaw angle in radians, range [-pi, pi].
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


# ============================================================
# Differential Drive Kinematics
# ============================================================

def twist_to_wheel_velocities(
    linear_x: float,
    angular_z: float,
    wheel_radius: float,
    wheel_separation: float,
) -> tuple:
    """Convert a body-frame twist to left/right wheel velocities.

    This is the inverse kinematics for a differential-drive robot.
    Applies to any robot with two parallel wheels (Jetbot, Turtlebot, Husky).

    Math:
        v_left  = (v - omega * L / 2) / r
        v_right = (v + omega * L / 2) / r

    Where:
        v = linear_x (forward speed, m/s)
        omega = angular_z (yaw rate, rad/s)
        L = wheel_separation (distance between wheels, m)
        r = wheel_radius (m)

    Args:
        linear_x: forward velocity command (m/s)
        angular_z: yaw rate command (rad/s, positive = counterclockwise)
        wheel_radius: radius of the drive wheels (m)
        wheel_separation: distance between the two wheels (m)

    Returns:
        Tuple (v_left, v_right) in rad/s — angular velocities for the wheels.
    """
    half_separation = wheel_separation / 2.0
    v_left = (linear_x - angular_z * half_separation) / wheel_radius
    v_right = (linear_x + angular_z * half_separation) / wheel_radius
    return (v_left, v_right)