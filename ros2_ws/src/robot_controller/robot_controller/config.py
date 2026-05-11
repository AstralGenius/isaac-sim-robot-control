"""
Controller Configuration

Centralizes all tunable parameters for the Jetbot waypoint controller.
Edit values here rather than searching through controller code.
"""
from typing import Tuple, List


# ============================================================
# ROS 2 Communication
# ============================================================

NODE_NAME = "jetbot_controller"

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ODOM = "/odom"


# ============================================================
# Control Loop Timing
# ============================================================

CONTROL_LOOP_PERIOD = 0.05   # seconds — 20 Hz control rate


# ============================================================
# Waypoint Trajectory
# ============================================================
# Default trajectory: a 2x2 m square starting from the origin.
# Each waypoint is (x, y) in meters in the odom frame.

DEFAULT_WAYPOINTS: List[Tuple[float, float]] = [
    (2.0, 0.0),   # waypoint 1
    (2.0, 2.0),   # waypoint 2
    (0.0, 2.0),   # waypoint 3
    (0.0, 0.0),   # waypoint 4 — back to start
]


# ============================================================
# Control Tolerances
# ============================================================
# How close is "close enough" to consider a waypoint reached
# or a heading aligned.

DISTANCE_TOLERANCE = 0.1     # meters
HEADING_TOLERANCE = 0.05     # radians (~3 degrees)


# ============================================================
# Control Gains (Proportional)
# ============================================================
# These convert errors (distance, heading) into velocity commands.
# Higher = more aggressive, but more prone to overshoot/oscillation.

KP_LINEAR = 0.5              # forward speed gain (m/s per meter of error)
KP_ANGULAR = 1.5             # rotation gain (rad/s per radian of error)


# ============================================================
# Velocity Limits
# ============================================================
# Hard caps on command output. Prevents the robot from exceeding
# safe speeds even with large errors.

MAX_LINEAR_SPEED = 0.3       # m/s
MAX_ANGULAR_SPEED = 1.5      # rad/s