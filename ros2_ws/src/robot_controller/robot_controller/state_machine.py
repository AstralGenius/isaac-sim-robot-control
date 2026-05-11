"""
Waypoint State Machine

Pure logic for the waypoint-following state machine.
Computes the next velocity command from the current pose, target,
and state — no ROS 2 dependencies.

The flow:
    ROTATING -> (heading aligned) -> DRIVING -> (reached) -> next waypoint
    Final waypoint reached -> DONE
"""
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Tuple

from robot_controller.pose_utils import normalize_angle


class ControllerState(Enum):
    """States of the waypoint-following controller."""
    ROTATING = auto()    # rotating in place to face the current waypoint
    DRIVING = auto()     # driving toward the current waypoint
    DONE = auto()        # all waypoints reached


@dataclass
class VelocityCommand:
    """A velocity command for the robot in the body frame."""
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass
class ControlGains:
    """Proportional gains and limits for the waypoint controller."""
    kp_linear: float
    kp_angular: float
    max_linear_speed: float
    max_angular_speed: float
    distance_tolerance: float
    heading_tolerance: float


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp a value into [lo, hi]."""
    return max(lo, min(hi, value))


def compute_errors(
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
) -> Tuple[float, float]:
    """Compute the distance and heading errors to a target waypoint.

    Returns:
        Tuple (distance_error, heading_error):
            distance_error: meters to the target
            heading_error: radians to rotate (normalized to [-pi, pi])
    """
    dx = target_x - current_x
    dy = target_y - current_y
    distance_error = math.sqrt(dx * dx + dy * dy)
    target_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(target_heading - current_yaw)
    return distance_error, heading_error


def step(
    state: ControllerState,
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
    gains: ControlGains,
) -> Tuple[ControllerState, VelocityCommand, bool]:
    """Compute the next state and velocity command for a single waypoint.

    This is the heart of the controller — pure function, fully testable.

    Args:
        state: current ControllerState (ROTATING or DRIVING)
        current_x, current_y, current_yaw: current robot pose
        target_x, target_y: current waypoint
        gains: control gains and tolerances

    Returns:
        Tuple (next_state, velocity_command, waypoint_reached):
            next_state: state after this step
            velocity_command: VelocityCommand to publish
            waypoint_reached: True iff the robot just arrived at the waypoint
    """
    distance_error, heading_error = compute_errors(
        current_x, current_y, current_yaw, target_x, target_y
    )

    if state == ControllerState.ROTATING:
        if abs(heading_error) < gains.heading_tolerance:
            # Heading aligned — transition to DRIVING
            return ControllerState.DRIVING, VelocityCommand(), False

        # Keep rotating
        angular_z = _clamp(
            gains.kp_angular * heading_error,
            -gains.max_angular_speed,
            gains.max_angular_speed,
        )
        return state, VelocityCommand(angular_z=angular_z), False

    if state == ControllerState.DRIVING:
        if distance_error < gains.distance_tolerance:
            # Waypoint reached — transition back to ROTATING for the next one
            return ControllerState.ROTATING, VelocityCommand(), True

        # Continue driving with heading correction
        linear_x = _clamp(
            gains.kp_linear * distance_error,
            0.0,
            gains.max_linear_speed,
        )
        angular_z = _clamp(
            gains.kp_angular * heading_error,
            -gains.max_angular_speed,
            gains.max_angular_speed,
        )
        return state, VelocityCommand(linear_x=linear_x, angular_z=angular_z), False

    # DONE state — stay stopped
    return ControllerState.DONE, VelocityCommand(), False