"""
Simulation Configuration

Centralizes all tunable parameters for the Isaac Sim side of the project.
Edit values here rather than searching through the simulation code.
"""

# ============================================================
# Robot Physical Parameters (Jetbot)
# ============================================================
# These values come from the Jetbot USD file and define how
# Twist commands map to wheel velocities.

WHEEL_RADIUS = 0.03          # meters
WHEEL_SEPARATION = 0.1125    # meters between left and right wheels


# ============================================================
# Robot Asset Path
# ============================================================
# Path to the robot USD inside the Isaac Sim asset server.
# To use a different robot, change this path.

ROBOT_USD_RELATIVE_PATH = "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
ROBOT_PRIM_PATH = "/World/Jetbot"
ROBOT_NAME = "jetbot"


# ============================================================
# Simulation World
# ============================================================

STAGE_UNITS_IN_METERS = 1.0   # 1 unit = 1 meter (SI units)


# ============================================================
# ROS 2 Topics
# ============================================================

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ODOM = "/odom"
TOPIC_SCAN = "/scan"          # used in Phase 3

ODOM_FRAME_ID = "odom"
ROBOT_FRAME_ID = "base_link"


# ============================================================
# Logging
# ============================================================

NODE_NAME = "isaac_sim_bridge"