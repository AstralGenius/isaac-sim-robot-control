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


# ============================================================
# LiDAR Sensor
# ============================================================
# SLAMTEC RPLidar S2E — 2D rotating LiDAR, 360°, 10 Hz, 30m range.
# Config file lives in Isaac Sim's lidar_configs directory.

LIDAR_CONFIG_NAME = "NVIDIA/Debug_Rotary"
LIDAR_PRIM_PATH = "/World/Jetbot/chassis/lidar"
LIDAR_FRAME_ID = "lidar_link"

# Mount position relative to the chassis (meters).
# 0.1 m above the chassis center gives clearance above the camera bracket.
LIDAR_MOUNT_TRANSLATION = (0.0, 0.0, 0.1)


# ============================================================
# Obstacles
# ============================================================
# Cuboid obstacles placed on the robot's path.
# Each tuple: (name, position, size, color)
# - position is the cube's center in world frame (meters)
# - size is the cube's edge length in meters (uniform cube)
# - color is RGB normalized to [0, 1]

OBSTACLE_DEFINITIONS = [
    ("obstacle_1", (1.5, 0.0, 0.075), 0.15, (1.0, 0.3, 0.3)),
    ("obstacle_2", (2.0, 1.0, 0.075), 0.15, (1.0, 0.3, 0.3)),
    ("obstacle_3", (1.0, 2.0, 0.075), 0.15, (1.0, 0.3, 0.3)),
]