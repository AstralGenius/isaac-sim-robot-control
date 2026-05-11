#!/bin/bash
# Launch Isaac Sim with the bundled ROS 2 Jazzy bridge.
#
# Must be run from the project root so that `python -m isaac_sim.simulation`
# can resolve the isaac_sim package.

# Resolve project root (one level up from this script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Path to Isaac Sim's bundled ROS 2 Jazzy libraries
ISAAC_ROS2_DIR="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy"

# Set ROS 2 environment variables
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use Isaac Sim's bundled rclpy (Python 3.11 compatible)
export PYTHONPATH="$ISAAC_ROS2_DIR/rclpy:$PROJECT_ROOT:$PYTHONPATH"
export LD_LIBRARY_PATH="$ISAAC_ROS2_DIR/lib:$LD_LIBRARY_PATH"

# Run as a Python module so package imports resolve
cd "$PROJECT_ROOT"
"$HOME/isaacsim/python.sh" -m isaac_sim.simulation