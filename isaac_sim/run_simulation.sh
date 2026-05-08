#!/bin/bash
# Launch Isaac Sim with the bundled ROS 2 Jazzy bridge

# Path to Isaac Sim's bundled ROS 2 Jazzy libraries
ISAAC_ROS2_DIR="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy"

# Set ROS 2 environment variables
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use Isaac Sim's bundled rclpy (Python 3.11 compatible) — must come BEFORE system paths
export PYTHONPATH="$ISAAC_ROS2_DIR/rclpy:$PYTHONPATH"

# Add bundled native libraries
export LD_LIBRARY_PATH="$ISAAC_ROS2_DIR/lib:$LD_LIBRARY_PATH"

# Run Isaac Sim with our simulation script
$HOME/isaacsim/python.sh $HOME/workspace/isaac-sim-robot-control/isaac_sim/simulation.py