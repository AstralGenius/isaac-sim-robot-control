import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/astralgenius/workspace/isaac-sim-robot-control/ros2_ws/install/robot_controller'
