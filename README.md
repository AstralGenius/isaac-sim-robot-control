# Isaac Sim Robot Control

A production-grade mobile robotics project demonstrating sim-to-real architecture using **NVIDIA Isaac Sim** and **ROS 2 Jazzy**. The robot autonomously navigates a sequence of waypoints using closed-loop control with real-time odometry feedback. The control stack is fully decoupled from the simulator — the same code could drive a real Jetbot without modification.

[![Isaac Sim Robot Demo](https://img.youtube.com/vi/Z3b984r81SI.jpg)](https://youtu.be/Z3b984r81SI)

---

## Architecture

```
┌─────────────────────┐                ┌─────────────────────┐
│  ROS 2 Controller   │   /cmd_vel     │  Isaac Sim          │
│  Node               │   ──────►      │  Standalone Script  │
│  (Python 3.12)      │                │  (Python 3.11)      │
│                     │   /odom        │                     │
│                     │   ◄──────      │                     │
└─────────────────────┘                └──────────┬──────────┘
                                                  │
                                            FastDDS bridge
                                                  │
                                                  ▼
                                            ┌──────────┐
                                            │  Jetbot  │
                                            │  in Sim  │
                                            └──────────┘
```

Two independent processes communicate via ROS 2 topics over FastDDS. This separation enables:

- **Sim-to-real transfer** — the controller is simulator-agnostic
- **Independent testing** — controller logic can be unit-tested without Isaac Sim
- **Production patterns** — matches the architecture used by Figure, 1X, and Boston Dynamics

---

## Features

### Closed-loop autonomous waypoint navigation
- Robot drives a configurable sequence of waypoints (default: 2×2m square)
- State machine alternates between **ROTATE** (align to target) and **DRIVE** (move toward target)
- Proportional control on both heading and distance errors
- Real-time pose feedback from `/odom`

### Production-grade ROS 2 architecture
- ROS 2 Jazzy controller node publishing differential drive commands
- Standalone Isaac Sim script with embedded ROS 2 bridge
- Differential drive kinematics (`Twist` → wheel velocity conversion)
- 20 Hz control loop with deterministic behavior
- Industry-standard colcon workspace and package layout

---

## Tech Stack

- **Simulation:** NVIDIA Isaac Sim 5.0 (RTX 5090, driver 580 open kernel modules)
- **Middleware:** ROS 2 Jazzy
- **Language:** Python 3.11 (Isaac Sim) / Python 3.12 (ROS 2)
- **DDS:** FastDDS
- **Build:** colcon, ament_python
- **OS:** Ubuntu 24.04 LTS (kernel 6.17)

---

## Project Structure

```
isaac-sim-robot-control/
├── isaac_sim/
│   ├── simulation.py              # Standalone Isaac Sim script with ROS 2 bridge
│   └── run_simulation.sh          # Launch script with proper env vars
├── ros2_ws/
│   └── src/
│       ├── robot_controller/      # Waypoint follower (closed-loop)
│       ├── robot_simulation/      # Future: Isaac Sim helpers
│       ├── robot_description/     # Future: URDF model
│       └── robot_bringup/         # Future: launch files
├── docker/                        # Future: containerized deployment
├── docs/                          # Architecture notes
├── tests/                         # Unit tests
├── media/
│   └── demo.mp4
└── README.md
```

---

## Quick Start

### Prerequisites

- Ubuntu 24.04 LTS
- NVIDIA GPU with driver 580+ (open kernel modules required for RTX 50-series)
- NVIDIA Isaac Sim 5.0
- ROS 2 Jazzy

### Build the workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run the simulation

**Terminal 1** — Start Isaac Sim with the ROS 2 bridge:

```bash
./isaac_sim/run_simulation.sh
```

**Terminal 2** — Start the waypoint controller:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run robot_controller jetbot_controller
```

The Jetbot will autonomously drive a 0.5×0.5m square: rotating to face each waypoint, then driving to it, until the loop is complete.

### Verify topics

```bash
ros2 topic list             # /cmd_vel, /odom should appear
ros2 topic echo /odom       # Real-time pose feedback
ros2 topic echo /cmd_vel    # Velocity commands from the controller
```

---

## How It Works

### Differential Drive Kinematics

The controller publishes high-level `Twist` messages with linear and angular velocities. The Isaac Sim bridge converts these to individual wheel velocities:

```
v_left  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
v_right = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
```

This matches the kinematics of any differential-drive robot (Jetbot, Turtlebot, Husky, etc).

### Closed-Loop Waypoint Following

The controller subscribes to `/odom` and runs a state machine for each waypoint:

1. **ROTATE** — Compute `heading_error = atan2(dy, dx) - current_yaw`. Apply proportional angular velocity until `|heading_error| < 0.05 rad`.
2. **DRIVE** — Compute `distance_error = sqrt(dx² + dy²)`. Apply proportional linear velocity (with continued heading correction) until `distance_error < 0.1 m`.
3. Advance to next waypoint, repeat.

The same logic would work on a real robot — just swap the Isaac Sim simulation for actual hardware odometry.

### Python Interop

Isaac Sim 5.0 ships Python 3.11; ROS 2 Jazzy targets Python 3.12. The Isaac Sim ROS 2 bridge ships its own Python 3.11-compatible `rclpy`, which is loaded via `LD_LIBRARY_PATH` and `PYTHONPATH` injection in `run_simulation.sh`. Inter-process communication happens at the FastDDS layer, so Python version mismatch doesn't matter at the wire level.

---

## Roadmap

### Phase 1 — Foundation ✅
- [x] Standalone Isaac Sim + ROS 2 architecture
- [x] Open-loop differential drive controller
- [x] Modular colcon workspace

### Phase 2 — Closed-loop control ✅
- [x] Publish `/odom` from Isaac Sim with full pose and twist
- [x] Subscribe to `/odom` in controller
- [x] State machine: ROTATE → DRIVE per waypoint
- [x] Proportional control on heading and distance errors
- [x] Configurable waypoint sequence

### Phase 3 — Perception 🚧
- [ ] Add RTX LiDAR sensor in Isaac Sim
- [ ] Publish `/scan` topic (sensor_msgs/LaserScan)
- [ ] Implement reactive obstacle avoidance
- [ ] Combine waypoint following with obstacle detection

### Phase 4 — Autonomous Navigation
- [ ] Integrate Nav2 stack
- [ ] SLAM Toolbox for mapping
- [ ] Goal-based navigation with behavior trees

### Phase 5 — Production Engineering
- [ ] Dockerfile and docker-compose for reproducibility
- [ ] Unit tests with pytest, integration tests with launch_testing
- [ ] GitHub Actions CI/CD pipeline
- [ ] Comprehensive technical documentation

---

## Engineering Notes

A detailed write-up of architecture decisions, sim-to-real considerations, and the Python version interop solution will be added in final phase.

---

## Author

**Vijay Yadav** — Robotics & AI Engineer

- LinkedIn: [vijay-yadav1995](https://www.linkedin.com/in/vijay-yadav1995)
- GitHub: [AstralGenius](https://github.com/AstralGenius)

---

## Acknowledgments

The Jetbot model and assets are provided by NVIDIA Isaac Sim. The architectural patterns demonstrated here are inspired by production robotics deployments at companies including Figure, 1X, and Boston Dynamics AI Institute.

## License

Apache License 2.0
