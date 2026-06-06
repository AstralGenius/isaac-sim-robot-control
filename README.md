# Isaac Sim Robot Control

A production-grade mobile robotics project demonstrating sim-to-real architecture using **NVIDIA Isaac Sim 5.1** and **ROS 2 Jazzy**. The robot autonomously navigates a sequence of waypoints with closed-loop odometry feedback and reactively avoids LiDAR-detected obstacles along its path. The control stack is fully decoupled from the simulator — the same code would drive a real Jetbot without modification.

[![Isaac Sim Robot Demo](https://img.youtube.com/vi/C-wrNuz52O8/0.jpg)](https://youtu.be/C-wrNuz52O8)

---

## Architecture

```
┌─────────────────────┐                ┌─────────────────────┐
│  ROS 2 Controller   │   /cmd_vel     │  Isaac Sim          │
│  Node               │   ──────►      │  Standalone Script  │
│  (Python 3.12)      │                │  (Python 3.11)      │
│                     │   /odom        │                     │
│                     │   ◄──────      │                     │
│                     │   /scan        │                     │
│                     │   ◄──────      │                     │
└─────────────────────┘                └──────────┬──────────┘
                                                  │
                                            FastDDS bridge
                                                  │
                                                  ▼
                                            ┌──────────┐
                                            │  Jetbot  │
                                            │ + LiDAR  │
                                            └──────────┘
```

Two independent processes communicate via ROS 2 topics over FastDDS. This separation enables:

- **Sim-to-real transfer** — the controller is simulator-agnostic
- **Independent testing** — each side can be unit-tested without the other
- **Production patterns** — matches the architecture used by Figure, 1X, and Boston Dynamics

---

## Code Organization

Both sides follow the **"functional core, imperative shell"** pattern. Pure logic (math, state machine, perception) lives in standalone modules with no I/O dependencies; ROS 2 and Isaac Sim integration lives in thin shells that wrap the pure logic.

### Isaac Sim side (`isaac_sim/`)

```
isaac_sim/
├── simulation.py          # Entry point — main loop, ~80 lines
├── config.py              # Tunable parameters (wheels, sensors, obstacles)
├── kinematics.py          # Pure math (quaternions, differential drive)
├── robot_bridge.py        # ROS 2 node: subscribes /cmd_vel, publishes /odom
├── scene_setup.py         # World, ground plane, robot loading
├── sensors.py             # PhysX LiDAR + OmniGraph /scan publisher
├── obstacles.py           # Static cuboid obstacles for testing avoidance
└── run_simulation.sh      # Launch script with ROS 2 env vars
```

### ROS 2 controller (`ros2_ws/src/robot_controller/robot_controller/`)

```
robot_controller/
├── jetbot_controller.py   # Entry point — Node class, ROS 2 plumbing
├── config.py              # Gains, tolerances, waypoints, perception params
├── pose_utils.py          # Pure math (quaternion → yaw, angle wrap)
├── state_machine.py       # Pure state machine + obstacle avoidance modifier
└── perception.py          # Pure scan-processing functions
```

### Module dependency graph

```
        simulation.py  ◄──┐                  jetbot_controller.py  ◄──┐
              │           │                          │                │
              ▼           │                          ▼                │
        scene_setup.py    │                   state_machine.py        │
        sensors.py        │                          │                │
        obstacles.py      │                          ▼                │
              │           │                   perception.py           │
              ▼           │                   pose_utils.py           │
        robot_bridge.py   │                          │                │
              │           │                          ▼                │
              ▼           │                   config.py               │
        kinematics.py     │                                           │
              │           │                                           │
              ▼           │                                           │
        config.py ────────┘                                           │
                                                                      │
        Pure functions (no rclpy, no Isaac Sim) ◄─────────────────────┘
```

Higher-level modules depend on lower-level ones; never the reverse.

---

## Features

### Closed-loop autonomous waypoint navigation
- Configurable waypoint sequences (default: 2×2m square)
- State machine alternates between **ROTATE** (align to target) and **DRIVE** (move toward target)
- Proportional control on both heading and distance errors
- Real-time pose feedback from `/odom`
- Smooth motion with continuous heading correction while driving

### Reactive obstacle avoidance
- 2D LiDAR (PhysX-based) publishing 360° scans at 20 Hz on `/scan`
- Perception module analyzes the front cone for nearby obstacles
- "Slow down and curve away" avoidance strategy applied as a post-processing layer
- Pure functions for scan analysis — fully unit-testable

### Production-grade ROS 2 architecture
- ROS 2 Jazzy controller node publishing differential drive commands
- Standalone Isaac Sim script with embedded ROS 2 bridge
- Differential drive kinematics (`Twist` → wheel velocity conversion)
- 20 Hz control loop with deterministic behavior
- Industry-standard colcon workspace and Python package layout

### Senior-grade code quality
- Type hints on all public APIs
- Pure functions extracted from I/O code for unit-testability
- Type-safe state machine using Python enums and dataclasses
- Centralized configuration — no scattered magic numbers
- Consistent module dependency hierarchy (no circular imports)

---

## Tech Stack

- **Simulation:** NVIDIA Isaac Sim 5.1 (RTX 5090, driver 580 open kernel modules)
- **Middleware:** ROS 2 Jazzy
- **Language:** Python 3.11 (Isaac Sim) / Python 3.12 (ROS 2)
- **DDS:** FastDDS
- **Build:** colcon, ament_python
- **OS:** Ubuntu 24.04 LTS (kernel 6.17)

---

## Quick Start

### Prerequisites

- Ubuntu 24.04 LTS
- NVIDIA GPU with driver 580+ (open kernel modules required for RTX 50-series)
- NVIDIA Isaac Sim 5.1
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

The Jetbot will autonomously drive a 2×2m square: rotating to face each waypoint, then driving to it, while reactively avoiding obstacles in its path.

### Verify topics

```bash
ros2 topic list             # /cmd_vel, /odom, /scan should appear
ros2 topic echo /odom       # Real-time pose feedback
ros2 topic echo /scan       # 360° LiDAR readings (use --once to limit output)
ros2 topic echo /cmd_vel    # Velocity commands from the controller
```

---

## How It Works

### Differential Drive Kinematics

The controller publishes high-level `Twist` messages with linear and angular velocities. The Isaac Sim bridge converts these to individual wheel velocities using the inverse kinematics in `isaac_sim/kinematics.py`:

```
v_left  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
v_right = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
```

This matches the kinematics of any differential-drive robot (Jetbot, Turtlebot, Husky, etc).

### Closed-Loop Waypoint Following

The controller subscribes to `/odom` and delegates to a pure state machine (`robot_controller/state_machine.py`):

1. **ROTATING** — Compute `heading_error = atan2(dy, dx) - current_yaw`. Apply proportional angular velocity until `|heading_error| < 0.05 rad`.
2. **DRIVING** — Compute `distance_error = sqrt(dx² + dy²)`. Apply proportional linear velocity (with continued heading correction) until `distance_error < 0.1 m`.
3. Advance to next waypoint, repeat. Transition to **DONE** when all waypoints are reached.

The state machine is implemented as a pure function — `step(state, pose, target, gains) → (next_state, command, reached)` — making it directly unit-testable without ROS 2 or Isaac Sim.

### Reactive Obstacle Avoidance

LiDAR data is processed by `robot_controller/perception.py`, which exposes pure functions for analyzing the scan:

- `distance_to_nearest_obstacle_in_front()` — minimum range within the front cone
- `is_obstacle_too_close()` — boolean threshold check
- `clearance_left_vs_right()` — comparative space on either side of the forward direction

Avoidance is implemented as a **post-processing modifier** in `state_machine.py`:

```python
command = step(...)                                  # waypoint command
command = apply_obstacle_avoidance(command, info)    # avoidance overlay
```

When an obstacle is within the danger distance, the modifier scales linear velocity to 30% and adds an angular velocity toward the side with more clearance. This architecture keeps the waypoint state machine pure while making the avoidance strategy easily swappable.

### Python Interop Between Isaac Sim and ROS 2

Isaac Sim 5.1 ships Python 3.11; ROS 2 Jazzy targets Python 3.12. The Isaac Sim ROS 2 bridge ships its own Python 3.11-compatible `rclpy`, which is loaded via `LD_LIBRARY_PATH` and `PYTHONPATH` injection in `run_simulation.sh`. Inter-process communication happens at the FastDDS layer, so Python version mismatch doesn't matter at the wire level — the two processes only exchange serialized messages.

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

### Phase 3 — Architecture refactor & perception ✅
- [x] Refactor to modular architecture (functional core / imperative shell)
- [x] Extract pure logic into standalone, unit-testable modules
- [x] Type-safe state machine with enums and dataclasses
- [x] Centralized configuration modules
- [x] Add 2D LiDAR sensor publishing `/scan`
- [x] Static cuboid obstacles for testing
- [x] Reactive obstacle avoidance with slow-and-curve strategy

### Phase 4 — Autonomous Navigation
- [ ] Integrate Nav2 stack
- [ ] SLAM Toolbox for mapping
- [ ] Goal-based navigation with behavior trees

### Phase 5 — Production Engineering
- [ ] Unit tests for kinematics, perception, and state machine
- [ ] Dockerfile and docker-compose for reproducibility
- [ ] Integration tests with launch_testing
- [ ] GitHub Actions CI/CD pipeline
- [ ] Comprehensive technical documentation

---

## Engineering Notes

A detailed write-up of architecture decisions, sim-to-real considerations, the Python version interop solution, and the PhysX-vs-RTX LiDAR trade-off lives in [`docs/architecture.md`](docs/architecture.md).

---

## Author

**Vijay Yadav** — Robotics & AI Engineer

- LinkedIn: [vijay-yadav1995](https://www.linkedin.com/in/vijay-yadav1995)
- GitHub: [AstralGenius](https://github.com/AstralGenius)
- Portfolio: [astralgenius.com](https://astralgenius.com)

---

## Acknowledgments

The Jetbot model and assets are provided by NVIDIA Isaac Sim. The architectural patterns demonstrated here are inspired by production robotics deployments at companies including Figure, 1X, and Boston Dynamics AI Institute.

## License

Apache License 2.0
