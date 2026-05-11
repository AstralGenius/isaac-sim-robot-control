# Architecture

This document captures the design decisions behind the Isaac Sim Robot Control project. It explains *why* the codebase is structured the way it is, the trade-offs that were considered, and the constraints that shaped the architecture.

The intended audience is engineers reviewing the project (recruiters, collaborators, or future-me).

---

## 1. The Problem

Build a mobile robotics demonstrator that:

1. **Runs in simulation** to enable rapid iteration without hardware.
2. **Uses production patterns** so the same control logic could drive a real robot.
3. **Demonstrates senior-level engineering practices** suitable for humanoid robotics roles.

The temptation in robotics projects is to write a single Python script that opens the simulator, loads a robot, and applies control commands directly. That approach works for tutorials but produces code that is:

- Untestable (everything depends on the simulator running)
- Non-portable (logic is fused with simulator APIs)
- Hard to extend (adding a feature requires touching everything)

The architecture below explicitly rejects that pattern.

---

## 2. Design Principles

The codebase is structured around four principles, in priority order:

### 2.1 Sim-to-real readiness

The control logic must be **simulator-agnostic**. Specifically: the controller node does not import any Isaac Sim modules, does not know whether it is talking to a simulation or hardware, and would run unchanged against a real Jetbot publishing standard ROS 2 topics.

This principle drove the most important architectural decision: running the controller and the simulation as two independent processes that communicate only over ROS 2 topics.

### 2.2 Functional core, imperative shell

Pure functions (math, state machine logic) are kept in modules with no I/O dependencies. ROS 2 nodes and Isaac Sim integration code form thin "shells" that wrap the pure logic.

This is a pattern popularized by Gary Bernhardt — the imperative shell handles side effects (network, physics, time), the functional core computes outputs from inputs deterministically. The benefit is testability: pure logic can be exhaustively tested with no infrastructure, while the shell only needs smoke tests.

### 2.3 Single responsibility per module

Each Python module owns exactly one concern. `kinematics.py` is responsible for math; `robot_bridge.py` is responsible for ROS 2 I/O; `scene_setup.py` is responsible for Isaac Sim scene construction. When a requirement changes, only the relevant module changes.

### 2.4 Configuration over magic numbers

All tunable parameters live in dedicated `config.py` modules. Magic numbers scattered across business logic make codebases brittle. Centralizing parameters makes tuning safe and explicit.

---

## 3. Process Architecture

```
┌─────────────────────┐                ┌─────────────────────┐
│  ROS 2 Controller   │   /cmd_vel     │  Isaac Sim          │
│  Process            │   ──────►      │  Process            │
│  (Python 3.12)      │                │  (Python 3.11)      │
│                     │   /odom        │                     │
│                     │   ◄──────      │                     │
└─────────────────────┘                └─────────────────────┘
        ROS 2 Jazzy                     Isaac Sim 5.0 with
        system install                  bundled ROS 2 bridge
              │                                  │
              └─────── FastDDS ──────────────────┘
                         │
                     network IPC
```

Two operating system processes, two different Python interpreters, communicating only through serialized ROS 2 messages over FastDDS. There is no shared memory, no shared imports, no direct function calls between the processes.

### 3.1 Why two processes?

The obvious alternative is to embed the controller logic inside the Isaac Sim script. That would be simpler — one process, one terminal, no IPC complexity.

It would also defeat the entire purpose. The whole point is that the controller doesn't depend on Isaac Sim. If swapping in a real robot required even one line of code change in the controller, the abstraction would be a lie.

Production robotics teams (Figure, 1X, Boston Dynamics) run their control stacks as separate ROS 2 nodes for exactly this reason. The simulator is just one of many possible peers.

### 3.2 Trade-offs accepted

- **Higher startup complexity**: two terminals, two source commands. Mitigated by `run_simulation.sh` handling the simulation side; a future launch file will unify them.
- **Network latency**: FastDDS within a single host adds sub-millisecond overhead, well below physics step time. Not a real concern.
- **Debugging**: harder to step through two processes simultaneously. Each side has independent logging, which suffices for most issues.

---

## 4. The Python Version Interop Problem

This is the most interesting engineering problem the project solves.

### 4.1 The constraint

- **Isaac Sim 5.0** ships its own Python interpreter at version 3.11. Its `rclpy` is a Python 3.11 build distributed inside the Isaac Sim install.
- **ROS 2 Jazzy** targets Ubuntu 24.04, which ships Python 3.12. Its `rclpy` is a Python 3.12 build.
- Importing the wrong `rclpy` inside the Isaac Sim process fails with a cryptic C extension load error, because Python C extensions are not ABI-compatible across versions.

A common but bad workaround is to install Python 3.11 system-wide and rebuild ROS 2 from source. This breaks future system updates and is not portable.

### 4.2 The solution

The two processes are kept fully separate. Each uses the `rclpy` that matches its Python version:

- The Isaac Sim process uses Isaac Sim's bundled Python 3.11 `rclpy`, loaded by injecting `LD_LIBRARY_PATH` and `PYTHONPATH` in `run_simulation.sh`.
- The controller process uses the standard Ubuntu Python 3.12 `rclpy` from `/opt/ros/jazzy`.

The two `rclpy` builds never live in the same process. They communicate at the wire level, where FastDDS only sees serialized bytes — Python versions are irrelevant.

This is a real-world example of the value of process separation. If the controller and simulator were one process, this problem would have no clean solution.

---

## 5. Module Architecture

### 5.1 Isaac Sim side (`isaac_sim/`)

```
simulation.py        — entry point, main loop (~80 lines)
  ↓
scene_setup.py       — world, ground plane, robot loading
  ↓
robot_bridge.py      — ROS 2 node: /cmd_vel subscribe, /odom publish
  ↓
kinematics.py        — pure math (quaternions, differential drive)
  ↓
config.py            — tunable parameters
```

Each arrow represents an import dependency. The hierarchy is strict: higher modules depend on lower modules, never the reverse. This produces a directed acyclic dependency graph.

### 5.2 ROS 2 controller side (`ros2_ws/src/robot_controller/`)

```
jetbot_controller.py — entry point, Node class, ROS 2 plumbing
  ↓
state_machine.py     — pure state logic (enum, dataclasses, step function)
  ↓
pose_utils.py        — pure math (quaternion → yaw, angle normalization)
  ↓
config.py            — gains, tolerances, default waypoints
```

The same structure mirrored on the controller side. Symmetry between the two sides is intentional — it reduces cognitive load when switching contexts.

### 5.3 The pure layer

The bottom two layers on each side (`kinematics.py`, `pose_utils.py`, both `config.py` files, and `state_machine.py`) have **zero dependencies on rclpy, Isaac Sim, or any I/O library**. They can be imported, called, and tested in a plain Python REPL.

This is the "functional core." It contains the interesting decisions of the system — how to compute wheel velocities, when to transition between states, how to wrap angles — without any infrastructure baggage.

---

## 6. State Machine Design

The waypoint-following logic is implemented as a pure state machine in `state_machine.py`.

### 6.1 States

A type-safe `ControllerState` enum:

- **ROTATING** — rotating in place to align heading with the current waypoint
- **DRIVING** — driving forward toward the current waypoint
- **DONE** — all waypoints reached; robot stops

Using an enum instead of strings catches typos at import time and enables IDE autocomplete.

### 6.2 The step function

```python
def step(state, current_pose, target, gains) -> (next_state, velocity_command, reached):
    ...
```

A pure function that takes the current state, robot pose, target waypoint, and control gains, and returns the next state plus a velocity command. No side effects. No I/O. Fully deterministic given the inputs.

This signature has a useful property: it can be tested exhaustively without any infrastructure. A unit test reads:

```python
new_state, command, reached = step(
    ControllerState.ROTATING,
    current_x=0, current_y=0, current_yaw=0,
    target_x=1, target_y=0,
    gains=test_gains,
)
assert command.angular_z == 0.0  # already facing target
assert new_state == ControllerState.DRIVING
```

That test runs in a millisecond. No Isaac Sim, no ROS 2, no network.

### 6.3 Control law

For each waypoint, simple proportional control on two errors:

- **Heading error** drives angular velocity when in ROTATING, with continued correction during DRIVING
- **Distance error** drives linear velocity when in DRIVING

Output is clamped to configured velocity limits. The gains were tuned empirically to the Jetbot's mass and wheel friction.

This is not sophisticated control — it overshoots slightly, has no integral action, doesn't anticipate corners. A real production system would use Pure Pursuit, MPC, or a Stanley controller. But the architecture supports swapping the control law without touching the surrounding code: replace `step()`, leave everything else alone.

---

## 7. Differential Drive Kinematics

`isaac_sim/kinematics.py` contains the inverse kinematics for a differential-drive robot:

```
v_left  = (linear.x - angular.z × wheel_separation / 2) / wheel_radius
v_right = (linear.x + angular.z × wheel_separation / 2) / wheel_radius
```

This formula maps a body-frame Twist command (linear velocity in the forward direction plus angular velocity around the vertical axis) to individual wheel velocities. It works for any robot with two parallel driven wheels — Jetbot, Turtlebot, Husky.

Putting this in a pure function (rather than embedding it in the bridge node) means:

- It's unit-testable independently
- It's reusable for any future differential-drive simulation
- The bridge node stays focused on ROS 2 plumbing

---

## 8. Decisions Considered and Rejected

### 8.1 OmniGraph for ROS 2 publishing

Isaac Sim supports authoring sensor publishing pipelines visually as OmniGraph nodes. We considered using OmniGraph instead of a Python ROS 2 node for /odom.

Rejected because:
- OmniGraph is Isaac Sim specific; the same pattern wouldn't transfer to a real robot
- Python provides more flexibility for custom message formats
- Debugging OmniGraph is harder than Python

OmniGraph will be revisited for the LiDAR publishing in Phase 3, where the C++ path provides genuine performance benefits and the ROS 2 message format is standard (`LaserScan`).

### 8.2 Embedded controller inside simulation.py

We considered putting the control logic inside the Isaac Sim script as a Python class, avoiding the second process entirely.

Rejected because:
- Defeats the sim-to-real principle
- Tightly couples control logic to Isaac Sim's lifecycle
- Makes it impossible to test the controller without launching the simulator

### 8.3 YAML-based configuration

We considered loading parameters from YAML files at startup.

Rejected for now because:
- Python module config is simpler and adequate for current scope
- ROS 2 parameter server is the standard way to expose runtime-tunable parameters
- Will migrate to ROS 2 parameters when Nav2 integration arrives in Phase 4

### 8.4 ROS 1

ROS 1 is end-of-life and not supported by Isaac Sim 5.0's bridge. Not seriously considered.

---

## 9. Future Architecture

### Phase 3: Perception

- RTX LiDAR added to Isaac Sim via OmniGraph (the performance benefit outweighs the architectural cost for sensor publishing)
- New ROS 2 topic `/scan` (sensor_msgs/LaserScan) published from Isaac Sim
- Controller gains a `scan_subscription` and a perception module that reasons about distance to nearest obstacles
- State machine extended with an AVOIDING state, or perception integrated as a velocity-shaping layer on top of the existing state machine

### Phase 4: Nav2 integration

- Replace hand-rolled waypoint logic with Nav2 BehaviorTree
- Add `nav2_bringup` launch files
- Robot publishes a TF tree (`odom` → `base_link` → sensors)
- SLAM Toolbox builds a map from `/scan` data

This will require restructuring the controller from a single node into a launch graph, but the principles of the existing architecture (pure logic separated from ROS 2 plumbing) carry over.

### Phase 5: Production readiness

- Unit tests for `kinematics.py`, `pose_utils.py`, `state_machine.py` (the pure layer)
- Integration tests using `launch_testing`
- Dockerfile for reproducible builds
- GitHub Actions CI/CD running build + test on every push

---

## 10. What I Learned

A few things became concrete while building this project:

1. **Process separation is a powerful abstraction.** It forced a clean interface (ROS 2 topics) between simulator and controller. Without it, the Python version interop problem would have been intractable.

2. **Pure functions pay back the extraction cost immediately.** Once `kinematics.py` and `state_machine.py` existed as pure modules, debugging became dramatically easier — I could test math hypotheses in a REPL instead of restarting the entire stack.

3. **Centralized configuration is non-negotiable above a certain code size.** The 200-line version of `simulation.py` had constants scattered throughout. After three rounds of tuning, I couldn't remember where a number lived. The `config.py` modules eliminated this entire class of problem.

4. **Documentation written immediately after refactoring is much better than documentation written later.** The "why" of every decision is fresh; weeks later, those reasons fade.

---

## Author's note

This project is part of a 30-day portfolio sprint targeting humanoid robotics roles. It is intentionally over-engineered for its size to demonstrate patterns that scale. A 200-line single-file version would do the same thing functionally; what's different here is that adding LiDAR, Nav2, or a new robot won't require restructuring anything.
