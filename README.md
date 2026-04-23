# Isaac Sim Mobile Robot Control

This project demonstrates a basic mobile robot control system built using NVIDIA Isaac Sim.
It focuses on low-level articulation control and structured behaviour in a simulated robotic environment.

---

## Overview

The robot is controlled using an articulation controller, enabling direct joint-level velocity commands.
A simple state-based control loop is implemented to produce deterministic behaviour, including forward motion and timed turning.

This project represents the first step toward building autonomous robotic systems by establishing a working control pipeline in simulation.

---

## Features

* Mobile robot simulation in NVIDIA Isaac Sim
* Articulation-based wheel velocity control
* Deterministic motion behaviour (forward + turn loop)
* Real-time control using physics callbacks
* Integration with Isaac Sim simulation environment

---

## Tech Stack

* Python
* NVIDIA Isaac Sim
* Articulation Controller

---

## Demo

[![Isaac Sim Robot Demo](https://img.youtube.com/vi/7z_VH4hv9Sw/0.jpg)](https://youtu.be/7z_VH4hv9Sw)

---

## Project Structure

```
isaac-sim-robot-control/
│
├── scripts/
│   └── hello_robot.py        # main control script
│
├── media/
│   └── demo.mkv              # simulation recording
│
├── docs/                     # optional notes / ideas
│
├── README.md
├── .gitignore
```

---

## How to Run

This project is designed to run inside NVIDIA Isaac Sim.

### Requirements

* NVIDIA Isaac Sim installed and configured
* Compatible system with GPU support

### Steps

1. Launch NVIDIA Isaac Sim
2. Ensure Isaac Sim assets are properly loaded (Nucleus server connected)
3. Navigate to the interactive examples environment
4. Run the script:

   ```
   hello_robot.py
   ```
5. Start simulation (Play) and observe robot behaviour

### Notes

* This project extends the "Hello World" example from Isaac Sim
* It assumes a working Isaac Sim installation
* It is not a standalone Python application

---

## Current Status

* Phase 1: Basic motion control (completed)
* Phase 2: Sensor integration (planned)
* Phase 3: Reactive obstacle avoidance (planned)

---

## Future Work

* Add camera or LiDAR sensor
* Implement obstacle detection
* Develop reactive navigation system
* Integrate learning-based perception models
* Improve control strategies and system robustness

---

## Key Learning Outcomes

* Understanding of articulation-based robot control
* Experience with real-time simulation pipelines
* Integration of control logic within a physics-based environment
* Transition from random motion to structured robotic behaviour

---

## Acknowledgment

This project is based on and extends the Hello World example provided in NVIDIA Isaac Sim, with additional control logic and behaviour implementation.

---

## Author

Vijay Yadav
Robotics & AI Engineer

LinkedIn: https://www.linkedin.com/in/vijay-yadav1995
GitHub: https://github.com/AstralGenius
