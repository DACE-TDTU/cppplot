# CppPlot Robotics Workflow

## Goal

Build a small robotics module on top of the existing CppPlot plotting and control stack. The first usable target is 2D mobile robotics: differential-drive kinematics, trajectory following, simulation, and SVG visualization.

## Scope

This module is not intended to replace ROS 2, Gazebo, Drake, or Pinocchio. It focuses on educational and lightweight engineering workflows:

- model simple robots in native C++;
- simulate closed-loop behavior;
- reuse CppPlot for trajectory and error visualization;
- reuse `cppplot::control` for LQR, MPC, Kalman, and CBF in later milestones.

## Milestones

### M1: Mobile Robot Foundation

Deliverables:

- `include/cppplot/robotics/types.hpp`
- `include/cppplot/robotics/mobile_base.hpp`
- `include/cppplot/robotics/simulation.hpp`
- `include/cppplot/robotics/robotics.hpp`
- `tests/test_robotics_mobile_base.cpp`

Acceptance criteria:

- forward and inverse differential-drive kinematics are mutually consistent;
- equal wheel speeds produce straight-line motion;
- opposite wheel speeds produce in-place rotation;
- pose integration normalizes heading.

### M2: Path Following

Deliverables:

- `include/cppplot/robotics/trajectory.hpp`
- `include/cppplot/robotics/controllers.hpp`
- pure pursuit controller;
- path helpers for nearest and lookahead points.

Acceptance criteria:

- a simulated robot can follow a line or circle path;
- tracking history is returned as structured data;
- basic controller limits are respected.

### M3: Visualization

Deliverables:

- `include/cppplot/robotics/visualization.hpp`
- `examples/robotics_diffdrive_demo.cpp`

Acceptance criteria:

- generated SVG shows reference path, robot trajectory, and final robot pose;
- generated SVG includes tracking error plots.

### M4: Control Integration

Deliverables:

- LQR tracking wrapper around existing `cppplot::control::care/lqr` utilities;
- PID wheel velocity controller;
- MPC tracking example using `cppplot::control::MPCController`.

Acceptance criteria:

- LQR example reduces lateral and heading error;
- PID example tracks wheel speed commands;
- MPC example returns bounded control inputs.

### M5: Safety

Deliverables:

- circular and rectangular safe-set helpers;
- CBF safety filter adapters for simple mobile-base controls;
- obstacle-avoidance demo.

Acceptance criteria:

- robot trajectory remains outside forbidden circular zones in demo cases;
- CBF activation history can be plotted.

### M6: Planar Manipulator

Deliverables:

- 2-link planar arm forward kinematics;
- Jacobian;
- iterative inverse kinematics;
- arm visualization demo.

Acceptance criteria:

- forward kinematics matches analytic expected values;
- inverse kinematics converges on reachable points;
- demo plots arm configurations and end-effector trace.

## Current Implementation Status

- [x] M1 core headers started.
- [x] M2 pure pursuit/path helpers started.
- [x] M3 basic visualization helper started.
- [x] First robotics example added.
- [x] First robotics unit test added.
- [ ] LQR/MPC robotics wrappers.
- [ ] CBF safety adapter.
- [ ] Planar manipulator module.

## Development Rules

- Keep the module header-only, matching the existing CppPlot style.
- Prefer small plain structs over heavy abstractions.
- Keep simulation deterministic and dependency-free.
- Use `std::vector` and the existing `cppplot::Matrix` only where useful.
- Add tests for numeric contracts before adding larger demos.
- Keep examples readable enough to double as documentation.
