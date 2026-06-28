# Semantic Autonomous Driving Stack

A modular, ROS2-based autonomous driving simulation stack built to industry standards.
The system integrates independent perception, planning, control, and semantic reasoning
modules into a cohesive autonomy pipeline.

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Vehicle State Layer                │  ← Phase 1 (current)
│         ads_vehicle_state / VehicleState.msg        │
└──────────────────────┬──────────────────────────────┘
                       │ /vehicle/state
┌──────────────────────▼──────────────────────────────┐
│                  Perception Layer                   │  ← Phase 3
│      Camera → YOLOv8 → DetectedObjects.msg         │
└──────────────────────┬──────────────────────────────┘
                       │ /perception/detected_objects
┌──────────────────────▼──────────────────────────────┐
│            Semantic Scene Understanding             │  ← Phase 7
│        Natural-language environment reasoning       │
└──────────────────────┬──────────────────────────────┘
                       │ /semantics/scene_description
┌──────────────────────▼──────────────────────────────┐
│                 Behavior Planner                    │  ← Phase 4
│    FSM: Cruise | SlowDown | Stop | EmergencyBrake  │
└──────────────────────┬──────────────────────────────┘
                       │ /planning/behavior_command
┌──────────────────────▼──────────────────────────────┐
│               Trajectory Planner                   │  ← Phase 5
└──────────────────────┬──────────────────────────────┘
                       │ /planning/trajectory
┌──────────────────────▼──────────────────────────────┐
│                   Controller                        │  ← Phase 5
│              PID / Pure Pursuit / MPC              │
└──────────────────────┬──────────────────────────────┘
                       │ /vehicle/control_command
┌──────────────────────▼──────────────────────────────┐
│                   Simulation                        │  ← Phase 6
│                 CARLA / Lightweight                 │
└─────────────────────────────────────────────────────┘
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS2 Humble |
| Language | Python 3.10+ (C++ for performance-critical nodes) |
| Perception | YOLOv8 (Ultralytics), OpenCV |
| ML Framework | PyTorch |
| Simulation | CARLA (Phase 6) |
| Visualization | RViz2 |
| CI/CD | GitHub Actions (Phase 9) |
| Containerization | Docker (Phase 9) |
| OS | Ubuntu 22.04 LTS |

---

## Repository Structure

```
.
├── src/
│   ├── ads_interfaces/          # Custom ROS2 message/service/action definitions
│   ├── ads_vehicle_state/       # Vehicle state simulation and monitoring
│   ├── ads_perception/          # Camera interface + YOLOv8 detection (Phase 3)
│   ├── ads_behavior_planner/    # FSM-based behavior planning (Phase 4)
│   ├── ads_trajectory_planner/  # Trajectory generation (Phase 5)
│   ├── ads_controller/          # PID / Pure Pursuit / Stanley controller (Phase 5)
│   ├── ads_semantic_reasoning/  # Scene language model (Phase 7)
│   └── ads_evaluation/          # Metrics collection and export (Phase 8)
└── docs/
    └── architecture.md
```

---

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

### Build

```bash
cd ~/path/to/autonomous-driving-stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run — Phase 1: Vehicle State Layer

```bash
ros2 launch ads_vehicle_state vehicle_state.launch.py
```

To enable debug logging:
```bash
ros2 launch ads_vehicle_state vehicle_state.launch.py log_level:=debug
```

Inspect the live topic:
```bash
ros2 topic echo /vehicle/state
ros2 topic hz /vehicle/state      # should report ~20 Hz
```

---

## Development Roadmap

| Phase | Module | Status |
|---|---|---|
| 0 | Environment Setup | ✅ Complete |
| 1 | ROS2 Core Skeleton + Vehicle State | ✅ Complete |
| 2 | Vehicle Dynamics + Control Interface | 🔜 Next |
| 3 | Perception (YOLOv8) | ⬜ Planned |
| 4 | Behavior Planner (FSM) | ⬜ Planned |
| 5 | Controller (PID / Pure Pursuit) | ⬜ Planned |
| 6 | Simulation (CARLA) | ⬜ Planned |
| 7 | Semantic Reasoning | ⬜ Planned |
| 8 | Evaluation Dashboard | ⬜ Planned |
| 9 | Documentation + CI/CD | ⬜ Planned |

---

## Design Principles

- **Loose coupling** — every module communicates exclusively through ROS2 topics, services, or actions.
- **Single responsibility** — each node has one clear job.
- **Testability** — nodes are designed to run in isolation with mock inputs.
- **Observability** — structured logging at every layer; evaluation module tracks key metrics.
- **Extensibility** — adding a new planner or controller requires only implementing a new node and connecting it to existing topics.

---

## License

MIT
