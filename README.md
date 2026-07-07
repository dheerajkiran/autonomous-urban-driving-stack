# Autonomous Urban Driving Stack

A modular, city-scale autonomous driving simulation built on ROS2. The system
navigates a real road network (Tempe, AZ) using OpenStreetMap data and SUMO
traffic simulation — routing from any address to any destination through live
multi-vehicle traffic.

---

## What it does

- Downloads the real road network of **Tempe, AZ** from OpenStreetMap
- Plans an **optimal driving route** from any start address to any destination using A*
- Simulates **60+ background vehicles** on real Tempe streets using SUMO
- The ego vehicle navigates the route, responds to traffic, and executes **lane changes**
- Every decision is narrated in **plain English** by the semantic reasoning layer
- All modules communicate through **ROS2 topics** with zero direct coupling

---

## Architecture

```
Terminal Input (start address → destination)
               │
               ▼
┌──────────────────────────────────────────┐
│              Map Layer                   │
│  map_loader   →  OSM download + SUMO net │
│  route_planner →  A* on Tempe road graph │
│  waypoint_publisher → route progress     │
└──────────────────────┬───────────────────┘
                       │ /navigation/route
                       ▼
┌──────────────────────────────────────────┐
│           Simulation Layer               │
│  sumo_bridge  →  SUMO ↔ ROS2 bridge     │
│  traffic_spawner → 60 background cars    │
│  mission_input → terminal address prompt │
└────────┬─────────────────┬──────────────┘
         │ /vehicle/state  │ /perception/traffic_vehicles
         ▼                 ▼
┌──────────────────────────────────────────┐
│           Behavior Planner               │  ← Phase 4
│   FSM: Cruise | SlowDown | LaneChange    │
│        Stop | Reroute | EmergencyBrake   │
└──────────────────────┬───────────────────┘
                       │ /vehicle/command
                       ▼
┌──────────────────────────────────────────┐
│              Controller                  │  ← Phase 5
│   Waypoint following + lane change exec  │
└──────────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────┐
│         Semantic Reasoning               │  ← Phase 7
│  "Slower vehicle ahead. Lane change      │
│   to right. Resuming cruise at 11 m/s." │
└──────────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────┐
│         Evaluation Dashboard             │  ← Phase 8
│  Route completion | Decisions | Latency  │
└──────────────────────────────────────────┘
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS2 Humble |
| Language | Python 3.10+ |
| Map Data | OpenStreetMap via `osmnx` |
| Traffic Simulation | SUMO (Simulation of Urban Mobility) |
| Routing | NetworkX A* on OSM road graph |
| Visualization | SUMO GUI + Pygame (Phase 8) |
| Semantic Reasoning | Rule-based + LLM narration (Phase 7) |
| CI/CD | GitHub Actions (Phase 9) |
| Containerization | Docker (Phase 9) |
| OS | Ubuntu 22.04 LTS (aarch64) |

---

## Repository Structure

```
.
├── launch/
│   └── ads_full.launch.py          # Single command — starts entire stack
├── src/
│   ├── ads_interfaces/             # All ROS2 message definitions
│   │   └── msg/
│   │       ├── VehicleState.msg
│   │       ├── VehicleCommand.msg
│   │       ├── Waypoint.msg
│   │       ├── Route.msg
│   │       ├── TrafficVehicle.msg
│   │       └── TrafficVehicleArray.msg
│   ├── ads_vehicle_state/          # Kinematic bicycle model + standalone test nodes
│   ├── ads_map/                    # OSM loading, A* routing, waypoint tracking
│   ├── ads_simulation/             # SUMO bridge, traffic spawner, mission input
│   ├── ads_behavior/               # Behavior planner FSM (Phase 4)
│   ├── ads_controller/             # Trajectory + lane-change controller (Phase 5)
│   ├── ads_semantic/               # Scene reasoning + English narration (Phase 7)
│   └── ads_evaluation/             # Metrics collection and export (Phase 8)
```

---

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- SUMO traffic simulator
- Python dependencies

```bash
# SUMO
sudo apt install -y sumo sumo-tools sumo-gui

# Python dependencies
pip install osmnx numpy
```

### Build

```bash
cd ~/ads_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run — Full Stack

```bash
ros2 launch launch/ads_full.launch.py
```

You will be prompted in the terminal:

```
  Start address  : Arizona State University, Tempe, AZ
  Destination    : Tempe Marketplace, Tempe, AZ
```

The SUMO GUI opens showing the Tempe road network. The ego vehicle (cyan)
navigates through live traffic from your start to destination.

### Run — Vehicle State Layer Only (no SUMO)

```bash
ros2 launch ads_vehicle_state vehicle_state.launch.py
```

Runs the standalone kinematic bicycle model with the scripted 6-step mission
(cruise → turn → decelerate → emergency stop → park). Useful for testing
behavior planner and controller logic without the full simulation.

---

## Development Roadmap

| Phase | Module | Status |
|---|---|---|
| 0 | Environment Setup | ✅ Complete |
| 1 | ROS2 Core Skeleton — Vehicle State Layer | ✅ Complete |
| 2 | Vehicle Dynamics + Command Interface | ✅ Complete |
| 3 | Map Loading, SUMO Integration, City-Scale Traffic | ✅ Complete |
| 4 | Behavior Planner (FSM) + Traffic-Aware Decisions | 🔧 In Progress |
| 5 | Controller — Waypoint Following + Lane Changes | ⬜ Planned |
| 6 | Full Traffic Scenarios + Dynamic Rerouting | ⬜ Planned |
| 7 | Semantic Reasoning — English Decision Narration | ⬜ Planned |
| 8 | Evaluation Dashboard + Waymo-Style Visualization | ⬜ Planned |
| 9 | Documentation, Docker, GitHub Actions | ⬜ Planned |

---

## Design Principles

- **Loose coupling** — every module communicates exclusively through ROS2 topics.
- **Single responsibility** — each node has one clearly defined job.
- **Real data** — routing and road geometry use actual OpenStreetMap data for Tempe, AZ.
- **Testability** — nodes run in isolation; `ads_vehicle_state` works without SUMO.
- **Observability** — structured logging at every layer; semantic layer narrates decisions.
- **Extensibility** — swapping the planner or controller requires only reconnecting topics.

---

## License

MIT
