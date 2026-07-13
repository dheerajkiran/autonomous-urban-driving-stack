# Autonomous Urban Driving Stack

A city-scale autonomous driving simulation built on ROS2. The vision is simple — a car robot that plans its own route and drives through real traffic on the actual streets of Tempe, AZ, using real OpenStreetMap road data and SUMO for traffic simulation.

---

## What it does

- Downloads the real road network of **Tempe, AZ** from OpenStreetMap
- Plans an **optimal driving route** between any two points using A*
- Displays the planned route on an interactive **map viewer** with click-to-drive pin placement
- Runs the **SUMO traffic simulator** on the real Tempe road network
- All modules communicate through **ROS2 topics** with zero direct coupling

---

## Architecture

```
Click-to-Drive (S pin → E pin in viewer)
               │
               ▼
┌──────────────────────────────────────────┐
│              Map Layer                   │
│  map_loader    →  OSM download + SUMO net│
│  route_planner →  A* on Tempe road graph │
└──────────────────────┬───────────────────┘
                       │ /navigation/route
                       ▼
┌──────────────────────────────────────────┐
│           Simulation Layer               │
│  sumo_bridge   →  SUMO process + tick   │
│  pygame_viewer →  OSM map + route UI    │
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
| Visualization | Pygame OSM viewer |
| OS | Ubuntu 22.04 LTS (aarch64) |

---

## Repository Structure

```
.
├── launch/
│   └── ads_full.launch.py          # Starts the full stack
├── src/
│   ├── ads_interfaces/             # ROS2 message definitions
│   │   └── msg/
│   │       ├── Waypoint.msg
│   │       └── Route.msg
│   ├── ads_map/                    # OSM loading and A* route planning
│   └── ads_simulation/             # SUMO bridge and map viewer
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
pip install osmnx "numpy<2" scikit-learn traci==1.12.0 sumolib==1.12.0 pygame
```

### Build

```bash
cd ~/ads_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# Terminal 1 — map loader
ros2 run ads_map map_loader

# Terminal 2 — route planner
ros2 run ads_map route_planner

# Terminal 3 — simulation + viewer
ros2 launch ads_simulation simulation.launch.py use_gui:=false
```

Once running, click an S pin and E pin on the map viewer to plan a route.

---

## Design Principles

- **Loose coupling** — every module communicates exclusively through ROS2 topics
- **Single responsibility** — each node has one clearly defined job
- **Real data** — routing and road geometry use actual OpenStreetMap data for Tempe, AZ

---

## License

MIT
