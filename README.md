# Autonomous Urban Driving Stack

A ROS2 driving simulation on real city streets: click a start and end pin on a live map of Tempe, AZ, and an ego vehicle plans a route and drives itself there through the actual OpenStreetMap road network, simulated in SUMO.

---

## What it does

- **Click-to-drive** — place S/E pins on an interactive OpenStreetMap viewer; press SPACE and the ego vehicle spawns on the correct road and drives to the destination
- Downloads the real road network of **Tempe, AZ** from OpenStreetMap
- Plans an **optimal driving route** between any two points using A*
- Resolves the ego's spawn/destination edges and the path between them directly from **SUMO's own network graph** (`sumolib`), independent of the OSM route shown in the viewer
- Runs the **SUMO traffic simulator** headless and steps it in real time from ROS2
- All modules communicate through **ROS2 topics** with zero direct coupling

---

## Architecture

```
Click-to-Drive (S pin → E pin in viewer, SPACE to confirm)
               │
               ▼
┌──────────────────────────────────────────┐
│              Map Layer                    │
│  map_loader    →  OSM download + SUMO net │
│  route_planner →  A* on Tempe road graph  │
└──────────────────────┬────────────────────┘
                       │ /navigation/route (display)
                       │ /navigation/latlon_goal, /navigation/mission_confirm
                       ▼
┌──────────────────────────────────────────┐
│           Simulation Layer                │
│  sumo_bridge   →  SUMO process + ego spawn│
│                    + /vehicle/state        │
│  pygame_viewer →  OSM map + route UI      │
└──────────────────────────────────────────┘
```

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS2 Humble |
| Language | Python 3.10+ |
| Map Data | OpenStreetMap via `osmnx` + direct Overpass API queries |
| Traffic Simulation | SUMO (Simulation of Urban Mobility), via `traci` + `sumolib` |
| Routing (display) | NetworkX A* on OSM road graph |
| Routing (ego) | `sumolib` shortest-path on the live SUMO network |
| Visualization | Pygame OSM viewer |
| OS | Ubuntu 22.04 LTS (aarch64) |

---

## Repository Structure

```
.
├── launch/
│   └── ads_full.launch.py          # Starts the full stack (map + simulation)
├── src/
│   ├── ads_interfaces/              # ROS2 message definitions
│   │   └── msg/
│   │       ├── Route.msg
│   │       ├── Waypoint.msg
│   │       ├── VehicleState.msg
│   │       ├── VehicleCommand.msg
│   │       ├── TrafficVehicle.msg
│   │       └── TrafficVehicleArray.msg
│   ├── ads_map/                     # OSM loading and A* route planning
│   └── ads_simulation/              # SUMO bridge, ego spawning, and map viewer
```

---

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- SUMO traffic simulator
- Python dependencies

```bash
# SUMO (sumo-gui is optional — not available on ARM64; headless "sumo" is all this project needs)
sudo apt install -y sumo sumo-tools

# Python dependencies
pip install osmnx "numpy<2" scikit-learn traci==1.12.0 sumolib==1.12.0 pygame requests websockets
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
ros2 launch launch/ads_full.launch.py
```

This brings up the whole stack — map download/caching, A* route planner, SUMO bridge, and the Pygame viewer. On first run it downloads and converts the Tempe road network (~1-2 minutes); subsequent runs use the cache and start in seconds.

Once the viewer window opens: **left-click** to place the start pin, **right-click** to place the destination pin, then press **SPACE** to spawn the ego vehicle and watch it drive.

To run layers independently (e.g. for debugging one node at a time):

```bash
ros2 launch ads_map map.launch.py
ros2 launch ads_simulation simulation.launch.py use_gui:=false
```

### Optional: 3D viewer

`car3d_bridge` streams the ego's live position/heading and the road network geometry over a plain WebSocket, for a lightweight browser-based 3D view alongside the 2D Pygame viewer. Run it alongside the main stack:

```bash
ros2 run ads_simulation car3d_bridge
```

Then open `src/ads_simulation/web/car3d_viewer.html` directly in a browser (works over `file://`, no server needed) and connect to the bridge's `ws://` address shown in the page.

---

## Engineering Notes

The SUMO network is built from a direct Overpass API fetch rather than `osmnx`'s own OSM-XML export — `osmnx.save_graph_xml()` reconstructs a synthetic OSM file that doesn't reliably preserve node IDs shared between ways at intersections, which `netconvert` needs to build real junction connectivity. Feeding it osmnx-reconstructed XML fragmented the network into thousands of disconnected islands; fetching genuine Overpass XML for the same area fixed it.

Ego routing resolves lat/lon clicks to SUMO edges via `sumolib`, trying several nearby candidate edges per pin (not just the closest) and falling back through `getShortestPath()` combinations — the geometrically closest edge to a click is sometimes a disconnected driveway or turnaround stub even on a well-formed network.

---

## Design Principles

- **Loose coupling** — every module communicates exclusively through ROS2 topics
- **Single responsibility** — each node has one clearly defined job
- **Real data** — routing and road geometry use actual OpenStreetMap data for Tempe, AZ

---

## License

MIT — see [LICENSE](LICENSE).
