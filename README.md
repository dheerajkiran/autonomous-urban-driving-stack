# Autonomous Urban Driving Stack

A ROS2 driving simulation on real city streets: click a start and end pin on a live map of Tempe, AZ, and an ego vehicle plans a route and drives itself there through the actual OpenStreetMap road network, simulated in SUMO.

---

## What it does

- **Click-to-drive** — place S/E pins on an interactive OpenStreetMap viewer; press SPACE and the ego vehicle spawns on the correct road and drives to the destination
- Downloads the real road network of **Tempe, AZ** from OpenStreetMap
- Plans an **optimal driving route** between any two points using A*
- Resolves the ego's spawn/destination edges and the path between them directly from **SUMO's own network graph** (`sumolib`), independent of the OSM route shown in the viewer
- Runs the **SUMO traffic simulator** headless and steps it in real time from ROS2
- **Live browser-based 3D viewer** — lane-accurate two-way roads (real lane counts/widths, not a single centerline), nearby buildings, and background traffic driving the opposite direction, all rendered live as the ego drives
- All modules communicate through **ROS2 topics** with zero direct coupling

---

## Architecture

```
Click-to-Drive (S pin -> E pin in viewer, SPACE to confirm)
               |
               v
+--------------------------------------------+
|               Map Layer                     |
|  map_loader    -> OSM download + SUMO net   |
|  route_planner -> A* on Tempe road graph     |
+----------------------+-----------------------+
                       | /navigation/route (display)
                       | /navigation/latlon_goal, /navigation/mission_confirm
                       v
+--------------------------------------------+
|             Simulation Layer                 |
|  sumo_bridge   -> SUMO process; ego +        |
|                   background-traffic spawn;  |
|                   lane/building geometry;     |
|                   /vehicle/state              |
|  pygame_viewer -> OSM map + route UI          |
+----------------------+-----------------------+
                       | /navigation/route_lanes
                       | /navigation/route_buildings
                       | /navigation/traffic_vehicles
                       | /vehicle/state
                       v
+--------------------------------------------+
|        3D Viewer (optional, browser)         |
|  car3d_bridge      -> WebSocket relay        |
|  car3d_viewer.html -> Three.js scene:        |
|    lane-accurate roads, buildings,           |
|    ego + oncoming background traffic         |
+--------------------------------------------+
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
| Visualization (2D) | Pygame OSM viewer |
| Visualization (3D) | Three.js over a plain WebSocket (`car3d_bridge` + `car3d_viewer.html`) |
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
│   └── ads_simulation/              # SUMO bridge, ego spawning, 2D viewer, 3D viewer
│       ├── ads_simulation/
│       │   ├── sumo_bridge.py       # SUMO process, ego + traffic spawn, lane/building geometry
│       │   ├── pygame_viewer.py     # 2D OSM map + click-to-drive UI
│       │   └── car3d_bridge.py      # WebSocket relay for the 3D viewer
│       └── web/
│           └── car3d_viewer.html    # Three.js 3D scene (open directly in a browser)
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

`car3d_bridge` streams live state to a browser-based Three.js scene over a plain WebSocket, scoped to whatever route the ego is currently driving:

- **Lane-accurate roads** — real per-lane width and count (not a single centerline), including the opposite-direction lanes of two-way streets
- **Nearby buildings** — footprints fetched live from Overpass around the current route
- **Background traffic** — a few vehicles driving the reverse direction of the route, so oncoming traffic is visible, not just empty opposing lanes
- A fixed top-down camera that translates to follow the ego without rotating with its heading, so turns read as the car changing screen-direction rather than the camera swinging around it

Everything is scoped to the current route rather than the whole city — routes are picked interactively, so there's no way to know which part of Tempe needs rendering ahead of time, and the full network/every building in Tempe is far more than any one route needs.

Run it alongside the main stack:

```bash
ros2 run ads_simulation car3d_bridge
```

Then open `src/ads_simulation/web/car3d_viewer.html` directly in a browser (works over `file://`, no server needed) and connect to the bridge's `ws://` address shown in the page.

---

## Engineering Notes

The SUMO network is built from a direct Overpass API fetch rather than `osmnx`'s own OSM-XML export — `osmnx.save_graph_xml()` reconstructs a synthetic OSM file that doesn't reliably preserve node IDs shared between ways at intersections, which `netconvert` needs to build real junction connectivity. Feeding it osmnx-reconstructed XML fragmented the network into thousands of disconnected islands; fetching genuine Overpass XML for the same area fixed it.

Ego routing resolves lat/lon clicks to SUMO edges via `sumolib`, trying several nearby candidate edges per pin (not just the closest) and falling back through `getShortestPath()` combinations — the geometrically closest edge to a click is sometimes a disconnected driveway or turnaround stub even on a well-formed network.

The 3D viewer's road surfaces are built junction-free (`includeJunctions=False`) and each lane is extended a fixed distance along its own straight-line direction to close the gap at intersections, rather than tracing SUMO's actual junction polygon. A real junction is a 2D shape with several corners, not a clean continuation of a lane's path — treating its boundary as a 1D point sequence doubled back on itself and produced a self-intersecting, tangled surface. Pure linear extrapolation can't self-intersect, and since the adjacent edge at a junction gets the same treatment, both surfaces overlap enough to visually close the gap.

The paired opposite-direction edge for a two-way street (used for both lane rendering and background traffic) is found by actual network topology — for edge A→B, whichever edge goes B→A, found by checking B's outgoing edges directly — rather than guessing the edge ID via string manipulation (`"123"` → `"-123"`). That convention holds for a simple OSM way, but a long arterial netconvert splits into multiple segments doesn't necessarily mirror the same split points under the same numbering on its reverse direction, so a real reverse edge can exist under an ID the guess never tries.

Nearby buildings for the 3D viewer are fetched live from Overpass, scoped to a padded bounding box around the current route, and run in a background thread — a route is picked interactively, so there's no way to know which part of Tempe needs buildings until the ego is about to drive there, and a synchronous network call inside the route-publish callback would stall ego spawning for however long the request takes.

---

## Design Principles

- **Loose coupling** — every module communicates exclusively through ROS2 topics
- **Single responsibility** — each node has one clearly defined job
- **Real data** — routing and road geometry use actual OpenStreetMap data for Tempe, AZ

---

## License

MIT — see [LICENSE](LICENSE).
