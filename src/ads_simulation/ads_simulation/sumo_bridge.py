"""
SUMO Bridge Node

Manages the SUMO simulation process lifecycle. Waits for /map/status == "READY",
starts SUMO headless or with GUI, and steps the simulation at a fixed rate.

Also spawns the ego vehicle on driver confirmation. The spawn/destination edges
and the route between them are resolved entirely from SUMO's own network
(via sumolib), independent of the OSM route shown in the Pygame viewer — this
guarantees the edge sequence handed to TraCI is actually connected, sidestepping
the broken multi-arg TraCI findRoute() on this SUMO build.

Subscribes
----------
/map/status               (std_msgs/String)  — waits for "READY" before starting SUMO
/navigation/latlon_goal    (std_msgs/String)  — JSON start/end lat-lon from the viewer's S/E pins
/navigation/mission_confirm(std_msgs/String)  — "GO" triggers ego spawn using the last goal

Publishes
---------
/simulation/status  (std_msgs/String)
/vehicle/state       (ads_interfaces/msg/VehicleState)  — ego kinematic state, while ego exists
/navigation/route    (ads_interfaces/msg/Route)  — the ego's actual SUMO edge path, published
                                                    once it spawns. Overwrites the OSM A*
                                                    preview from route_planner in the viewer,
                                                    since the two routers can legitimately
                                                    disagree on which streets to take.
/navigation/route_lanes (std_msgs/String)  — JSON per-lane geometry grouped by edge, for
                                              car3d_bridge's lane-accurate 3D road rendering:
                                              {"edges": [{"direction": "forward"|"reverse",
                                              "lanes": [{"width":, "shape": [[x,y],...]}, ...]}]}.
                                              Includes each route edge's paired opposite-direction
                                              edge if one exists. Raw SUMO x/y, not lat/lon — the
                                              2D viewer doesn't consume this.
/navigation/route_buildings (std_msgs/String)  — JSON building footprints near the current
                                                   route: {"buildings": [[[x,y], ...], ...]}.
                                                   Fetched live from Overpass, scoped to a padded
                                                   bounding box around the route — routes are
                                                   picked interactively so there's no way to know
                                                   which part of Tempe needs buildings ahead of
                                                   time, and downloading the whole city's building
                                                   footprints upfront is far more than any one
                                                   route needs. Runs in a background thread so the
                                                   network round-trip never blocks ego spawning.
                                                   Raw SUMO x/y, for car3d_bridge only.
"""

import json
import math
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, VehicleState, Waypoint


class SumoBridge(Node):

    def __init__(self) -> None:
        super().__init__("sumo_bridge")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        self.declare_parameter("net_filename", "tempe.net.xml")
        self.declare_parameter("cfg_filename", "tempe.sumocfg")
        self.declare_parameter("step_length", 0.05)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("use_gui", False)
        self.declare_parameter("edge_search_radius_m", 50.0)
        self.declare_parameter("building_search_pad_deg", 0.0006)   # ~60m

        self._cache_dir    = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length  = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._use_gui      = self.get_parameter("use_gui").value
        self._edge_search_radius_m = self.get_parameter("edge_search_radius_m").value
        self._building_pad_deg = self.get_parameter("building_search_pad_deg").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        self._status_pub = self.create_publisher(String, "/simulation/status", 10)
        self._state_pub  = self.create_publisher(VehicleState, "/vehicle/state", 10)
        self._route_pub  = self.create_publisher(Route, "/navigation/route", 10)
        self._lanes_pub  = self.create_publisher(String, "/navigation/route_lanes", 10)
        self._buildings_pub = self.create_publisher(String, "/navigation/route_buildings", 10)

        self._map_status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )
        self._latlon_goal_sub = self.create_subscription(
            String, "/navigation/latlon_goal", self._on_latlon_goal, 10
        )
        self._mission_confirm_sub = self.create_subscription(
            String, "/navigation/mission_confirm", self._on_mission_confirm, 10
        )

        self._sumo_running: bool      = False
        self._traci: Optional[object] = None
        self._net: Optional[object]   = None   # sumolib.net.Net, loaded once SUMO starts
        self._pending_goal: Optional[dict] = None
        self._route_counter = 0

        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info(
            f"SumoBridge initialized — net='{self._net_path}', "
            f"gui={self._use_gui}, rate={self._publish_rate} Hz"
        )

    def _on_map_status(self, msg: String) -> None:
        if msg.data == "READY" and not self._sumo_running:
            self._start_sumo()

    def _on_latlon_goal(self, msg: String) -> None:
        try:
            goal = json.loads(msg.data)
            self._pending_goal = {
                "start_lat": float(goal["start_lat"]),
                "start_lon": float(goal["start_lon"]),
                "end_lat":   float(goal["end_lat"]),
                "end_lon":   float(goal["end_lon"]),
            }
        except (json.JSONDecodeError, KeyError, ValueError) as exc:
            self.get_logger().error(f"Invalid latlon_goal format: {exc}")
            return

        self._preview_route(self._pending_goal)

    def _preview_route(self, goal: dict) -> None:
        """Publish the real SUMO route as soon as both pins are placed, before
        the ego spawns — so the viewer never shows a route the car won't
        actually drive.

        Pins can be placed before SUMO finishes starting (the Pygame window
        opens well before "SUMO started" logs) — in that case this bails out
        silently, but _start_sumo re-calls it once SUMO comes up if a goal is
        already pending, so no click before that point is lost.
        """
        if not self._sumo_running:
            self.get_logger().info("Preview deferred — SUMO not running yet.")
            return
        if not self._load_net():
            return
        resolved = self._resolve_route(goal)
        if resolved is None:
            return
        edges, _start_edge, _end_edge = resolved
        self._publish_actual_route(edges, goal)
        self.get_logger().info(f"Preview route published — {len(edges)} edges.")

    def _on_mission_confirm(self, msg: String) -> None:
        if msg.data != "GO":
            return
        if not self._sumo_running:
            self.get_logger().warn("Mission confirmed but SUMO is not running yet.")
            return
        if self._pending_goal is None:
            self.get_logger().warn("Mission confirmed but no S/E pins have been placed yet.")
            return
        self._spawn_ego(self._pending_goal)

    def _start_sumo(self) -> None:
        if not self._net_path.exists():
            self.get_logger().error(
                f"SUMO network file not found: '{self._net_path}'. "
                "Ensure map_loader completed successfully."
            )
            return

        try:
            import traci
            self._traci = traci
        except ImportError:
            self.get_logger().error("traci not found. Run: sudo apt install sumo sumo-tools")
            return

        sumo_binary = "sumo-gui" if self._use_gui else "sumo"

        use_cfg = self._cfg_path.exists()
        base_args = (
            [sumo_binary, "-c", str(self._cfg_path)] if use_cfg
            else [sumo_binary, "--net-file", str(self._net_path)]
        )

        sumo_cmd = base_args + [
            "--no-step-log", "--no-warnings", "--start",
            "--ignore-route-errors",
            "--tls.all-off",
            f"--step-length={self._step_length}",
            "--time-to-teleport=-1",
        ]

        self.get_logger().info(f"Starting {sumo_binary} ...")

        try:
            traci.start(sumo_cmd)
            self._sumo_running = True
            self._publish_status("RUNNING")
            self.get_logger().info(
                f"SUMO started — cfg={'yes' if use_cfg else 'no'}, gui={self._use_gui}"
            )
            # Load the network now, synchronously, so the first pin-placement
            # doesn't stall behind a multi-second XML parse — sumolib.net.readNet()
            # on a network this size blocks the whole single-threaded executor,
            # including the mission_confirm subscription, if deferred to first use.
            self._load_net()
            if self._pending_goal is not None:
                self._preview_route(self._pending_goal)
        except Exception as exc:
            self.get_logger().error(f"Failed to start SUMO: {exc}")
            self._publish_status("ERROR")

    # ------------------------------------------------------------------
    # Ego spawning
    #
    # Spawn/destination edges and the path between them are resolved purely
    # from SUMO's own network graph via sumolib — never from the OSM A*
    # route. That keeps edge lookup (getNeighboringEdges, using the net's
    # own projection) and path connectivity (getShortestPath, Dijkstra over
    # SUMO's real edge connections) both grounded in the graph TraCI will
    # actually drive on, avoiding the broken TraCI findRoute() stitching
    # that caused the ego to stall mid-route previously.
    # ------------------------------------------------------------------

    def _load_net(self) -> bool:
        if self._net is not None:
            return True
        try:
            import sumolib
        except ImportError:
            self.get_logger().error("sumolib not found. Run: sudo apt install sumo sumo-tools")
            return False
        try:
            self._net = sumolib.net.readNet(str(self._net_path))
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to load SUMO network for routing: {exc}")
            return False

    def _nearby_edges(self, lat: float, lon: float, limit: int = 5) -> list:
        """Return up to `limit` drivable SUMO edges near a lat/lon, nearest first.

        Returning several candidates (not just the single closest) matters
        because OSM-derived networks include short dead-end fragments —
        driveways, service stubs, turnarounds — that can be geometrically
        closest to a pin while having no real outgoing connectivity. Trying
        several candidates lets _spawn_ego fall back past a disconnected
        stub to a real through-street.
        """
        x, y = self._net.convertLonLat2XY(lon, lat)
        candidates = self._net.getNeighboringEdges(
            x, y, r=self._edge_search_radius_m, includeJunctions=False
        )
        candidates = [
            (edge, dist) for edge, dist in candidates
            if edge.allows("passenger") and not edge.getID().startswith(":")
        ]
        candidates.sort(key=lambda pair: pair[1])
        return [edge for edge, _dist in candidates[:limit]]

    def _resolve_route(self, goal: dict):
        """Resolve a lat/lon goal to a connected SUMO edge path.

        Returns (edges, start_edge, end_edge), or None if no drivable edge
        was found near a pin or no connected path exists between candidates.
        Shared by _preview_route (before spawn) and _spawn_ego (at spawn) so
        both always agree on exactly the same route.
        """
        start_candidates = self._nearby_edges(goal["start_lat"], goal["start_lon"])
        end_candidates = self._nearby_edges(goal["end_lat"], goal["end_lon"])
        if not start_candidates or not end_candidates:
            self.get_logger().error(
                f"No drivable edge within {self._edge_search_radius_m}m of the S or E pin."
            )
            return None

        for s_edge in start_candidates:
            for e_edge in end_candidates:
                if s_edge.getID() == e_edge.getID():
                    return [s_edge], s_edge, e_edge
                path, _cost = self._net.getShortestPath(s_edge, e_edge, vClass="passenger")
                if path is not None:
                    return path, s_edge, e_edge

        self.get_logger().error(
            f"No connected route found among {len(start_candidates)} start "
            f"and {len(end_candidates)} end edge candidates near the pins."
        )
        return None

    def _spawn_ego(self, goal: dict) -> None:
        if not self._load_net():
            return

        resolved = self._resolve_route(goal)
        if resolved is None:
            return
        edges, start_edge, end_edge = resolved
        edge_ids = [edge.getID() for edge in edges]

        try:
            if "ego" in self._traci.vehicle.getIDList():
                self._traci.vehicle.remove("ego")

            self._route_counter += 1
            route_id = f"ego_route_{self._route_counter}"
            self._traci.route.add(route_id, edge_ids)
            self._traci.vehicle.add("ego", route_id, departLane="best", departSpeed="max")
            self._traci.vehicle.setSpeedMode("ego", 7)
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn ego vehicle: {exc}")
            return

        self.get_logger().info(
            f"Ego spawned — {len(edge_ids)} edges, "
            f"'{start_edge.getID()}' → '{end_edge.getID()}'."
        )

        self._publish_actual_route(edges, goal)

    def _publish_actual_route(self, edges: list, goal: dict) -> None:
        """Publish the ego's real SUMO edge path as the displayed route.

        route_planner's OSM A* preview and this edge path come from two
        different graphs (OSM nodes vs. SUMO edges) with different weighting,
        so they can legitimately pick different streets for the same start/end.
        Overwriting /navigation/route with the path SUMO will actually drive
        keeps what the viewer draws honest about what happens.
        """
        waypoints = []
        for i, edge in enumerate(edges):
            road_name = edge.getName() or edge.getID()
            speed_limit = edge.getSpeed()
            shape = edge.getShape(includeJunctions=False)
            points = shape[1:] if i > 0 else shape   # skip duplicate junction point
            for x, y in points:
                lon, lat = self._net.convertXY2LonLat(x, y)
                wp = Waypoint()
                wp.latitude    = lat
                wp.longitude   = lon
                wp.x           = x
                wp.y           = y
                wp.osmid       = 0
                wp.speed_limit = speed_limit
                wp.road_name   = road_name
                waypoints.append(wp)

        total_distance = sum(edge.getLength() for edge in edges)
        estimated_duration = sum(
            edge.getLength() / max(edge.getSpeed(), 0.1) for edge in edges
        )

        route_msg = Route()
        route_msg.header.stamp = self.get_clock().now().to_msg()
        route_msg.header.frame_id = "map"
        route_msg.start_address = f"{goal['start_lat']:.5f}, {goal['start_lon']:.5f}"
        route_msg.end_address   = f"{goal['end_lat']:.5f}, {goal['end_lon']:.5f}"
        route_msg.total_distance_m = total_distance
        route_msg.estimated_duration_s = estimated_duration
        route_msg.waypoints = waypoints
        route_msg.current_waypoint_index = 0

        self._route_pub.publish(route_msg)
        self._publish_route_lanes(edges)
        self._publish_route_buildings(waypoints)

    def _publish_route_buildings(self, waypoints: list) -> None:
        """Kick off a background fetch of building footprints near the route.

        Routes are picked interactively, so there's no way to know ahead of
        time which part of Tempe needs buildings — this fetches live from
        Overpass, scoped to a padded box around the route's own lat/lon
        (already computed for the waypoints), rather than pre-downloading
        every building in the city. Runs in a background thread: a live
        HTTP round-trip done inline in this callback would stall ego
        spawning for however long the request takes — the same class of
        bug fixed earlier by not blocking on synchronous network/IO work
        inside a ROS2 subscription callback.
        """
        lats = [wp.latitude for wp in waypoints]
        lons = [wp.longitude for wp in waypoints]
        pad = self._building_pad_deg
        bbox = (min(lats) - pad, min(lons) - pad, max(lats) + pad, max(lons) + pad)
        threading.Thread(
            target=self._fetch_and_publish_buildings, args=(bbox,), daemon=True
        ).start()

    def _fetch_and_publish_buildings(self, bbox: tuple) -> None:
        south, west, north, east = bbox
        query = (
            "[out:xml][timeout:30];"
            f'way["building"]({south},{west},{north},{east});'
            "(._;>;);"
            "out body;"
        )
        try:
            import requests
            response = requests.post(
                "https://overpass-api.de/api/interpreter",
                data={"data": query},
                headers={"User-Agent": "autonomous-driving-stack/0.1 (portfolio project)"},
                timeout=30,
            )
            response.raise_for_status()
        except Exception as exc:
            self.get_logger().warn(f"Building fetch failed (non-fatal): {exc}")
            return

        try:
            import xml.etree.ElementTree as ET
            root = ET.fromstring(response.content)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse building response: {exc}")
            return

        nodes = {}
        for node in root.findall("node"):
            try:
                nodes[node.get("id")] = (float(node.get("lat")), float(node.get("lon")))
            except (TypeError, ValueError):
                continue

        buildings = []
        for way in root.findall("way"):
            if not any(tag.get("k") == "building" for tag in way.findall("tag")):
                continue
            shape = []
            for nd in way.findall("nd"):
                latlon = nodes.get(nd.get("ref"))
                if latlon is None:
                    continue
                x, y = self._net.convertLonLat2XY(latlon[1], latlon[0])
                shape.append([x, y])
            if len(shape) >= 3:
                buildings.append(shape)

        msg = String()
        msg.data = json.dumps({"buildings": buildings})
        self._buildings_pub.publish(msg)
        self.get_logger().info(f"Fetched {len(buildings)} building footprints near route.")

    def _publish_route_lanes(self, edges: list) -> None:
        """Publish per-lane geometry for the current route, for car3d_bridge.

        The ego's real TraCI position is already laterally correct for
        whichever lane it's actually in — this is purely about giving the 3D
        viewer real lane width/count to render, instead of a single
        edge-centerline that makes a correctly-positioned car look like it's
        drifting off a thin line on any multi-lane road.

        Lanes are grouped by edge (not sent as one flat list) so the viewer
        can tell an internal divider between two lanes of the same edge
        apart from the road's outer edge. Each route edge's paired
        opposite-direction edge is looked up and included too (marked
        direction="reverse") so the full two-way street renders, not just
        the single direction the ego is actually driving on — netconvert's
        OSM two-way-street convention pairs edge "123" with "-123", which
        this relies on; a genuinely one-way street simply has no pair.
        """
        def edge_group(edge, direction: str) -> dict:
            return {
                "direction": direction,
                "lanes": [
                    # includeJunctions=False — a real junction is a 2D
                    # polygon, not a clean continuation of the lane's path,
                    # so treating its boundary as a 1D point sequence
                    # doubles back on itself and produces a tangled,
                    # self-overlapping mess once turned into a filled
                    # surface. car3d_viewer instead extends each lane's own
                    # straight-line direction a few meters past its true
                    # endpoint to close the gap — simple linear
                    # extrapolation that can never self-intersect.
                    {
                        "width": lane.getWidth(),
                        "shape": [list(p) for p in lane.getShape(includeJunctions=False)],
                    }
                    for lane in edge.getLanes()
                ],
            }

        groups = [edge_group(edge, "forward") for edge in edges]

        seen_reverse_ids = set()
        for edge in edges:
            eid = edge.getID()
            reverse_id = eid[1:] if eid.startswith("-") else f"-{eid}"
            if reverse_id in seen_reverse_ids:
                continue
            try:
                reverse_edge = self._net.getEdge(reverse_id)
            except KeyError:
                continue
            seen_reverse_ids.add(reverse_id)
            groups.append(edge_group(reverse_edge, "reverse"))

        msg = String()
        msg.data = json.dumps({"edges": groups})
        self._lanes_pub.publish(msg)

    def _stop_sumo(self) -> None:
        if self._sumo_running and self._traci is not None:
            try:
                self._traci.close()
            except Exception:
                pass
            self._sumo_running = False
            self.get_logger().info("SUMO stopped.")

    def _tick(self) -> None:
        if not self._sumo_running:
            return
        try:
            self._traci.simulationStep()
            self._publish_ego_state()
        except Exception as exc:
            self.get_logger().error(f"SUMO step failed: {exc}")
            self._sumo_running = False

    def _publish_ego_state(self) -> None:
        if "ego" not in self._traci.vehicle.getIDList():
            return

        x, y = self._traci.vehicle.getPosition("ego")
        sumo_angle_deg = self._traci.vehicle.getAngle("ego")  # clockwise from north
        # Convert to the message's convention: radians, 0 = east, CCW positive.
        heading = math.atan2(
            math.sin(math.radians(90.0 - sumo_angle_deg)),
            math.cos(math.radians(90.0 - sumo_angle_deg)),
        )

        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.speed = self._traci.vehicle.getSpeed("ego")
        msg.acceleration = self._traci.vehicle.getAcceleration("ego")
        msg.heading = heading
        msg.x = x
        msg.y = y
        msg.is_autonomous = True
        self._state_pub.publish(msg)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        self._stop_sumo()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SumoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SumoBridge.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
