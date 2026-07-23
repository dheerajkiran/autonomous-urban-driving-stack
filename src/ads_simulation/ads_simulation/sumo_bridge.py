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
/navigation/traffic_vehicles (std_msgs/String)  — JSON state for ambient traffic, published
                                                    every tick: {"vehicles": [{"id":, "x":, "y":,
                                                    "heading":, "speed":}, ...]}. Hundreds of
                                                    vehicles are spawned/replenished on random
                                                    origin/destination pairs across the whole
                                                    network (_replenish_ambient_traffic) for a
                                                    genuinely city-scale feel, but only whichever
                                                    of them are currently within road_context_pad_m
                                                    of the ego's live position are included here —
                                                    car3d_viewer's software-rendered WebGL can't
                                                    afford hundreds of rendered cars regardless of
                                                    how many actually exist in the simulation.
                                                    Raw SUMO x/y, for car3d_bridge only.
"""

import json
import math
import random
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
        self.declare_parameter("road_context_pad_m", 450.0)
        self.declare_parameter("ambient_traffic_count", 300)
        self.declare_parameter("ambient_spawn_batch", 10)

        self._cache_dir    = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length  = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._use_gui      = self.get_parameter("use_gui").value
        self._edge_search_radius_m = self.get_parameter("edge_search_radius_m").value
        self._building_pad_deg = self.get_parameter("building_search_pad_deg").value
        self._road_context_pad_m = self.get_parameter("road_context_pad_m").value
        self._ambient_traffic_count = self.get_parameter("ambient_traffic_count").value
        self._ambient_spawn_batch = self.get_parameter("ambient_spawn_batch").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        self._status_pub = self.create_publisher(String, "/simulation/status", 10)
        self._state_pub  = self.create_publisher(VehicleState, "/vehicle/state", 10)
        self._route_pub  = self.create_publisher(Route, "/navigation/route", 10)
        self._lanes_pub  = self.create_publisher(String, "/navigation/route_lanes", 10)
        self._buildings_pub = self.create_publisher(String, "/navigation/route_buildings", 10)
        self._traffic_pub   = self.create_publisher(String, "/navigation/traffic_vehicles", 10)

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
        self._all_drivable_edges: Optional[list] = None   # cached once, on net load
        self._pending_goal: Optional[dict] = None
        self._route_counter = 0
        self._ambient_counter = 0

        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)
        # Separate, slower timer — ambient traffic is city-wide and
        # independent of whatever route the ego is on, so it doesn't need
        # the main 50Hz simulation tick's cadence, and routing a batch of
        # new vehicles is too much work to redo that often anyway.
        self._ambient_timer = self.create_timer(2.0, self._replenish_ambient_traffic)

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
            self._all_drivable_edges = [
                e for e in self._net.getEdges()
                if e.allows("passenger") and not e.getID().startswith(":")
            ]
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

        self._publish_actual_route(edges, goal, spawn=True)

    def _publish_actual_route(self, edges: list, goal: dict, spawn: bool = False) -> None:
        """Publish the ego's real SUMO edge path as the displayed route.

        route_planner's OSM A* preview and this edge path come from two
        different graphs (OSM nodes vs. SUMO edges) with different weighting,
        so they can legitimately pick different streets for the same start/end.
        Overwriting /navigation/route with the path SUMO will actually drive
        keeps what the viewer draws honest about what happens.

        `spawn` gates the building fetch specifically — unlike the route/lane
        publish (cheap, purely local, fine to redo on every pin adjustment
        before the user commits), it's a live Overpass HTTP call. Firing one
        on every preview update hammers Overpass hard enough while testing
        multiple routes in quick succession to get rate-limited (429) or
        time out (504) — so it only runs once the ego actually spawns.
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
        if spawn:
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
            "[out:xml][timeout:60];"
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
                timeout=60,
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

    def _reverse_edges(self, edges: list) -> list:
        """Look up each edge's paired opposite-direction edge, in order.

        Found by actual network topology — for edge A->B, the reverse is
        whichever edge goes B->A, found by checking B's outgoing edges
        directly — not by guessing an ID string like "-123". netconvert's
        "123"/"-123" OSM two-way-street convention holds for a simple way,
        but a long arterial split into multiple segments ("123#0", "123#1",
        ...) doesn't necessarily mirror the same split points under the
        same numbering on its reverse direction, so a same-ID-but-negated
        guess can miss a real reverse edge that exists under a different
        ID. A genuinely one-way street has no B->A edge at all, and is
        skipped. Deduplicated and order-preserving, so callers that need a
        connected reverse route (background traffic) can just reverse the
        returned list.
        """
        result = []
        seen_ids = set()
        for edge in edges:
            from_node, to_node = edge.getFromNode(), edge.getToNode()
            reverse_edge = next(
                (c for c in to_node.getOutgoing() if c.getToNode() == from_node),
                None,
            )
            if reverse_edge is None or reverse_edge.getID() in seen_ids:
                continue
            seen_ids.add(reverse_edge.getID())
            result.append(reverse_edge)
        return result

    def _replenish_ambient_traffic(self) -> None:
        """Keep a steady ambient vehicle count spread across the whole
        network, independent of whatever route the ego is on — random
        origin/destination edge pairs, routed with the same sumolib
        getShortestPath used for the ego, so a genuinely city-scale traffic
        feel rather than only near the ego's own route. Vehicles that
        finish their random trip exit the network naturally and get
        replaced here; car3d_bridge only ever sees whichever of these end
        up near the ego at a given moment (see _publish_traffic_vehicles) —
        rendering all of them regardless of distance isn't something this
        3D viewer, running on software-rendered WebGL, can afford at a
        target count in the hundreds.
        """
        if not self._sumo_running or not self._all_drivable_edges:
            return

        current = sum(
            1 for v in self._traci.vehicle.getIDList() if v.startswith("ambient_")
        )
        to_spawn = min(self._ambient_spawn_batch, self._ambient_traffic_count - current)
        if to_spawn <= 0:
            return

        spawned = 0
        for _ in range(to_spawn):
            for _attempt in range(5):
                origin = random.choice(self._all_drivable_edges)
                dest = random.choice(self._all_drivable_edges)
                if origin.getID() == dest.getID():
                    continue
                path, _cost = self._net.getShortestPath(origin, dest, vClass="passenger")
                if path is None:
                    continue

                self._ambient_counter += 1
                vid = f"ambient_{self._ambient_counter}"
                try:
                    route_id = f"ambient_route_{self._ambient_counter}"
                    self._traci.route.add(route_id, [e.getID() for e in path])
                    self._traci.vehicle.add(
                        vid, route_id, departLane="random", departSpeed="max",
                    )
                    self._traci.vehicle.setSpeedMode(vid, 7)
                    spawned += 1
                except Exception:
                    pass   # route/vehicle add failed for this attempt — skip it
                break

        if spawned:
            self.get_logger().info(
                f"Ambient traffic replenished — +{spawned}, "
                f"{current + spawned}/{self._ambient_traffic_count} total."
            )

    def _nearby_drivable_edges(self, edges: list, pad_m: float) -> list:
        """All drivable edges within a padded bounding box around a set of
        edges — the road network is already fully loaded locally (unlike
        buildings, no live fetch needed), so this is just a bounding-box
        scan over sumolib's own edge list."""
        xs, ys = [], []
        for edge in edges:
            for x, y in edge.getShape():
                xs.append(x)
                ys.append(y)
        xmin, xmax = min(xs) - pad_m, max(xs) + pad_m
        ymin, ymax = min(ys) - pad_m, max(ys) + pad_m

        return [
            e for e in self._all_drivable_edges
            if any(xmin <= x <= xmax and ymin <= y <= ymax for x, y in e.getShape())
        ]

    @staticmethod
    def _junction_patch_shape(node, margin_m: float = 2.0, points: int = 10) -> list:
        """Real junction polygon if one exists, else a small synthetic
        circle at the node's actual location — every node where two edges
        meet needs *some* patch shape, real intersection or not.

        The circle's radius is sized to the widest edge actually connected
        to this node (half its total lane width, plus a margin), not a flat
        constant — a fixed small radius left a real-world lane-count change
        (e.g. a road going from 1 to 3 lanes) poking out past the patch as
        a visible rectangular step, since the patch wasn't wide enough to
        cover the wider edge in the first place.
        """
        real_shape = node.getShape()
        if real_shape:
            return [list(p) for p in real_shape]

        connected = list(node.getIncoming()) + list(node.getOutgoing())
        half_widths = [
            sum(lane.getWidth() for lane in edge.getLanes()) / 2.0
            for edge in connected
        ]
        radius = (max(half_widths) if half_widths else 3.0) + margin_m

        cx, cy = node.getCoord()
        return [
            [cx + radius * math.cos(2 * math.pi * i / points),
             cy + radius * math.sin(2 * math.pi * i / points)]
            for i in range(points)
        ]

    def _publish_route_lanes(self, edges: list) -> None:
        """Publish per-lane geometry for car3d_bridge — not just the ego's
        own route, but every drivable edge in the surrounding area, the
        same way _publish_route_buildings shows nearby buildings rather
        than only ones the route directly touches. This is purely visual
        context: it doesn't change ego routing/driving at all, which still
        only ever uses `edges` (the actual route) elsewhere.

        The ego's real TraCI position is already laterally correct for
        whichever lane it's actually in — this is purely about giving the 3D
        viewer real lane width/count to render, instead of a single
        edge-centerline that makes a correctly-positioned car look like it's
        drifting off a thin line on any multi-lane road.

        Lanes are grouped by edge (not sent as one flat list) so the viewer
        can tell an internal divider between two lanes of the same edge
        apart from the road's outer edge. Each edge's paired
        opposite-direction edge (see _reverse_edges) is looked up and
        included too (marked direction="reverse") so two-way streets
        render fully; a genuinely one-way street simply has no pair.
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
                    # surface. Junction polygons are sent separately below
                    # and rendered as their own patch instead — geometrically
                    # correct regardless of how sharply the road turns there,
                    # unlike car3d_viewer's old straight-line extrapolation
                    # hack, which only worked where a road kept going roughly
                    # straight through the intersection.
                    {
                        "width": lane.getWidth(),
                        "shape": [list(p) for p in lane.getShape(includeJunctions=False)],
                    }
                    for lane in edge.getLanes()
                ],
            }

        context_edges = self._nearby_drivable_edges(edges, self._road_context_pad_m)
        reverse_edges = self._reverse_edges(context_edges)
        reverse_ids = {e.getID() for e in reverse_edges}

        seen_ids = set()
        groups = []
        unique_edges = []
        for edge in context_edges + reverse_edges:
            if edge.getID() in seen_ids:
                continue
            seen_ids.add(edge.getID())
            unique_edges.append(edge)
            direction = "reverse" if edge.getID() in reverse_ids else "forward"
            groups.append(edge_group(edge, direction))

        # Real junction footprints (SUMO's own polygon for the paved
        # intersection area), one per unique node touched by any edge above.
        # Not every node is a real intersection, though — netconvert splits
        # an OSM way into a new edge at *any* shared node, including purely
        # shape-defining ones used just to trace a gentle curve in an
        # otherwise "straight" road, and those have no real junction
        # polygon (getShape() comes back empty). Skipping them left a real
        # gap at every such point — visually identical to the sharp-turn
        # tangle this was meant to fix, just triggered by a curve instead
        # of an intersection. A small synthetic circle at the node's actual
        # location closes it regardless of why the node exists.
        junction_nodes = {}
        for edge in unique_edges:
            for node in (edge.getFromNode(), edge.getToNode()):
                junction_nodes[node.getID()] = node
        junctions = [
            {"shape": self._junction_patch_shape(node)}
            for node in junction_nodes.values()
        ]

        msg = String()
        msg.data = json.dumps({"edges": groups, "junctions": junctions})
        self._lanes_pub.publish(msg)
        self.get_logger().info(
            f"Road context published — {len(groups)} edges, {len(junctions)} junctions."
        )

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
            self._publish_traffic_vehicles()
        except Exception as exc:
            self.get_logger().error(f"SUMO step failed: {exc}")
            self._sumo_running = False

    @staticmethod
    def _sumo_heading(angle_deg: float) -> float:
        """SUMO's vehicle angle is degrees clockwise from north — convert to
        radians, 0 = east, CCW positive (the convention every consumer of
        vehicle state in this codebase, ego or traffic, expects)."""
        return math.atan2(
            math.sin(math.radians(90.0 - angle_deg)),
            math.cos(math.radians(90.0 - angle_deg)),
        )

    def _publish_ego_state(self) -> None:
        if "ego" not in self._traci.vehicle.getIDList():
            return

        x, y = self._traci.vehicle.getPosition("ego")
        heading = self._sumo_heading(self._traci.vehicle.getAngle("ego"))

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

    def _publish_traffic_vehicles(self) -> None:
        """Publish live state for whichever ambient traffic is currently
        near the ego, for car3d_bridge.

        Ambient traffic is spawned city-wide (_replenish_ambient_traffic)
        and SUMO tracks its lifecycle on its own — nothing here needs to
        remember vehicle ids across calls, unlike the old route-concentrated
        design. What this does need to do is filter to a bounded radius
        around the ego's *current* position (not the whole route's
        bounding box, which is what lanes/buildings use) — traffic is
        continuously moving and the ego is continuously moving along a
        route that can span kilometers, so "nearby" has to be relative to
        where the ego actually is right now, not a static area computed
        once back when the route was published.
        """
        if "ego" not in self._traci.vehicle.getIDList():
            return
        ego_x, ego_y = self._traci.vehicle.getPosition("ego")
        pad = self._road_context_pad_m

        vehicles = []
        for vid in self._traci.vehicle.getIDList():
            if vid == "ego":
                continue
            x, y = self._traci.vehicle.getPosition(vid)
            if abs(x - ego_x) > pad or abs(y - ego_y) > pad:
                continue
            vehicles.append({
                "id": vid,
                "x": x,
                "y": y,
                "heading": self._sumo_heading(self._traci.vehicle.getAngle(vid)),
                "speed": self._traci.vehicle.getSpeed(vid),
            })

        msg = String()
        msg.data = json.dumps({"vehicles": vehicles})
        self._traffic_pub.publish(msg)

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
