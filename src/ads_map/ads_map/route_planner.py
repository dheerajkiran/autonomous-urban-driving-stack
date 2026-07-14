"""
Route Planner Node

Subscribes to mission goals (start + destination addresses) and computes
an optimal driving route on the Tempe OSM road network using NetworkX A*.

Geocodes human-readable addresses to OSM node IDs using osmnx's Nominatim
wrapper, then finds the shortest path weighted by travel time (distance / speed_limit).

Subscribes
----------
/navigation/mission_goal  (std_msgs/String)  — JSON: {"start": "...", "end": "..."}
/map/status               (std_msgs/String)  — waits for "READY" before accepting goals

Publishes
---------
/navigation/route  (ads_interfaces/msg/Route)  — full ordered waypoint sequence
"""

import json
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, Waypoint


class RoutePlanner(Node):
    """
    Computes city-scale driving routes on the Tempe road network.

    Waits for the map to be ready (/map/status == "READY"), then accepts
    mission goals and publishes Route messages. Supports any pair of addresses
    or intersections within Tempe, AZ.

    Route weight
    ------------
    Edges are weighted by travel_time = length / speed_kph, favouring faster
    roads over shorter ones. Falls back to edge length if speed data is absent.
    """

    def __init__(self) -> None:
        super().__init__("route_planner")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        self.declare_parameter("graphml_filename", "tempe_drive.graphml")
        self.declare_parameter("default_speed_kph", 50.0)
        self.declare_parameter("waypoint_spacing_m", 30.0)

        self._cache_dir = Path(self.get_parameter("cache_dir").value).expanduser()
        self._graphml_filename = self.get_parameter("graphml_filename").value
        self._default_speed_kph = self.get_parameter("default_speed_kph").value
        self._waypoint_spacing_m = self.get_parameter("waypoint_spacing_m").value

        self._graph = None
        self._map_ready = False

        self._route_pub = self.create_publisher(Route, "/navigation/route", 10)

        self._status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )
        self._goal_sub = self.create_subscription(
            String, "/navigation/mission_goal", self._on_mission_goal, 10
        )
        self._latlon_goal_sub = self.create_subscription(
            String, "/navigation/latlon_goal", self._on_latlon_goal, 10
        )

        self.get_logger().info("RoutePlanner initialized — waiting for map.")

    # ------------------------------------------------------------------
    # Map readiness
    # ------------------------------------------------------------------

    def _on_map_status(self, msg: String) -> None:
        if msg.data == "READY" and not self._map_ready:
            self._load_graph()

    def _load_graph(self) -> None:
        graphml_path = self._cache_dir / self._graphml_filename
        if not graphml_path.exists():
            self.get_logger().error(
                f"GraphML not found at '{graphml_path}'. Ensure map_loader ran first."
            )
            return
        try:
            import osmnx as ox
            self._graph = ox.load_graphml(str(graphml_path))
            self._map_ready = True
            self.get_logger().info(
                f"Road graph loaded — {len(self._graph.nodes):,} nodes, "
                f"{len(self._graph.edges):,} edges. Ready for route requests."
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load graph: {exc}")

    # ------------------------------------------------------------------
    # Mission goal handling
    # ------------------------------------------------------------------

    def _on_mission_goal(self, msg: String) -> None:
        if not self._map_ready:
            self.get_logger().warn("Route request received but map is not ready yet.")
            return
        try:
            goal = json.loads(msg.data)
            start_addr = goal["start"]
            end_addr = goal["end"]
        except (json.JSONDecodeError, KeyError) as exc:
            self.get_logger().error(
                f"Invalid mission goal format: {exc}. "
                "Expected JSON: {{\"start\": \"...\", \"end\": \"...\"}}"
            )
            return

        self.get_logger().info(f"Route request: '{start_addr}' → '{end_addr}'")
        self._compute_and_publish_route(start_addr, end_addr)

    def _on_latlon_goal(self, msg: String) -> None:
        """Handle a click-to-drive goal from the Pygame viewer (lat/lon, no geocoding)."""
        if not self._map_ready:
            self.get_logger().warn("Lat/lon goal received but map is not ready yet.")
            return
        try:
            goal = json.loads(msg.data)
            start_lat = float(goal["start_lat"])
            start_lon = float(goal["start_lon"])
            end_lat   = float(goal["end_lat"])
            end_lon   = float(goal["end_lon"])
        except (json.JSONDecodeError, KeyError, ValueError) as exc:
            self.get_logger().error(f"Invalid latlon_goal format: {exc}")
            return

        import osmnx as ox
        try:
            start_node = ox.distance.nearest_nodes(self._graph, start_lon, start_lat)
            end_node   = ox.distance.nearest_nodes(self._graph, end_lon,   end_lat)
        except Exception as exc:
            self.get_logger().error(f"nearest_nodes failed: {exc}")
            return

        start_label = f"{start_lat:.5f}, {start_lon:.5f}"
        end_label   = f"{end_lat:.5f}, {end_lon:.5f}"
        self.get_logger().info(f"Lat/lon route: ({start_label}) → ({end_label})")
        self._route_from_nodes(start_node, end_node, start_label, end_label)

    def _compute_and_publish_route(self, start_addr: str, end_addr: str) -> None:
        import osmnx as ox

        try:
            start_node = ox.distance.nearest_nodes(
                self._graph, *self._geocode(start_addr)
            )
            end_node = ox.distance.nearest_nodes(
                self._graph, *self._geocode(end_addr)
            )
        except Exception as exc:
            self.get_logger().error(f"Geocoding failed: {exc}")
            return

        self._route_from_nodes(start_node, end_node, start_addr, end_addr)

    def _route_from_nodes(self, start_node, end_node, start_label: str, end_label: str) -> None:
        import networkx as nx

        try:
            node_path = nx.shortest_path(
                self._graph, start_node, end_node, weight="length"
            )
        except nx.NetworkXNoPath:
            self.get_logger().error(
                f"No path found between '{start_label}' and '{end_label}'."
            )
            return

        waypoints = self._nodes_to_waypoints(node_path)
        total_distance = self._compute_distance(waypoints)
        avg_speed_ms = (self._default_speed_kph * 1000.0) / 3600.0
        estimated_duration = total_distance / avg_speed_ms if avg_speed_ms > 0 else 0.0

        route_msg = Route()
        route_msg.header.stamp = self.get_clock().now().to_msg()
        route_msg.header.frame_id = "map"
        route_msg.start_address = start_label
        route_msg.end_address   = end_label
        route_msg.total_distance_m = total_distance
        route_msg.estimated_duration_s = estimated_duration
        route_msg.waypoints = waypoints
        route_msg.current_waypoint_index = 0

        self._route_pub.publish(route_msg)

        self.get_logger().info(
            f"Route published — {len(waypoints)} waypoints, "
            f"{total_distance / 1000.0:.2f} km, "
            f"~{estimated_duration / 60.0:.1f} min"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _geocode(self, address: str) -> tuple[float, float]:
        """Return (longitude, latitude) for an address string."""
        import osmnx as ox
        point = ox.geocoder.geocode(address)
        if point is None:
            raise ValueError(f"Could not geocode address: '{address}'")
        lat, lon = point
        return lon, lat  # osmnx nearest_nodes expects (X=lon, Y=lat)

    def _nodes_to_waypoints(self, node_path: list) -> list[Waypoint]:
        """Build waypoints from A* node path, including OSM edge geometry so the
        displayed route follows actual road curves instead of straight-line shortcuts."""
        waypoints = []
        for i, osmid in enumerate(node_path):
            node_data   = self._graph.nodes[osmid]
            lat         = node_data.get("y", 0.0)
            lon         = node_data.get("x", 0.0)
            street_name = self._get_street_name(osmid)
            speed_limit = self._get_speed_limit(osmid)

            wp = Waypoint()
            wp.latitude    = lat
            wp.longitude   = lon
            wp.x           = 0.0
            wp.y           = 0.0
            wp.osmid       = int(osmid)
            wp.speed_limit = speed_limit
            wp.road_name   = street_name
            waypoints.append(wp)

            # Insert intermediate geometry points for this edge so the
            # route polyline traces the actual road curve, not a straight line.
            if i < len(node_path) - 1:
                next_id   = node_path[i + 1]
                edge_data = self._graph.get_edge_data(osmid, next_id)
                if edge_data:
                    edge  = edge_data.get(0, edge_data[next(iter(edge_data))])
                    geom  = edge.get("geometry", None)
                    if geom is not None:
                        try:
                            coords = list(geom.coords)   # [(lon, lat), ...]
                            for lon_g, lat_g in coords[1:-1]:
                                wg = Waypoint()
                                wg.latitude    = lat_g
                                wg.longitude   = lon_g
                                wg.x           = 0.0
                                wg.y           = 0.0
                                wg.osmid       = 0
                                wg.speed_limit = speed_limit
                                wg.road_name   = street_name
                                waypoints.append(wg)
                        except Exception:
                            pass   # no geometry — straight line is fine
        return waypoints

    def _get_street_name(self, node_id) -> str:
        for _, _, data in self._graph.edges(node_id, data=True):
            name = data.get("name", "")
            if isinstance(name, list):
                return name[0]
            if name:
                return str(name)
        return "Unknown Road"

    def _get_speed_limit(self, node_id) -> float:
        for _, _, data in self._graph.edges(node_id, data=True):
            speed = data.get("maxspeed", None)
            if speed is not None:
                try:
                    if isinstance(speed, list):
                        speed = speed[0]
                    return float(str(speed).split()[0]) * 1000.0 / 3600.0
                except (ValueError, IndexError):
                    pass
        return self._default_speed_kph * 1000.0 / 3600.0

    @staticmethod
    def _compute_distance(waypoints: list[Waypoint]) -> float:
        total = 0.0
        for i in range(1, len(waypoints)):
            dlat = math.radians(waypoints[i].latitude - waypoints[i - 1].latitude)
            dlon = math.radians(waypoints[i].longitude - waypoints[i - 1].longitude)
            a = (
                math.sin(dlat / 2) ** 2
                + math.cos(math.radians(waypoints[i - 1].latitude))
                * math.cos(math.radians(waypoints[i].latitude))
                * math.sin(dlon / 2) ** 2
            )
            total += 6371000.0 * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return total


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RoutePlanner.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
