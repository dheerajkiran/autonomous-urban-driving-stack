"""
Map Loader Node

Downloads and caches the Tempe, AZ road network from OpenStreetMap using osmnx.
On first run, downloads the full city graph and saves it locally as GraphML.
On subsequent runs, loads from cache (fast, ~1 second).

Also runs netconvert to generate the SUMO-compatible network file (.net.xml)
required by sumo_bridge for simulation.

Publishes
---------
/map/status  (std_msgs/String)  — "LOADING" | "READY" | "ERROR"

Parameters
----------
cache_dir       : path where OSM + SUMO files are stored (default: ~/ads_map_cache)
city_query      : osmnx place query string (default: "Tempe, Arizona, USA")
network_type    : osmnx network type — "drive" for drivable roads only
force_reload    : if true, re-downloads even if cache exists
"""

import os
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapLoader(Node):
    """
    Loads and caches the Tempe road network for use by route_planner and sumo_bridge.

    On startup it checks for a local cache. If the cache is missing or
    force_reload is set, it downloads the OSM graph via osmnx and converts it
    to SUMO format with netconvert. Downstream nodes should wait for the
    /map/status = "READY" signal before requesting routes.
    """

    _STATUS_LOADING = "LOADING"
    _STATUS_READY = "READY"
    _STATUS_ERROR = "ERROR"

    def __init__(self) -> None:
        super().__init__("map_loader")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        self.declare_parameter("city_query", "Tempe, Arizona, USA")
        self.declare_parameter("network_type", "drive")
        self.declare_parameter("force_reload", False)

        self._cache_dir = Path(self.get_parameter("cache_dir").value)
        self._city_query = self.get_parameter("city_query").value
        self._network_type = self.get_parameter("network_type").value
        self._force_reload = self.get_parameter("force_reload").value

        self._cache_dir.mkdir(parents=True, exist_ok=True)

        self._graphml_path = self._cache_dir / "tempe_drive.graphml"
        self._osm_path = self._cache_dir / "tempe.osm"
        self._net_path = self._cache_dir / "tempe.net.xml"

        self._status_pub = self.create_publisher(String, "/map/status", 10)

        # Defer heavy work to after the node is spinning so logs appear cleanly.
        self.create_timer(0.1, self._bootstrap, clock=self.get_clock())
        self._bootstrapped = False

        self.get_logger().info(
            f"MapLoader initialized — cache='{self._cache_dir}', query='{self._city_query}'"
        )

    # ------------------------------------------------------------------
    # Bootstrap (runs once after spin starts)
    # ------------------------------------------------------------------

    def _bootstrap(self) -> None:
        if self._bootstrapped:
            return
        self._bootstrapped = True
        self._publish_status(self._STATUS_LOADING)
        try:
            self._ensure_graph()
            self._ensure_sumo_network()
            self._publish_status(self._STATUS_READY)
            self.get_logger().info("Map ready — road network and SUMO network available.")
        except Exception as exc:
            self.get_logger().error(f"Map loading failed: {exc}")
            self._publish_status(self._STATUS_ERROR)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"Map status: {status}")

    # ------------------------------------------------------------------
    # OSM graph
    # ------------------------------------------------------------------

    def _ensure_graph(self) -> None:
        if self._graphml_path.exists() and not self._force_reload:
            self.get_logger().info(
                f"OSM graph cache found — loading from '{self._graphml_path}'"
            )
            return

        self.get_logger().info(
            f"Downloading OSM road network for '{self._city_query}' — this may take 30-60 seconds..."
        )

        try:
            import osmnx as ox
        except ImportError:
            raise RuntimeError(
                "osmnx is not installed. Run: pip install osmnx"
            )

        ox.settings.log_console = False
        ox.settings.use_cache = True

        G = ox.graph_from_place(self._city_query, network_type=self._network_type)

        # Save GraphML for route_planner reuse.
        ox.save_graphml(G, filepath=str(self._graphml_path))

        # Save raw OSM XML for netconvert.
        ox.save_graph_xml(G, filepath=str(self._osm_path))

        node_count = len(G.nodes)
        edge_count = len(G.edges)
        self.get_logger().info(
            f"OSM graph downloaded — {node_count:,} nodes, {edge_count:,} edges"
        )

    # ------------------------------------------------------------------
    # SUMO network
    # ------------------------------------------------------------------

    def _ensure_sumo_network(self) -> None:
        if self._net_path.exists() and not self._force_reload:
            self.get_logger().info(
                f"SUMO network cache found — '{self._net_path}'"
            )
            return

        if not self._osm_path.exists():
            raise RuntimeError(
                f"OSM file not found at '{self._osm_path}'. Re-run with force_reload:=true."
            )

        self.get_logger().info("Converting OSM → SUMO network with netconvert...")

        result = subprocess.run(
            [
                "netconvert",
                "--osm-files", str(self._osm_path),
                "--output-file", str(self._net_path),
                "--geometry.remove",
                "--roundabouts.guess",
                "--ramps.guess",
                "--junctions.join",
                "--tls.guess-signals",
                "--tls.discard-simple",
                "--tls.join",
                "--output.street-names",
                "--output.original-names",
                "--osm.sidewalks", "false",
                "--osm.crossings", "false",
            ],
            capture_output=True,
            text=True,
            timeout=300,
        )

        if result.returncode != 0:
            raise RuntimeError(
                f"netconvert failed (exit {result.returncode}):\n{result.stderr[-500:]}"
            )

        self.get_logger().info(f"SUMO network generated — '{self._net_path}'")

    # ------------------------------------------------------------------
    # Public accessors (used by other nodes in the same process or via service)
    # ------------------------------------------------------------------

    @property
    def graphml_path(self) -> Path:
        return self._graphml_path

    @property
    def net_path(self) -> Path:
        return self._net_path

    @property
    def cache_dir(self) -> Path:
        return self._cache_dir


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MapLoader.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
