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

        self._cache_dir = Path(self.get_parameter("cache_dir").value).expanduser()
        self._city_query = self.get_parameter("city_query").value
        self._network_type = self.get_parameter("network_type").value
        self._force_reload = self.get_parameter("force_reload").value

        self._cache_dir.mkdir(parents=True, exist_ok=True)

        self._graphml_path = self._cache_dir / "tempe_drive.graphml"
        self._osm_path     = self._cache_dir / "tempe.osm"
        self._net_path     = self._cache_dir / "tempe.net.xml"
        self._poly_path    = self._cache_dir / "tempe.poly.xml"
        self._cfg_path     = self._cache_dir / "tempe.sumocfg"

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
            self._ensure_poly_file()
            self._ensure_sumo_config()
            self._publish_status(self._STATUS_READY)
            self.get_logger().info("Map ready — road network, polygons, and SUMO config available.")
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
        both_cached = self._graphml_path.exists() and self._osm_path.exists()
        if both_cached and not self._force_reload:
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

        # Download unsimplified — required for OSM XML export (netconvert/SUMO).
        self.get_logger().info("Downloading unsimplified graph for SUMO network generation...")
        G_raw = ox.graph_from_place(
            self._city_query, network_type=self._network_type, simplify=False
        )
        ox.save_graph_xml(G_raw, filepath=str(self._osm_path))
        self.get_logger().info(
            f"Raw OSM XML saved — {len(G_raw.nodes):,} nodes, {len(G_raw.edges):,} edges"
        )

        # Simplify for efficient A* route planning and save as GraphML.
        self.get_logger().info("Simplifying graph for route planning...")
        G = ox.simplify_graph(G_raw)
        ox.save_graphml(G, filepath=str(self._graphml_path))
        self.get_logger().info(
            f"Simplified graph saved — {len(G.nodes):,} nodes, {len(G.edges):,} edges"
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
    # OSM polygons (buildings, parks, water bodies)
    # ------------------------------------------------------------------

    def _ensure_poly_file(self) -> None:
        if self._poly_path.exists() and not self._force_reload:
            self.get_logger().info(f"Polygon cache found — '{self._poly_path}'")
            return

        if not self._osm_path.exists():
            self.get_logger().warn("OSM file missing — skipping polygon generation.")
            return

        self.get_logger().info("Generating building/park/water polygons with polyconvert...")

        result = subprocess.run(
            [
                "polyconvert",
                "--net-file",    str(self._net_path),
                "--osm-files",   str(self._osm_path),
                "--output-file", str(self._poly_path),
                "--osm.keep-full-type",
                "--type-file",   "/usr/share/sumo/data/typemap/osmPolyconvert.typ.xml",
            ],
            capture_output=True,
            text=True,
            timeout=120,
        )

        if result.returncode != 0:
            # Polygon file is optional — log warning but don't fail the whole map load.
            self.get_logger().warn(
                f"polyconvert exited with code {result.returncode} — "
                "simulation will run without building polygons.\n"
                f"{result.stderr[-300:]}"
            )
        else:
            self.get_logger().info(f"Polygon file generated — '{self._poly_path}'")

    # ------------------------------------------------------------------
    # SUMO config file (.sumocfg)
    # ------------------------------------------------------------------

    def _ensure_sumo_config(self) -> None:
        if self._cfg_path.exists() and not self._force_reload:
            self.get_logger().info(f"SUMO config cache found — '{self._cfg_path}'")
            return

        additional = str(self._poly_path) if self._poly_path.exists() else ""

        cfg_xml = f"""<?xml version="1.0" encoding="utf-8"?>
<configuration>
    <input>
        <net-file value="{self._net_path}"/>
        {"<additional-files value=" + chr(34) + additional + chr(34) + "/>" if additional else "<!-- no polygon file -->"}
    </input>
    <time>
        <step-length value="0.05"/>
        <begin value="0"/>
    </time>
    <processing>
        <collision.action value="warn"/>
        <time-to-teleport value="60"/>
        <waiting-time-memory value="100"/>
    </processing>
    <report>
        <no-step-log value="true"/>
        <no-warnings value="true"/>
        <verbose value="false"/>
    </report>
</configuration>
"""
        self._cfg_path.write_text(cfg_xml)
        self.get_logger().info(f"SUMO config written — '{self._cfg_path}'")

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
    def cfg_path(self) -> Path:
        return self._cfg_path

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
