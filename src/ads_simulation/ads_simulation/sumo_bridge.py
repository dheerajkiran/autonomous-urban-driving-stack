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
"""

import json
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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

        self._cache_dir    = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length  = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._use_gui      = self.get_parameter("use_gui").value
        self._edge_search_radius_m = self.get_parameter("edge_search_radius_m").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        self._status_pub = self.create_publisher(String, "/simulation/status", 10)

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

    def _nearest_edge(self, lat: float, lon: float):
        """Return the closest drivable SUMO edge to a lat/lon, or None."""
        x, y = self._net.convertLonLat2XY(lon, lat)
        candidates = self._net.getNeighboringEdges(
            x, y, r=self._edge_search_radius_m, includeJunctions=False
        )
        candidates = [
            (edge, dist) for edge, dist in candidates
            if edge.allows("passenger") and not edge.getID().startswith(":")
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda pair: pair[1])
        return candidates[0][0]

    def _spawn_ego(self, goal: dict) -> None:
        if not self._load_net():
            return

        start_edge = self._nearest_edge(goal["start_lat"], goal["start_lon"])
        end_edge = self._nearest_edge(goal["end_lat"], goal["end_lon"])
        if start_edge is None or end_edge is None:
            self.get_logger().error(
                f"No drivable edge within {self._edge_search_radius_m}m of the S or E pin."
            )
            return

        if start_edge.getID() == end_edge.getID():
            edges = [start_edge]
        else:
            path, _cost = self._net.getShortestPath(start_edge, end_edge, vClass="passenger")
            if path is None:
                self.get_logger().error(
                    f"No connected route from edge '{start_edge.getID()}' "
                    f"to '{end_edge.getID()}'."
                )
                return
            edges = path

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
        except Exception as exc:
            self.get_logger().error(f"SUMO step failed: {exc}")
            self._sumo_running = False

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
