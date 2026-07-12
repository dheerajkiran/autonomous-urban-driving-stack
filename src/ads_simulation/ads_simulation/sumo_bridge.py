"""
SUMO Bridge Node

The core simulation-ROS2 integration layer. Manages the SUMO process lifecycle,
controls the ego vehicle, spawns background traffic, and bridges SUMO state to
ROS2 topics at each simulation step.

At each tick:
  1. Steps the SUMO simulation forward by one timestep.
  2. Reads the ego vehicle's position, speed, and heading from SUMO.
  3. Publishes it as VehicleState on /vehicle/state.
  4. Reads all nearby vehicles from SUMO (within detection_radius_m).
  5. Publishes them as TrafficVehicleArray on /perception/traffic_vehicles.
  6. Reads the latest VehicleCommand and applies speed/lane changes to SUMO.
  7. Tops up background traffic to the target vehicle count.

Subscribes
----------
/vehicle/command           (ads_interfaces/msg/VehicleCommand)
/navigation/route          (ads_interfaces/msg/Route)

Publishes
---------
/vehicle/state             (ads_interfaces/msg/VehicleState)
/perception/traffic_vehicles  (ads_interfaces/msg/TrafficVehicleArray)
/simulation/status         (std_msgs/String)
/simulation/traffic_count  (std_msgs/String)
"""

import math
import random
import string
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ads_interfaces.msg import (
    Route,
    TrafficVehicle,
    TrafficVehicleArray,
    VehicleCommand,
    VehicleState,
)

_EGO_ID = "ego"
_EGO_TYPE = "passenger"

_BG_VEHICLE_TYPES = [
    ("car", 0.65),
    ("van", 0.15),
    ("truck", 0.10),
    ("bus", 0.05),
    ("moto", 0.05),
]

_BG_COLORS = [
    (200, 200, 200, 255),
    (180, 0,   0,   255),
    (0,   0,   180, 255),
    (0,   140, 0,   255),
    (230, 180, 0,   255),
    (100, 100, 100, 255),
    (255, 140, 0,   255),
]


def _weighted_choice(choices):
    r = random.random()
    cumulative = 0.0
    for v, w in choices:
        cumulative += w
        if r <= cumulative:
            return v
    return choices[-1][0]


def _rand_id():
    suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))
    return f"bg_{suffix}"


class SumoBridge(Node):
    """
    Bridges SUMO simulation state to the ROS2 autonomy stack.

    Starts SUMO (or sumo-gui for visualization), inserts the ego vehicle,
    maintains background traffic, and runs a fixed-rate loop that steps the
    simulation and exchanges state/command data with the rest of the stack.
    """

    def __init__(self) -> None:
        super().__init__("sumo_bridge")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        self.declare_parameter("net_filename", "tempe.net.xml")
        self.declare_parameter("cfg_filename", "tempe.sumocfg")
        self.declare_parameter("step_length", 0.05)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("detection_radius_m", 80.0)
        self.declare_parameter("use_gui", True)
        self.declare_parameter("ego_depart_speed", 0.0)
        self.declare_parameter("target_vehicle_count", 60)

        self._cache_dir = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._detection_radius = self.get_parameter("detection_radius_m").value
        self._use_gui = self.get_parameter("use_gui").value
        self._target_vehicle_count = self.get_parameter("target_vehicle_count").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._state_pub = self.create_publisher(VehicleState, "/vehicle/state", qos)
        self._traffic_pub = self.create_publisher(
            TrafficVehicleArray, "/perception/traffic_vehicles", qos
        )
        self._status_pub = self.create_publisher(String, "/simulation/status", 10)
        self._count_pub = self.create_publisher(String, "/simulation/traffic_count", 10)

        self._command_sub = self.create_subscription(
            VehicleCommand, "/vehicle/command", self._on_command, qos
        )
        self._route_sub = self.create_subscription(
            Route, "/navigation/route", self._on_route, 10
        )
        self._go_sub = self.create_subscription(
            String, "/navigation/mission_confirm", self._on_mission_confirm, 10
        )
        self._map_status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )

        self._latest_command: Optional[VehicleCommand] = None
        self._active_route: Optional[Route] = None
        self._sumo_running = False
        self._ego_spawned = False
        self._traci = None
        self._valid_edges: list = []
        self._spawn_counter = 0
        self._tick_count = 0

        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info(
            f"SumoBridge initialized — net='{self._net_path}', "
            f"gui={self._use_gui}, rate={self._publish_rate} Hz"
        )

    # ------------------------------------------------------------------
    # Map readiness gate
    # ------------------------------------------------------------------

    def _on_map_status(self, msg: String) -> None:
        if msg.data == "READY" and not self._sumo_running:
            self._start_sumo()

    # ------------------------------------------------------------------
    # SUMO lifecycle
    # ------------------------------------------------------------------

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
            self.get_logger().error(
                "traci not found. Install SUMO: sudo apt install sumo sumo-tools"
            )
            return

        sumo_binary = "sumo-gui" if self._use_gui else "sumo"

        use_cfg = self._cfg_path.exists()
        if use_cfg:
            base_args = [sumo_binary, "-c", str(self._cfg_path)]
        else:
            self.get_logger().warn("sumocfg not found — starting with net-file only.")
            base_args = [sumo_binary, "--net-file", str(self._net_path)]

        viewsettings_path = Path(__file__).parent.parent / "config" / "viewsettings.xml"
        if not viewsettings_path.exists():
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory("ads_simulation")
                viewsettings_path = Path(share) / "config" / "viewsettings.xml"
            except Exception:
                pass

        sumo_cmd = base_args + [
            "--no-step-log", "--no-warnings", "--start",
            "--ignore-route-errors",
            "--tls.all-off",
        ]
        if self._use_gui and viewsettings_path.exists():
            sumo_cmd += ["--gui-settings-file", str(viewsettings_path)]
            self.get_logger().info(f"Viewsettings loaded: '{viewsettings_path}'")

        self.get_logger().info(f"Starting {sumo_binary} ...")

        try:
            traci.start(sumo_cmd)
            self._sumo_running = True
            self._define_bg_vehicle_types()
            self._cache_valid_edges()
            self._publish_status("RUNNING")
            self.get_logger().info(
                f"SUMO started — cfg={'yes' if use_cfg else 'no'}, "
                f"gui={self._use_gui}, edges={len(self._valid_edges)}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to start SUMO: {exc}")
            self._publish_status("ERROR")

    def _stop_sumo(self) -> None:
        if self._sumo_running and self._traci is not None:
            try:
                self._traci.close()
            except Exception:
                pass
            self._sumo_running = False
            self.get_logger().info("SUMO stopped.")

    # ------------------------------------------------------------------
    # Background traffic setup
    # ------------------------------------------------------------------

    def _define_bg_vehicle_types(self) -> None:
        existing = self._traci.vehicletype.getIDList()
        for vtype, _ in _BG_VEHICLE_TYPES:
            if vtype not in existing:
                self._traci.vehicletype.copy("DEFAULT_VEHTYPE", vtype)

        if _EGO_TYPE not in existing:
            self._traci.vehicletype.copy("DEFAULT_VEHTYPE", _EGO_TYPE)

        # Faster acceleration so ego recovers from junction stops quickly.
        self._traci.vehicletype.setAccel(_EGO_TYPE, 5.0)
        self._traci.vehicletype.setDecel(_EGO_TYPE, 6.0)

    def _cache_valid_edges(self) -> None:
        all_edges = self._traci.edge.getIDList()
        self._valid_edges = [
            e for e in all_edges
            if not e.startswith(":")
            and self._traci.edge.getLaneNumber(e) > 0
        ]
        self.get_logger().info(
            f"Edge cache built — {len(self._valid_edges)} valid edges"
        )

    def _spawn_bg_vehicle(self) -> None:
        if len(self._valid_edges) < 2:
            return
        src = random.choice(self._valid_edges)
        dst = random.choice(self._valid_edges)
        if src == dst:
            return
        vid = _rand_id()
        route_id = f"route_{vid}"
        vtype = _weighted_choice(_BG_VEHICLE_TYPES)
        color = random.choice(_BG_COLORS)
        try:
            self._traci.route.add(route_id, [src, dst])
            self._traci.vehicle.add(
                vid, routeID=route_id, typeID=vtype,
                departSpeed="speedLimit", departLane="random",
            )
            self._traci.vehicle.setColor(vid, color)
            self._spawn_counter += 1
        except Exception:
            try:
                self._traci.route.remove(route_id)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_command(self, msg: VehicleCommand) -> None:
        self._latest_command = msg

    def _on_route(self, msg: Route) -> None:
        self._active_route = msg
        self.get_logger().info(
            f"Route ready: '{msg.start_address}' → '{msg.end_address}', "
            f"{len(msg.waypoints)} waypoints — press SPACE in viewer to start ego"
        )

    def _on_mission_confirm(self, msg: String) -> None:
        if not self._sumo_running:
            self.get_logger().warn("GO received but SUMO is not running yet.")
            return
        if self._active_route is None:
            self.get_logger().warn("GO received but no route is loaded yet.")
            return
        if self._ego_spawned:
            # Remove current ego so we can respawn on the new route.
            try:
                self._traci.vehicle.remove(_EGO_ID)
            except Exception:
                pass
            self._ego_spawned = False
        self.get_logger().info("GO — spawning ego on current route.")
        self._spawn_ego()

    # ------------------------------------------------------------------
    # Ego vehicle management
    # ------------------------------------------------------------------

    def _nearest_real_edge(self, x: float, y: float) -> str:
        """Return the nearest non-internal edge to (x, y) in SUMO coordinates."""
        candidate = self._traci.simulation.convertRoad(x, y, isGeo=False)[0]
        if not candidate.startswith(":"):
            return candidate
        # Fall back to closest edge in our valid_edges list.
        best_edge, best_dist = None, float("inf")
        for edge in self._valid_edges:
            try:
                shape = self._traci.lane.getShape(f"{edge}_0")
                ex, ey = shape[0]
                dist = math.sqrt((ex - x) ** 2 + (ey - y) ** 2)
                if dist < best_dist:
                    best_dist, best_edge = dist, edge
            except Exception:
                continue
        return best_edge or self._valid_edges[0]

    def _build_route_edges(self, waypoints) -> list:
        """Convert A* waypoints to a connected SUMO edge sequence.

        Only keeps waypoints that are at least 150 m apart so that each
        findRoute sub-call spans a meaningful stretch of road and avoids
        routing around a block when two waypoints fall on neighbouring
        junction edges.
        """
        MIN_SPACING = 150.0  # metres

        # Project waypoints and thin them out.
        # Skip osmid==0 entries — those are display-only geometry interpolation points
        # added by route_planner. Using them for SUMO routing causes nearest_real_edge
        # to snap to wrong edges on parallel roads.
        sparse: list = []   # [(x, y, edge)]
        for wp in waypoints:
            if wp.osmid == 0:
                continue
            x, y = self._traci.simulation.convertGeo(
                wp.longitude, wp.latitude, fromGeo=True
            )
            if sparse:
                px, py, _ = sparse[-1]
                if math.sqrt((x - px) ** 2 + (y - py) ** 2) < MIN_SPACING:
                    continue
            edge = self._nearest_real_edge(x, y)
            if edge and (not sparse or edge != sparse[-1][2]):
                sparse.append((x, y, edge))

        if not sparse:
            return []

        # Always include the last waypoint as the final destination.
        last_wp = waypoints[-1]
        lx, ly = self._traci.simulation.convertGeo(
            last_wp.longitude, last_wp.latitude, fromGeo=True
        )
        last_edge = self._nearest_real_edge(lx, ly)
        if last_edge and last_edge != sparse[-1][2]:
            sparse.append((lx, ly, last_edge))

        wp_edges = [e for _, _, e in sparse]

        if len(wp_edges) < 2:
            return wp_edges

        # Stitch sub-routes between each consecutive pair.
        full_edges: list = []
        for i in range(len(wp_edges) - 1):
            try:
                result = self._traci.simulation.findRoute(
                    wp_edges[i], wp_edges[i + 1], "", -1.0, 0
                )
                sub = list(result.edges)
            except Exception:
                sub = [wp_edges[i], wp_edges[i + 1]]

            if not sub:
                continue
            if full_edges and full_edges[-1] == sub[0]:
                full_edges.extend(sub[1:])
            else:
                full_edges.extend(sub)

        return full_edges if full_edges else wp_edges

    def _spawn_ego(self) -> None:
        if self._active_route is None or not self._active_route.waypoints:
            return

        try:
            traci = self._traci

            if _EGO_TYPE not in traci.vehicletype.getIDList():
                traci.vehicletype.copy("DEFAULT_VEHTYPE", _EGO_TYPE)

            route_edges = self._build_route_edges(self._active_route.waypoints)
            if len(route_edges) < 2:
                self.get_logger().error("Could not build a valid SUMO route from waypoints.")
                return

            route_id = "ego_route"
            try:
                traci.route.add(route_id, route_edges)
            except Exception:
                pass

            traci.vehicle.add(
                _EGO_ID,
                routeID=route_id,
                typeID=_EGO_TYPE,
                departSpeed="speedLimit",
                departLane="best",
            )
            traci.vehicle.setColor(_EGO_ID, (0, 200, 255, 255))

            # Clear junction right-of-way bit (8) so ego doesn't pause at minor roads.
            # Bits 1+2+4 (safe speed, accel, decel) remain set. TLS already off globally.
            traci.vehicle.setSpeedMode(_EGO_ID, 7)

            self._ego_spawned = True
            self.get_logger().info(
                f"Ego spawned — {len(route_edges)} SUMO edges following A* plan "
                f"({len(self._active_route.waypoints)} waypoints)"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn ego vehicle: {exc}")

    def _update_ego_destination(self) -> None:
        if self._active_route is None or not self._active_route.waypoints:
            return
        try:
            end_wp = self._active_route.waypoints[-1]
            end_x, end_y = self._traci.simulation.convertGeo(
                end_wp.longitude, end_wp.latitude, fromGeo=True
            )
            end_edge = self._traci.simulation.convertRoad(end_x, end_y, isGeo=False)[0]
            self._traci.vehicle.changeTarget(_EGO_ID, end_edge)
            self.get_logger().info(f"Ego destination updated to edge '{end_edge}'")
        except Exception as exc:
            self.get_logger().warn(f"Could not update ego destination: {exc}")

    # ------------------------------------------------------------------
    # Main simulation tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if not self._sumo_running:
            return

        try:
            self._traci.simulationStep()
        except Exception as exc:
            self.get_logger().error(f"SUMO step failed: {exc}")
            self._sumo_running = False
            return

        self._tick_count += 1

        # One getIDList call per tick — reused everywhere below.
        vehicle_ids = self._traci.vehicle.getIDList()

        if self._ego_spawned:
            if _EGO_ID not in vehicle_ids:
                self._ego_spawned = False
                self._publish_status("EGO_ARRIVED")
                self.get_logger().info("Ego vehicle reached destination.")
            else:
                self._apply_command()
                self._publish_ego_state()
                if self._target_vehicle_count > 0:
                    self._publish_traffic(vehicle_ids)

        if self._target_vehicle_count > 0 and self._tick_count % 20 == 0:
            self._top_up_traffic(vehicle_ids)

    def _top_up_traffic(self, vehicle_ids) -> None:
        if not self._valid_edges:
            return
        try:
            current = len(vehicle_ids)
            deficit = self._target_vehicle_count - current
            for _ in range(min(deficit, 3)):
                self._spawn_bg_vehicle()
            msg = String()
            msg.data = f"Active: {current} | Target: {self._target_vehicle_count}"
            self._count_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Traffic top-up error: {exc}", throttle_duration_sec=5.0)

    def _apply_command(self) -> None:
        # Caller guarantees ego is alive — no getIDList() needed here.
        cmd = self._latest_command
        if cmd is None:
            return
        try:
            if cmd.emergency_stop:
                self._traci.vehicle.setSpeed(_EGO_ID, 0.0)
                return
            if cmd.target_speed >= 0.0:
                self._traci.vehicle.setSpeed(_EGO_ID, cmd.target_speed)
            steer = cmd.steering_angle
            if abs(steer) > 0.1:
                direction = (
                    self._traci.vehicle.LANECHANGE_LEFT
                    if steer > 0
                    else self._traci.vehicle.LANECHANGE_RIGHT
                )
                self._traci.vehicle.changeLane(_EGO_ID, direction, duration=3.0)
        except Exception as exc:
            self.get_logger().warn(f"Command failed: {exc}", throttle_duration_sec=2.0)

    def _publish_ego_state(self) -> None:
        try:
            x, y      = self._traci.vehicle.getPosition(_EGO_ID)
            speed     = self._traci.vehicle.getSpeed(_EGO_ID)
            angle_deg = self._traci.vehicle.getAngle(_EGO_ID)
            heading_rad = math.radians(90.0 - angle_deg)

            msg = VehicleState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.x = x
            msg.y = y
            msg.speed = speed
            msg.heading = heading_rad
            msg.yaw_rate = 0.0
            msg.acceleration = 0.0
            msg.throttle = min(1.0, speed / 15.0)
            msg.brake = 0.0
            msg.steering_angle = 0.0
            msg.gear = "DRIVE" if speed > 0.1 else "PARK"
            msg.is_autonomous = True
            self._state_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to read ego state: {exc}", throttle_duration_sec=2.0)

    def _publish_traffic(self, vehicle_ids) -> None:
        try:
            ego_x, ego_y = self._traci.vehicle.getPosition(_EGO_ID)
        except Exception:
            return

        traffic_msgs = []
        for vid in vehicle_ids:
            if vid == _EGO_ID:
                continue
            try:
                vx, vy = self._traci.vehicle.getPosition(vid)
                dist = math.sqrt((vx - ego_x) ** 2 + (vy - ego_y) ** 2)
                if dist > self._detection_radius:
                    continue
                tv = TrafficVehicle()
                tv.vehicle_id = vid
                tv.x = vx
                tv.y = vy
                tv.speed = self._traci.vehicle.getSpeed(vid)
                tv.heading = math.radians(90.0 - self._traci.vehicle.getAngle(vid))
                tv.lane_id = self._traci.vehicle.getLaneID(vid)
                tv.distance_to_ego = dist
                tv.vehicle_type = self._traci.vehicle.getTypeID(vid)
                traffic_msgs.append(tv)
            except Exception:
                continue

        arr = TrafficVehicleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        arr.vehicles = sorted(traffic_msgs, key=lambda v: v.distance_to_ego)
        arr.total_simulated = len(vehicle_ids) - 1
        arr.detection_radius_m = self._detection_radius
        self._traffic_pub.publish(arr)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

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
