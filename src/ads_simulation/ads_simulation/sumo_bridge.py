"""
SUMO Bridge Node

The core simulation-ROS2 integration layer. Manages the SUMO process lifecycle,
controls the ego vehicle inside the simulation, and bridges SUMO state to ROS2
topics at each simulation step.

At each tick:
  1. Steps the SUMO simulation forward by one timestep.
  2. Reads the ego vehicle's position, speed, and heading from SUMO.
  3. Publishes it as VehicleState on /vehicle/state.
  4. Reads all nearby vehicles from SUMO (within detection_radius_m).
  5. Publishes them as TrafficVehicleArray on /perception/traffic_vehicles.
  6. Reads the latest VehicleCommand and applies speed/lane changes to SUMO.

The ego vehicle follows the SUMO-internal route set via TraCI's
changeTarget() — destination is updated whenever a new Route is received
from /navigation/route.

Subscribes
----------
/vehicle/command           (ads_interfaces/msg/VehicleCommand)
/navigation/route          (ads_interfaces/msg/Route)

Publishes
---------
/vehicle/state             (ads_interfaces/msg/VehicleState)
/perception/traffic_vehicles  (ads_interfaces/msg/TrafficVehicleArray)
/simulation/status         (std_msgs/String)
"""

import math
import os
import subprocess
import time
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


class SumoBridge(Node):
    """
    Bridges SUMO simulation state to the ROS2 autonomy stack.

    Starts SUMO (or sumo-gui for visualization), inserts the ego vehicle,
    and runs a fixed-rate loop that steps the simulation and exchanges
    state/command data with the rest of the stack.

    The node waits for /map/status == "READY" before starting SUMO so that
    the network file is guaranteed to exist.
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
        self.declare_parameter("max_vehicles", 100)

        self._cache_dir = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._detection_radius = self.get_parameter("detection_radius_m").value
        self._use_gui = self.get_parameter("use_gui").value
        self._max_vehicles = self.get_parameter("max_vehicles").value

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

        self._command_sub = self.create_subscription(
            VehicleCommand, "/vehicle/command", self._on_command, qos
        )
        self._route_sub = self.create_subscription(
            Route, "/navigation/route", self._on_route, 10
        )
        self._map_status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )

        self._latest_command: Optional[VehicleCommand] = None
        self._active_route: Optional[Route] = None
        self._sumo_running = False
        self._ego_spawned = False
        self._traci = None

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

        # Prefer the .sumocfg (includes polygons + settings). Fall back to --net-file.
        use_cfg = self._cfg_path.exists()
        if use_cfg:
            base_args = [sumo_binary, "-c", str(self._cfg_path)]
        else:
            self.get_logger().warn(
                "sumocfg not found — starting SUMO with net-file only (no polygons)."
            )
            base_args = [sumo_binary, "--net-file", str(self._net_path)]

        # Attach viewsettings if available (dark theme, AV-style colors).
        viewsettings_path = Path(__file__).parent.parent / "config" / "viewsettings.xml"
        if not viewsettings_path.exists():
            # Try installed share path.
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory("ads_simulation")
                viewsettings_path = Path(share) / "config" / "viewsettings.xml"
            except Exception:
                pass

        sumo_cmd = base_args + ["--no-step-log", "--no-warnings", "--start"]
        if self._use_gui and viewsettings_path.exists():
            sumo_cmd += ["--gui-settings-file", str(viewsettings_path)]
            self.get_logger().info(f"Viewsettings loaded: '{viewsettings_path}'")

        self.get_logger().info(f"Starting {sumo_binary} ...")

        try:
            traci.start(sumo_cmd)
            self._sumo_running = True
            self._publish_status("RUNNING")
            self.get_logger().info(
                f"SUMO started — cfg={'yes' if use_cfg else 'no'}, "
                f"gui={self._use_gui}"
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
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_command(self, msg: VehicleCommand) -> None:
        self._latest_command = msg

    def _on_route(self, msg: Route) -> None:
        self._active_route = msg
        self.get_logger().info(
            f"Route received: '{msg.start_address}' → '{msg.end_address}', "
            f"{len(msg.waypoints)} waypoints"
        )
        if self._sumo_running and not self._ego_spawned:
            self._spawn_ego()
        elif self._sumo_running and self._ego_spawned:
            self._update_ego_destination()

    # ------------------------------------------------------------------
    # Ego vehicle management
    # ------------------------------------------------------------------

    def _spawn_ego(self) -> None:
        if self._active_route is None or not self._active_route.waypoints:
            return

        route = self._active_route
        start_wp = route.waypoints[0]
        end_wp = route.waypoints[-1]

        try:
            traci = self._traci

            # Find nearest SUMO edges to start and end waypoints.
            start_x, start_y = traci.simulation.convertGeo(
                start_wp.longitude, start_wp.latitude, fromGeo=True
            )
            end_x, end_y = traci.simulation.convertGeo(
                end_wp.longitude, end_wp.latitude, fromGeo=True
            )

            start_edge = traci.simulation.convertRoad(start_x, start_y, isGeo=False)[0]
            end_edge = traci.simulation.convertRoad(end_x, end_y, isGeo=False)[0]

            # Build a SUMO route for the ego vehicle.
            route_id = "ego_route"
            try:
                traci.route.add(route_id, [start_edge, end_edge])
            except Exception:
                # Route may already exist from a prior run.
                pass

            traci.vehicle.add(
                _EGO_ID,
                routeID=route_id,
                typeID=_EGO_TYPE,
                departSpeed=str(self.get_parameter("ego_depart_speed").value),
                departLane="best",
            )

            # Style the ego vehicle distinctively in the GUI.
            traci.vehicle.setColor(_EGO_ID, (0, 200, 255, 255))  # Cyan

            self._ego_spawned = True
            self.get_logger().info(
                f"Ego vehicle spawned at edge '{start_edge}', "
                f"destination edge '{end_edge}'"
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

        if not self._ego_spawned:
            return

        self._apply_command()
        self._publish_ego_state()
        self._publish_traffic()

    def _apply_command(self) -> None:
        cmd = self._latest_command
        if cmd is None or _EGO_ID not in self._traci.vehicle.getIDList():
            return

        try:
            if cmd.emergency_stop:
                self._traci.vehicle.setSpeed(_EGO_ID, 0.0)
                return

            if cmd.target_speed >= 0.0:
                self._traci.vehicle.setSpeed(_EGO_ID, cmd.target_speed)

            # Lane changes — SUMO uses lane index, positive = left, negative = right.
            steer = cmd.steering_angle
            if abs(steer) > 0.1:
                direction = (
                    self._traci.vehicle.LANECHANGE_LEFT
                    if steer > 0
                    else self._traci.vehicle.LANECHANGE_RIGHT
                )
                self._traci.vehicle.changeLane(_EGO_ID, direction, duration=3.0)

        except Exception as exc:
            self.get_logger().warn(f"Command application failed: {exc}", throttle_duration_sec=2.0)

    def _publish_ego_state(self) -> None:
        if _EGO_ID not in self._traci.vehicle.getIDList():
            return

        try:
            x, y = self._traci.vehicle.getPosition(_EGO_ID)
            speed = self._traci.vehicle.getSpeed(_EGO_ID)
            angle_deg = self._traci.vehicle.getAngle(_EGO_ID)  # SUMO: 0=north, CW
            heading_rad = math.radians(90.0 - angle_deg)       # convert to standard (0=east, CCW)

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
            self.get_logger().warn(
                f"Failed to read ego state: {exc}", throttle_duration_sec=2.0
            )

    def _publish_traffic(self) -> None:
        if _EGO_ID not in self._traci.vehicle.getIDList():
            return

        try:
            ego_x, ego_y = self._traci.vehicle.getPosition(_EGO_ID)
        except Exception:
            return

        traffic_msgs = []
        all_ids = self._traci.vehicle.getIDList()

        for vid in all_ids:
            if vid == _EGO_ID:
                continue
            try:
                vx, vy = self._traci.vehicle.getPosition(vid)
                dist = math.sqrt((vx - ego_x) ** 2 + (vy - ego_y) ** 2)
                if dist > self._detection_radius:
                    continue

                vspeed = self._traci.vehicle.getSpeed(vid)
                vangle_deg = self._traci.vehicle.getAngle(vid)
                vheading = math.radians(90.0 - vangle_deg)
                vlane = self._traci.vehicle.getLaneID(vid)
                vtype = self._traci.vehicle.getTypeID(vid)

                tv = TrafficVehicle()
                tv.vehicle_id = vid
                tv.x = vx
                tv.y = vy
                tv.speed = vspeed
                tv.heading = vheading
                tv.lane_id = vlane
                tv.distance_to_ego = dist
                tv.vehicle_type = vtype
                traffic_msgs.append(tv)

            except Exception:
                continue

        traffic_msgs.sort(key=lambda v: v.distance_to_ego)

        arr = TrafficVehicleArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        arr.vehicles = traffic_msgs
        arr.total_simulated = len(all_ids) - 1
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
