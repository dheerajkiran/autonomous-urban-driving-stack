"""
SUMO Bridge Node

Manages the SUMO simulation process lifecycle and background traffic.
Bridges simulation state to ROS2 topics at each simulation step.

Waits for /map/status == "READY" before starting SUMO, then steps the
simulation at a fixed rate and maintains background vehicle count.

Subscribes
----------
/map/status  (std_msgs/String)  — waits for "READY" before starting SUMO

Publishes
---------
/simulation/status         (std_msgs/String)
/simulation/traffic_count  (std_msgs/String)
"""

import random
import string
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_BG_VEHICLE_TYPES = [
    ("car",   0.65),
    ("van",   0.15),
    ("truck", 0.10),
    ("bus",   0.05),
    ("moto",  0.05),
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

    Starts SUMO (headless or GUI), maintains background traffic, and runs
    a fixed-rate loop that steps the simulation forward each tick.
    """

    def __init__(self) -> None:
        super().__init__("sumo_bridge")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        self.declare_parameter("net_filename", "tempe.net.xml")
        self.declare_parameter("cfg_filename", "tempe.sumocfg")
        self.declare_parameter("step_length", 0.05)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("use_gui", False)
        self.declare_parameter("target_vehicle_count", 60)

        self._cache_dir       = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename    = self.get_parameter("net_filename").value
        self._cfg_filename    = self.get_parameter("cfg_filename").value
        self._step_length     = self.get_parameter("step_length").value
        self._publish_rate    = self.get_parameter("publish_rate").value
        self._use_gui         = self.get_parameter("use_gui").value
        self._target_vehicle_count = self.get_parameter("target_vehicle_count").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        self._status_pub = self.create_publisher(String, "/simulation/status", 10)
        self._count_pub  = self.create_publisher(String, "/simulation/traffic_count", 10)

        self._map_status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )

        self._sumo_running: bool        = False
        self._traci: Optional[object]   = None
        self._valid_edges: list         = []
        self._spawn_counter: int        = 0
        self._tick_count: int           = 0

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
            f"--step-length={self._step_length}",
            "--time-to-teleport=-1",
        ]
        if self._use_gui and viewsettings_path.exists():
            sumo_cmd += ["--gui-settings-file", str(viewsettings_path)]

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
    # Background traffic
    # ------------------------------------------------------------------

    def _define_bg_vehicle_types(self) -> None:
        existing = self._traci.vehicletype.getIDList()
        for vtype, _ in _BG_VEHICLE_TYPES:
            if vtype not in existing:
                self._traci.vehicletype.copy("DEFAULT_VEHTYPE", vtype)

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
        vid      = _rand_id()
        route_id = f"route_{vid}"
        vtype    = _weighted_choice(_BG_VEHICLE_TYPES)
        color    = random.choice(_BG_COLORS)
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
        vehicle_ids = self._traci.vehicle.getIDList()

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
