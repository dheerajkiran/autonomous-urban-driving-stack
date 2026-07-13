"""
SUMO Bridge Node

Manages the SUMO simulation process lifecycle. Waits for /map/status == "READY",
starts SUMO headless or with GUI, and steps the simulation at a fixed rate.

Subscribes
----------
/map/status  (std_msgs/String)  — waits for "READY" before starting SUMO

Publishes
---------
/simulation/status  (std_msgs/String)
"""

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

        self._cache_dir    = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_filename = self.get_parameter("net_filename").value
        self._cfg_filename = self.get_parameter("cfg_filename").value
        self._step_length  = self.get_parameter("step_length").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._use_gui      = self.get_parameter("use_gui").value

        self._net_path = self._cache_dir / self._net_filename
        self._cfg_path = self._cache_dir / self._cfg_filename

        self._status_pub = self.create_publisher(String, "/simulation/status", 10)

        self._map_status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )

        self._sumo_running: bool      = False
        self._traci: Optional[object] = None

        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info(
            f"SumoBridge initialized — net='{self._net_path}', "
            f"gui={self._use_gui}, rate={self._publish_rate} Hz"
        )

    def _on_map_status(self, msg: String) -> None:
        if msg.data == "READY" and not self._sumo_running:
            self._start_sumo()

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
