"""
Vehicle State Monitor

Diagnostic subscriber for the /vehicle/state topic. Tracks rolling statistics,
detects significant state transitions, and emits structured log summaries at a
configurable interval.

In the full ADS pipeline this node runs alongside the operational nodes as a
passive observer — it never commands the vehicle and does not affect system
behaviour. It is the first integration smoke test: if this node logs coherent
state, the publisher and message definition are working correctly.

Subscribes
----------
/vehicle/state  (ads_interfaces/msg/VehicleState)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ads_interfaces.msg import VehicleState


class VehicleStateMonitor(Node):
    """
    Passive diagnostic observer for vehicle state.

    Tracks:
    - Message throughput (to detect publisher drops)
    - Peak speed and total distance travelled
    - Gear transitions (logged at INFO level for visibility)
    - Rolling summary emitted every <log_interval> seconds

    All state is reset on node startup — this node carries no persistent state
    between launches.
    """

    _STATIONARY_THRESHOLD_MPS: float = 0.1  # Below this speed the vehicle is "stopped"

    def __init__(self) -> None:
        super().__init__("vehicle_state_monitor")

        self.declare_parameter("topic_name", "/vehicle/state")
        self.declare_parameter("log_interval", 2.0)

        self._topic_name: str = self.get_parameter("topic_name").value
        self._log_interval: float = self.get_parameter("log_interval").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            VehicleState,
            self._topic_name,
            self._on_vehicle_state,
            qos,
        )
        self._summary_timer = self.create_timer(self._log_interval, self._emit_summary)

        self._reset_statistics()

        self.get_logger().info(
            f"VehicleStateMonitor ready — subscribed to '{self._topic_name}', "
            f"summary every {self._log_interval:.1f} s"
        )

    # ------------------------------------------------------------------
    # Statistics bookkeeping
    # ------------------------------------------------------------------

    def _reset_statistics(self) -> None:
        self._msg_count: int = 0
        self._peak_speed: float = 0.0
        self._total_distance: float = 0.0
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_gear: str = ""
        self._last_speed: float = 0.0

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        self._msg_count += 1

        # Distance integration via Euclidean delta between consecutive poses.
        if self._last_x is not None and self._last_y is not None:
            dx = msg.x - self._last_x
            dy = msg.y - self._last_y
            self._total_distance += math.hypot(dx, dy)

        self._last_x = msg.x
        self._last_y = msg.y
        self._peak_speed = max(self._peak_speed, msg.speed)
        self._last_speed = msg.speed

        if msg.gear != self._last_gear:
            self.get_logger().info(
                f"Gear transition: '{self._last_gear}' → '{msg.gear}'"
            )
            self._last_gear = msg.gear

        self.get_logger().debug(
            f"[#{self._msg_count:05d}] "
            f"speed={msg.speed:.2f} m/s | "
            f"accel={msg.acceleration:.3f} m/s² | "
            f"pos=({msg.x:.2f}, {msg.y:.2f}) m | "
            f"heading={math.degrees(msg.heading):.1f}° | "
            f"throttle={msg.throttle:.2f} | brake={msg.brake:.2f} | "
            f"gear={msg.gear} | autonomous={msg.is_autonomous}"
        )

    # ------------------------------------------------------------------
    # Periodic summary
    # ------------------------------------------------------------------

    def _emit_summary(self) -> None:
        if self._msg_count == 0:
            self.get_logger().warn(
                f"No messages received on '{self._topic_name}'. "
                "Verify that vehicle_state_publisher is running."
            )
            return

        motion_state = (
            "STATIONARY"
            if self._last_speed < self._STATIONARY_THRESHOLD_MPS
            else "MOVING"
        )

        self.get_logger().info(
            f"[SUMMARY] "
            f"msgs={self._msg_count} | "
            f"state={motion_state} | "
            f"speed={self._last_speed:.2f} m/s | "
            f"peak={self._peak_speed:.2f} m/s | "
            f"distance={self._total_distance:.1f} m | "
            f"gear={self._last_gear}"
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VehicleStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested — stopping VehicleStateMonitor.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
