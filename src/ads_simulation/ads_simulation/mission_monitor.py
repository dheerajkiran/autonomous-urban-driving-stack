"""
Mission Monitor Node

Prints ego vehicle position and speed every 10 seconds.
Announces when the vehicle reaches its destination (SUMO removes
the vehicle on arrival, so the topic goes silent).

Run this in a separate terminal while the full stack is running.

Usage
-----
  ros2 run ads_simulation mission_monitor
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, VehicleState


class MissionMonitor(Node):

    def __init__(self) -> None:
        super().__init__("mission_monitor")

        self._start_address = ""
        self._end_address = ""
        self._start_x: float | None = None
        self._start_y: float | None = None
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._last_speed: float = 0.0
        self._distance_traveled: float = 0.0
        self._last_msg_time: float = 0.0
        self._speed_zero_since: float = 0.0
        self._arrived = False
        self._mission_active = False

        self.create_subscription(VehicleState, "/vehicle/state", self._on_state, 10)
        self.create_subscription(Route, "/navigation/route", self._on_route, 10)

        # Print update every 10 seconds.
        self.create_timer(10.0, self._print_update)
        # Check for arrival every 2 seconds.
        self.create_timer(2.0, self._check_arrival)

        self.get_logger().info("MissionMonitor ready — waiting for a route.")

    def _on_route(self, msg: Route) -> None:
        self._start_address = msg.start_address
        self._end_address = msg.end_address
        self._mission_active = True
        self._arrived = False
        self._distance_traveled = 0.0
        self._start_x = None
        self._start_y = None
        self.get_logger().info(
            f"\n  Mission started: {msg.start_address}\n"
            f"           → {msg.end_address}\n"
            f"  Total planned: {msg.total_distance_m / 1000.0:.2f} km  "
            f"({msg.estimated_duration_s / 60.0:.1f} min estimate)"
        )

    def _on_state(self, msg: VehicleState) -> None:
        self._last_msg_time = time.time()
        if msg.speed < 0.1 and self._last_speed >= 0.1:
            self._speed_zero_since = time.time()
        self._last_speed = msg.speed

        if self._start_x is None:
            self._start_x = msg.x
            self._start_y = msg.y

        if self._last_x is not None:
            dx = msg.x - self._last_x
            dy = msg.y - self._last_y
            self._distance_traveled += math.sqrt(dx * dx + dy * dy)

        self._last_x = msg.x
        self._last_y = msg.y

    def _print_update(self) -> None:
        if not self._mission_active or self._arrived:
            return
        if self._last_x is None:
            return

        elapsed_since_msg = time.time() - self._last_msg_time
        if elapsed_since_msg > 5.0:
            return  # vehicle likely arrived — _check_arrival will handle it

        self.get_logger().info(
            f"\n  Position : x={self._last_x:.1f}  y={self._last_y:.1f}\n"
            f"  Speed    : {self._last_speed:.1f} m/s  "
            f"({self._last_speed * 3.6:.1f} km/h)\n"
            f"  Distance : {self._distance_traveled / 1000.0:.3f} km traveled"
        )

    def _check_arrival(self) -> None:
        if not self._mission_active or self._arrived or self._last_msg_time == 0.0:
            return

        # Vehicle removed from SUMO — clean arrival.
        silent_for = time.time() - self._last_msg_time
        if silent_for > 5.0:
            self._announce_arrival("Destination reached")
            return

        # Vehicle stopped for 60+ seconds — stuck or arrived at edge.
        if self._speed_zero_since > 0.0:
            stopped_for = time.time() - self._speed_zero_since
            if stopped_for > 60.0:
                self._announce_arrival("Vehicle stopped at destination edge")

    def _announce_arrival(self, reason: str) -> None:
        self._arrived = True
        self._mission_active = False
        self.get_logger().info(
            f"\n  ✓ {reason}!\n"
            f"    {self._start_address}\n"
            f"    → {self._end_address}\n"
            f"    Distance traveled: {self._distance_traveled / 1000.0:.3f} km"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
