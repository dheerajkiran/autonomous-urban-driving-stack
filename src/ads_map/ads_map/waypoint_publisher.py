"""
Waypoint Publisher Node

Tracks the ego vehicle's progress along the active route and publishes the
current target waypoint. Advances to the next waypoint when the vehicle
comes within <waypoint_radius_m> of the current target.

Subscribes
----------
/navigation/route    (ads_interfaces/msg/Route)         — active route from route_planner
/vehicle/state       (ads_interfaces/msg/VehicleState)  — ego vehicle position (x, y)

Publishes
---------
/navigation/current_waypoint  (ads_interfaces/msg/Waypoint)  — next waypoint to reach
/navigation/route             (ads_interfaces/msg/Route)      — re-publishes route with
                                                                updated current_waypoint_index
/navigation/progress          (std_msgs/String)               — human-readable progress string
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, VehicleState, Waypoint


class WaypointPublisher(Node):
    """
    Progress tracker and current waypoint broadcaster.

    Monitors how close the ego vehicle is to each successive waypoint and
    advances the index when within the arrival radius. Announces each
    waypoint transition at INFO level so the terminal shows route progress.
    """

    def __init__(self) -> None:
        super().__init__("waypoint_publisher")

        self.declare_parameter("waypoint_radius_m", 20.0)
        self.declare_parameter("publish_rate", 10.0)

        self._waypoint_radius = self.get_parameter("waypoint_radius_m").value
        self._publish_rate = self.get_parameter("publish_rate").value

        self._active_route: Optional[Route] = None
        self._ego_x: float = 0.0
        self._ego_y: float = 0.0

        self._route_sub = self.create_subscription(
            Route, "/navigation/route", self._on_route, 10
        )
        self._state_sub = self.create_subscription(
            VehicleState, "/vehicle/state", self._on_vehicle_state, 10
        )

        self._wp_pub = self.create_publisher(Waypoint, "/navigation/current_waypoint", 10)
        self._route_pub = self.create_publisher(Route, "/navigation/route_progress", 10)
        self._progress_pub = self.create_publisher(String, "/navigation/progress", 10)

        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info("WaypointPublisher initialized — waiting for route.")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_route(self, msg: Route) -> None:
        self._active_route = msg
        total_wp = len(msg.waypoints)
        self.get_logger().info(
            f"New route received: '{msg.start_address}' → '{msg.end_address}' "
            f"| {total_wp} waypoints | {msg.total_distance_m / 1000.0:.2f} km"
        )

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        self._ego_x = msg.x
        self._ego_y = msg.y

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._active_route is None or not self._active_route.waypoints:
            return

        route = self._active_route
        idx = route.current_waypoint_index

        if idx >= len(route.waypoints):
            self.get_logger().info(
                "Destination reached.",
                throttle_duration_sec=5.0,
            )
            return

        current_wp = route.waypoints[idx]
        dist = self._distance_to_waypoint(current_wp)

        if dist < self._waypoint_radius and idx < len(route.waypoints) - 1:
            route.current_waypoint_index += 1
            idx = route.current_waypoint_index
            current_wp = route.waypoints[idx]
            self.get_logger().info(
                f"Waypoint {idx}/{len(route.waypoints) - 1} reached — "
                f"next: {current_wp.road_name} "
                f"({current_wp.latitude:.5f}, {current_wp.longitude:.5f})"
            )

        self._wp_pub.publish(current_wp)

        pct = (idx / max(len(route.waypoints) - 1, 1)) * 100.0
        progress_str = (
            f"Waypoint {idx + 1}/{len(route.waypoints)} | "
            f"{pct:.0f}% complete | "
            f"Next: {current_wp.road_name} | "
            f"Dist to wp: {dist:.0f} m"
        )
        msg = String()
        msg.data = progress_str
        self._progress_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _distance_to_waypoint(self, wp: Waypoint) -> float:
        """Distance from ego vehicle (SUMO x/y) to waypoint (SUMO x/y)."""
        if wp.x == 0.0 and wp.y == 0.0:
            # Waypoint SUMO coords not yet populated — use a large placeholder.
            return float("inf")
        dx = self._ego_x - wp.x
        dy = self._ego_y - wp.y
        return math.sqrt(dx * dx + dy * dy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WaypointPublisher.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
