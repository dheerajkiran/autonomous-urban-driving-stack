"""
Mission Input Node

Prompts the user for a start address and destination via the terminal,
then publishes the mission goal as a JSON string on /navigation/mission_goal.

Runs the input prompt in a background thread so it doesn't block ROS2 spin.
Supports multiple missions in sequence — after each route completes, the
user is prompted again.

Publishes
---------
/navigation/mission_goal  (std_msgs/String)  — JSON: {"start": "...", "end": "..."}

Subscribes
----------
/map/status  (std_msgs/String)  — waits for "READY" before prompting
"""

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_BANNER = """
╔══════════════════════════════════════════════════════════╗
║       Autonomous Driving Stack — Mission Input           ║
║       Tempe, AZ Road Network                             ║
╚══════════════════════════════════════════════════════════╝

Example addresses:
  • Arizona State University, Tempe, AZ
  • Tempe Marketplace, Tempe, AZ
  • Mill Avenue, Tempe, AZ
  • Tempe Town Lake, Tempe, AZ
  • Sky Harbor Airport, Phoenix, AZ

Type a start and destination to begin navigation.
"""


class MissionInput(Node):
    """
    Terminal-based mission goal publisher.

    Waits for the map to report READY, then launches an input thread that
    prompts the user for start/destination pairs. Each accepted input is
    published as a JSON string for route_planner to consume.
    """

    def __init__(self) -> None:
        super().__init__("mission_input")

        self._goal_pub = self.create_publisher(String, "/navigation/mission_goal", 10)
        self._status_sub = self.create_subscription(
            String, "/map/status", self._on_map_status, 10
        )

        self._map_ready = False
        self._input_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()

        self.get_logger().info(
            "MissionInput initialized — waiting for map to be ready..."
        )

    # ------------------------------------------------------------------
    # Map readiness gate
    # ------------------------------------------------------------------

    def _on_map_status(self, msg: String) -> None:
        if msg.data == "READY" and not self._map_ready:
            self._map_ready = True
            self.get_logger().info("Map is ready. Starting mission prompt.")
            self._start_input_thread()
        elif msg.data == "ERROR":
            self.get_logger().error(
                "Map loader reported an error — cannot accept missions."
            )

    def _start_input_thread(self) -> None:
        self._input_thread = threading.Thread(
            target=self._prompt_loop,
            daemon=True,
            name="mission-input-thread",
        )
        self._input_thread.start()

    # ------------------------------------------------------------------
    # Input loop (runs in background thread)
    # ------------------------------------------------------------------

    def _prompt_loop(self) -> None:
        print(_BANNER)

        while not self._shutdown_event.is_set():
            try:
                start = input("  Start address  : ").strip()
                if not start or self._shutdown_event.is_set():
                    continue

                destination = input("  Destination    : ").strip()
                if not destination or self._shutdown_event.is_set():
                    continue

                self._publish_goal(start, destination)

            except EOFError:
                # Non-interactive terminal (piped input exhausted).
                break
            except KeyboardInterrupt:
                break

    def _publish_goal(self, start: str, destination: str) -> None:
        payload = json.dumps({"start": start, "end": destination})
        msg = String()
        msg.data = payload
        self._goal_pub.publish(msg)
        self.get_logger().info(
            f"Mission published: '{start}' → '{destination}'"
        )
        print(f"\n  Route request sent. Watch the simulation...\n")

    def destroy_node(self) -> None:
        self._shutdown_event.set()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MissionInput.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
