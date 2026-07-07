"""
Traffic Spawner Node

Populates the Tempe road network with background vehicles to simulate
realistic city traffic. Vehicles are spawned on random valid edges,
assigned random routes, and removed when they complete their journey.

Maintains a target vehicle count by monitoring SUMO's vehicle list and
spawning replacements as vehicles depart the network.

Subscribes
----------
/simulation/status  (std_msgs/String)  — starts spawning when "RUNNING"

Publishes
---------
/simulation/traffic_count  (std_msgs/String)  — live vehicle count summary
"""

import random
import string
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_VEHICLE_TYPES = [
    ("passenger", 0.65),    # 65% regular cars
    ("passenger/van", 0.15),  # 15% vans
    ("truck", 0.10),          # 10% trucks
    ("bus", 0.05),            # 5% buses
    ("motorcycle", 0.05),     # 5% motorcycles
]

_VEHICLE_COLORS = [
    (200, 200, 200, 255),  # silver
    (180, 0, 0, 255),      # red
    (0, 0, 180, 255),      # blue
    (0, 140, 0, 255),      # green
    (230, 180, 0, 255),    # yellow
    (100, 100, 100, 255),  # dark grey
    (255, 140, 0, 255),    # orange
]


def _weighted_choice(choices: list[tuple]) -> str:
    """Pick from (value, weight) pairs."""
    values, weights = zip(*choices)
    r = random.random()
    cumulative = 0.0
    for v, w in zip(values, weights):
        cumulative += w
        if r <= cumulative:
            return v
    return values[-1]


def _random_id(prefix: str = "bg_") -> str:
    suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))
    return f"{prefix}{suffix}"


class TrafficSpawner(Node):
    """
    Background traffic manager for the SUMO simulation.

    Maintains a target number of vehicles on the Tempe road network.
    Vehicles are spawned on randomly chosen source edges with randomly
    chosen destination edges, creating emergent traffic patterns.
    """

    def __init__(self) -> None:
        super().__init__("traffic_spawner")

        self.declare_parameter("target_vehicle_count", 60)
        self.declare_parameter("spawn_rate", 2.0)          # vehicles per second
        self.declare_parameter("spawn_check_interval", 1.0) # seconds
        self.declare_parameter("min_route_length_m", 500.0)

        self._target_count = self.get_parameter("target_vehicle_count").value
        self._spawn_rate = self.get_parameter("spawn_rate").value
        self._check_interval = self.get_parameter("spawn_check_interval").value

        self._simulation_running = False
        self._traci = None
        self._valid_edges: list[str] = []
        self._spawn_counter = 0

        self._status_sub = self.create_subscription(
            String, "/simulation/status", self._on_simulation_status, 10
        )
        self._count_pub = self.create_publisher(String, "/simulation/traffic_count", 10)

        self._timer = self.create_timer(self._check_interval, self._tick)

        self.get_logger().info(
            f"TrafficSpawner initialized — target={self._target_count} vehicles"
        )

    # ------------------------------------------------------------------
    # Simulation readiness
    # ------------------------------------------------------------------

    def _on_simulation_status(self, msg: String) -> None:
        if msg.data == "RUNNING" and not self._simulation_running:
            self._connect_traci()

    def _connect_traci(self) -> None:
        try:
            import traci
            self._traci = traci
            self._simulation_running = True
            self._cache_valid_edges()
            self.get_logger().info(
                f"TrafficSpawner connected to SUMO — "
                f"{len(self._valid_edges)} usable edges found."
            )
        except ImportError:
            self.get_logger().error("traci not available.")
        except Exception as exc:
            self.get_logger().error(f"TraCI connection failed: {exc}")

    def _cache_valid_edges(self) -> None:
        """Build a list of edges long enough to use as spawning sources."""
        try:
            all_edges = self._traci.edge.getIDList()
            min_len = self.get_parameter("min_route_length_m").value

            self._valid_edges = [
                e for e in all_edges
                if not e.startswith(":")              # exclude internal junctions
                and self._traci.edge.getLastStepLength(e) > 0  # has driveable length
            ]

            self.get_logger().info(
                f"Edge cache built — {len(self._valid_edges)} valid edges "
                f"(from {len(all_edges)} total)"
            )
        except Exception as exc:
            self.get_logger().error(f"Edge caching failed: {exc}")

    # ------------------------------------------------------------------
    # Spawn tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if not self._simulation_running or not self._valid_edges:
            return

        try:
            current_count = len(self._traci.vehicle.getIDList())
            deficit = self._target_count - current_count
            to_spawn = min(deficit, max(1, int(self._spawn_rate * self._check_interval)))

            for _ in range(max(0, to_spawn)):
                self._spawn_one()

            msg = String()
            msg.data = (
                f"Active: {current_count} | "
                f"Target: {self._target_count} | "
                f"Total spawned: {self._spawn_counter}"
            )
            self._count_pub.publish(msg)

        except Exception as exc:
            self.get_logger().warn(
                f"Spawn tick error: {exc}", throttle_duration_sec=5.0
            )

    def _spawn_one(self) -> None:
        if len(self._valid_edges) < 2:
            return

        src_edge = random.choice(self._valid_edges)
        dst_edge = random.choice(self._valid_edges)
        if src_edge == dst_edge:
            return

        vid = _random_id()
        route_id = f"route_{vid}"
        vtype = _weighted_choice(_VEHICLE_TYPES)
        color = random.choice(_VEHICLE_COLORS)

        try:
            self._traci.route.add(route_id, [src_edge, dst_edge])
            self._traci.vehicle.add(
                vid,
                routeID=route_id,
                typeID=vtype,
                departSpeed="speedLimit",
                departLane="random",
            )
            self._traci.vehicle.setColor(vid, color)
            self._spawn_counter += 1
        except Exception:
            # Route may be unreachable — silently skip.
            try:
                self._traci.route.remove(route_id)
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrafficSpawner.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
