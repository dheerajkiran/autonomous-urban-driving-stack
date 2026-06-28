"""
Vehicle State Publisher

Simulates vehicle kinematic state using a simplified bicycle model and publishes
it on /vehicle/state. In a production system this node would be replaced by (or
wrap) a hardware abstraction layer that reads from the vehicle's CAN bus or a
high-fidelity simulator such as CARLA.

Publishes
---------
/vehicle/state  (ads_interfaces/msg/VehicleState)  @ <publish_rate> Hz
"""

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ads_interfaces.msg import VehicleState


class VehicleStatePublisher(Node):
    """
    Publishes simulated vehicle kinematic state at a configurable rate.

    The vehicle starts from rest and accelerates to a target cruising speed using
    a constant-acceleration model. Gaussian noise is added to the speed measurement
    to approximate real-world sensor noise. Position is integrated from velocity
    using forward Euler, with heading fixed to 0 rad (east) for Phase 1.

    This node is intentionally decoupled from the planner and controller — it
    models the vehicle plant, not the control policy.
    """

    # Gear constants match the VehicleState.msg field contract.
    GEAR_PARK = "PARK"
    GEAR_DRIVE = "DRIVE"

    # Throttle applied during constant-speed cruise to model parasitic losses.
    _CRUISE_THROTTLE: float = 0.08

    def __init__(self) -> None:
        super().__init__("vehicle_state_publisher")

        self._declare_parameters()
        self._load_parameters()
        self._init_state()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(VehicleState, self._topic_name, qos)
        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info(
            f"VehicleStatePublisher ready — topic='{self._topic_name}', "
            f"rate={self._publish_rate:.1f} Hz, "
            f"target_speed={self._target_speed:.1f} m/s"
        )

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("target_speed", 10.0)
        self.declare_parameter("acceleration_rate", 1.5)
        self.declare_parameter("topic_name", "/vehicle/state")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("noise_stddev", 0.05)

    def _load_parameters(self) -> None:
        self._publish_rate: float = self.get_parameter("publish_rate").value
        self._target_speed: float = self.get_parameter("target_speed").value
        self._accel_rate: float = self.get_parameter("acceleration_rate").value
        self._topic_name: str = self.get_parameter("topic_name").value
        self._frame_id: str = self.get_parameter("frame_id").value
        self._noise_stddev: float = self.get_parameter("noise_stddev").value
        self._dt: float = 1.0 / self._publish_rate

    def _init_state(self) -> None:
        self._x: float = 0.0
        self._y: float = 0.0
        self._heading: float = 0.0        # rad — fixed east for Phase 1
        self._speed: float = 0.0          # m/s
        self._acceleration: float = 0.0   # m/s²
        self._yaw_rate: float = 0.0       # rad/s
        self._steering_angle: float = 0.0 # rad
        self._throttle: float = 0.0
        self._brake: float = 0.0
        self._gear: str = self.GEAR_PARK
        self._seq: int = 0

    # ------------------------------------------------------------------
    # Dynamics
    # ------------------------------------------------------------------

    def _step_dynamics(self) -> None:
        """Advance the kinematic bicycle model one time step forward."""
        at_target = self._speed >= self._target_speed

        if at_target:
            self._acceleration = 0.0
            self._throttle = self._CRUISE_THROTTLE
            self._brake = 0.0
        else:
            self._acceleration = self._accel_rate
            # Scale throttle proportionally to acceleration demand.
            self._throttle = min(1.0, self._acceleration / 4.0)
            self._brake = 0.0
            self._gear = self.GEAR_DRIVE

        self._speed = min(
            self._speed + self._acceleration * self._dt,
            self._target_speed,
        )

        # Perturb speed to simulate sensor noise before integrating position.
        noisy_speed = self._speed + random.gauss(0.0, self._noise_stddev)

        self._x += noisy_speed * math.cos(self._heading) * self._dt
        self._y += noisy_speed * math.sin(self._heading) * self._dt

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._step_dynamics()
        self._seq += 1

        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.speed = self._speed
        msg.acceleration = self._acceleration
        msg.heading = self._heading
        msg.yaw_rate = self._yaw_rate
        msg.x = self._x
        msg.y = self._y
        msg.throttle = self._throttle
        msg.brake = self._brake
        msg.steering_angle = self._steering_angle
        msg.gear = self._gear
        msg.is_autonomous = True

        self._publisher.publish(msg)

        self.get_logger().debug(
            f"[seq={self._seq:05d}] "
            f"speed={self._speed:.2f} m/s | "
            f"accel={self._acceleration:.2f} m/s² | "
            f"pos=({self._x:.1f}, {self._y:.1f}) | "
            f"gear={self._gear}"
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VehicleStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested — stopping VehicleStatePublisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
