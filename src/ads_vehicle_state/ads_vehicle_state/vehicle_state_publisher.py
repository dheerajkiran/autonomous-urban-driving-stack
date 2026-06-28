"""
Vehicle State Publisher

Models the vehicle plant using a kinematic bicycle model. Subscribes to
/vehicle/command and publishes the resulting kinematic state on /vehicle/state.

In production this node wraps the hardware abstraction layer (CAN bus, CARLA,
or another simulator). For Phase 2 it responds to VehicleCommand messages
issued by vehicle_commander; in later phases the trajectory planner or
controller will be the command source.

Kinematic bicycle model equations
----------------------------------
  yaw_rate   = (speed / wheelbase) * tan(steering_angle)
  heading   += yaw_rate * dt
  x         += speed * cos(heading) * dt
  y         += speed * sin(heading) * dt

Publishes
---------
/vehicle/state  (ads_interfaces/msg/VehicleState)  @ <publish_rate> Hz

Subscribes
----------
/vehicle/command  (ads_interfaces/msg/VehicleCommand)
"""

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ads_interfaces.msg import VehicleCommand, VehicleState


class VehicleStatePublisher(Node):
    """
    Kinematic bicycle-model plant simulator.

    Accepts VehicleCommand messages and integrates the bicycle model equations
    forward at <publish_rate> Hz, broadcasting the resulting VehicleState.

    Safety behaviour
    ----------------
    - If no command has been received the vehicle stays in PARK at rest.
    - If the last command is older than <command_timeout> seconds a warning is
      emitted and the vehicle decelerates to a stop (fail-safe).
    - emergency_stop in a VehicleCommand overrides all other fields and applies
      maximum deceleration.
    """

    GEAR_PARK = "PARK"
    GEAR_DRIVE = "DRIVE"
    GEAR_NEUTRAL = "NEUTRAL"
    GEAR_REVERSE = "REVERSE"
    _VALID_GEARS = {GEAR_PARK, GEAR_DRIVE, GEAR_NEUTRAL, GEAR_REVERSE}

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

        self._state_pub = self.create_publisher(VehicleState, self._state_topic, qos)
        self._command_sub = self.create_subscription(
            VehicleCommand, self._command_topic, self._on_command, qos
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self.get_logger().info(
            f"VehicleStatePublisher ready — "
            f"state='{self._state_topic}', command='{self._command_topic}', "
            f"rate={self._publish_rate:.1f} Hz, "
            f"wheelbase={self._wheelbase:.2f} m"
        )

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("state_topic", "/vehicle/state")
        self.declare_parameter("command_topic", "/vehicle/command")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("noise_stddev", 0.03)
        self.declare_parameter("command_timeout", 0.5)
        # Vehicle physics
        self.declare_parameter("wheelbase", 2.7)
        self.declare_parameter("max_speed", 30.0)
        self.declare_parameter("max_accel", 3.0)
        self.declare_parameter("max_decel", 6.0)
        self.declare_parameter("emergency_decel", 9.0)
        self.declare_parameter("max_steering_angle", 0.5)

    def _load_parameters(self) -> None:
        self._publish_rate: float = self.get_parameter("publish_rate").value
        self._state_topic: str = self.get_parameter("state_topic").value
        self._command_topic: str = self.get_parameter("command_topic").value
        self._frame_id: str = self.get_parameter("frame_id").value
        self._noise_stddev: float = self.get_parameter("noise_stddev").value
        self._command_timeout: float = self.get_parameter("command_timeout").value
        self._wheelbase: float = self.get_parameter("wheelbase").value
        self._max_speed: float = self.get_parameter("max_speed").value
        self._max_accel: float = self.get_parameter("max_accel").value
        self._max_decel: float = self.get_parameter("max_decel").value
        self._emergency_decel: float = self.get_parameter("emergency_decel").value
        self._max_steering: float = self.get_parameter("max_steering_angle").value
        self._dt: float = 1.0 / self._publish_rate

    def _init_state(self) -> None:
        self._x: float = 0.0
        self._y: float = 0.0
        self._heading: float = 0.0
        self._speed: float = 0.0
        self._acceleration: float = 0.0
        self._yaw_rate: float = 0.0
        self._steering_angle: float = 0.0
        self._throttle: float = 0.0
        self._brake: float = 0.0
        self._gear: str = self.GEAR_PARK

        self._current_command: Optional[VehicleCommand] = None
        self._last_command_stamp: Optional[float] = None
        self._seq: int = 0

    # ------------------------------------------------------------------
    # Command callback
    # ------------------------------------------------------------------

    def _on_command(self, msg: VehicleCommand) -> None:
        self._current_command = msg
        self._last_command_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().debug(
            f"Command received — target_speed={msg.target_speed:.2f} m/s, "
            f"steering={math.degrees(msg.steering_angle):.1f}°, "
            f"gear={msg.gear}, e-stop={msg.emergency_stop}"
        )

    # ------------------------------------------------------------------
    # Dynamics
    # ------------------------------------------------------------------

    def _is_command_stale(self) -> bool:
        if self._last_command_stamp is None:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - self._last_command_stamp
        return age > self._command_timeout

    def _step_dynamics(self) -> None:
        """Advance the bicycle model one dt using the current command."""
        if self._current_command is None:
            # No command received yet — remain stationary in PARK.
            return

        if self._is_command_stale():
            self.get_logger().warn(
                "Command topic is stale — applying fail-safe deceleration.",
                throttle_duration_sec=2.0,
            )
            self._apply_deceleration(self._max_decel)
            return

        cmd = self._current_command

        if cmd.emergency_stop:
            self._apply_deceleration(self._emergency_decel)
            if self._speed < 0.1:
                self._gear = self.GEAR_PARK
            return

        # Clamp steering to physical limits.
        target_steer = max(-self._max_steering, min(self._max_steering, cmd.steering_angle))
        self._steering_angle = target_steer

        # Speed controller.
        speed_error = cmd.target_speed - self._speed
        if speed_error >= 0.0:
            accel = min(self._max_accel, speed_error / self._dt)
            self._throttle = min(1.0, accel / self._max_accel) if self._max_accel > 0 else 0.0
            self._brake = 0.0
        else:
            accel = max(-self._max_decel, speed_error / self._dt)
            self._throttle = 0.0
            self._brake = min(1.0, -accel / self._max_decel) if self._max_decel > 0 else 0.0

        self._acceleration = accel
        self._speed = max(0.0, min(self._speed + accel * self._dt, self._max_speed))

        # Gear management.
        if cmd.gear in self._VALID_GEARS:
            if cmd.gear == self.GEAR_PARK and self._speed > 0.1:
                self._gear = self.GEAR_DRIVE  # Can't engage PARK while moving.
            else:
                self._gear = cmd.gear

        # Kinematic bicycle model — update heading and position.
        if self._speed > 0.01:
            self._yaw_rate = (self._speed / self._wheelbase) * math.tan(self._steering_angle)
            self._heading += self._yaw_rate * self._dt
            self._heading = (self._heading + math.pi) % (2.0 * math.pi) - math.pi

        noisy_speed = self._speed + random.gauss(0.0, self._noise_stddev)
        self._x += noisy_speed * math.cos(self._heading) * self._dt
        self._y += noisy_speed * math.sin(self._heading) * self._dt

    def _apply_deceleration(self, decel_rate: float) -> None:
        accel = max(-decel_rate, -self._speed / self._dt)
        self._acceleration = accel
        self._throttle = 0.0
        self._brake = 1.0
        self._speed = max(0.0, self._speed + accel * self._dt)
        self._yaw_rate = 0.0
        if self._speed < 0.01:
            self._speed = 0.0

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

        self._state_pub.publish(msg)

        self.get_logger().debug(
            f"[seq={self._seq:05d}] "
            f"speed={self._speed:.2f} m/s | "
            f"heading={math.degrees(self._heading):.1f}° | "
            f"pos=({self._x:.1f}, {self._y:.1f}) | "
            f"steer={math.degrees(self._steering_angle):.1f}° | "
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
