"""
Vehicle Commander

Publishes a scripted VehicleCommand mission sequence on /vehicle/command.
This node acts as a stand-in for the trajectory planner during Phase 2,
exercising all command features: straight-line cruise, left turn, speed
reduction, and emergency stop.

In Phase 4 this node will be replaced by the behavior planner + controller.
It is retained as a test utility and integration smoke test.

Publishes
---------
/vehicle/command  (ads_interfaces/msg/VehicleCommand)  @ <publish_rate> Hz

Mission sequence (configurable via parameters)
-----------------------------------------------
  Step 0 — Cruise straight at 8 m/s for 5 s
  Step 1 — Turn left (0.25 rad) at 8 m/s for 5 s
  Step 2 — Straighten out at 8 m/s for 4 s
  Step 3 — Decelerate to 3 m/s over 4 s
  Step 4 — Emergency stop
  Step 5 — Park (mission complete)
"""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ads_interfaces.msg import VehicleCommand


@dataclass(frozen=True)
class MissionStep:
    label: str
    duration: float        # seconds (0.0 = terminal step, stays indefinitely)
    target_speed: float    # m/s
    steering_angle: float  # rad
    gear: str
    emergency_stop: bool


class VehicleCommander(Node):
    """
    Scripted mission publisher for Phase 2 integration testing.

    Executes a fixed mission sequence, advancing through steps by elapsed time.
    Each step is logged at INFO level so the terminal shows the mission narrative.
    The commander keeps publishing the last step's command indefinitely once
    the mission ends (park at rest).
    """

    _MISSION: tuple[MissionStep, ...] = (
        MissionStep("Cruise straight",   duration=5.0, target_speed=8.0,  steering_angle=0.0,  gear="DRIVE", emergency_stop=False),
        MissionStep("Turn left",         duration=5.0, target_speed=8.0,  steering_angle=0.25, gear="DRIVE", emergency_stop=False),
        MissionStep("Straighten out",    duration=4.0, target_speed=8.0,  steering_angle=0.0,  gear="DRIVE", emergency_stop=False),
        MissionStep("Decelerate",        duration=4.0, target_speed=3.0,  steering_angle=0.0,  gear="DRIVE", emergency_stop=False),
        MissionStep("Emergency stop",    duration=3.0, target_speed=0.0,  steering_angle=0.0,  gear="DRIVE", emergency_stop=True),
        MissionStep("Park",              duration=0.0, target_speed=0.0,  steering_angle=0.0,  gear="PARK",  emergency_stop=False),
    )

    def __init__(self) -> None:
        super().__init__("vehicle_commander")

        self.declare_parameter("command_topic", "/vehicle/command")
        self.declare_parameter("publish_rate", 20.0)

        self._command_topic: str = self.get_parameter("command_topic").value
        self._publish_rate: float = self.get_parameter("publish_rate").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(VehicleCommand, self._command_topic, qos)
        self._timer = self.create_timer(1.0 / self._publish_rate, self._tick)

        self._step_index: int = 0
        self._step_start_time: Optional[float] = None
        self._mission_complete: bool = False

        self.get_logger().info(
            f"VehicleCommander ready — {len(self._MISSION)} mission steps, "
            f"publishing to '{self._command_topic}' at {self._publish_rate:.1f} Hz"
        )

    # ------------------------------------------------------------------
    # Mission execution
    # ------------------------------------------------------------------

    def _current_step(self) -> MissionStep:
        return self._MISSION[self._step_index]

    def _advance_step(self) -> None:
        self._step_index += 1
        self._step_start_time = self.get_clock().now().nanoseconds * 1e-9
        step = self._current_step()
        self.get_logger().info(
            f"[MISSION] Step {self._step_index}/{len(self._MISSION) - 1}: "
            f"{step.label} — "
            f"speed={step.target_speed:.1f} m/s, "
            f"steering={math.degrees(step.steering_angle):.1f}°, "
            f"e-stop={step.emergency_stop}"
        )

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        # Initialise on first tick.
        if self._step_start_time is None:
            self._step_start_time = now
            step = self._current_step()
            self.get_logger().info(
                f"[MISSION] Step 0/{len(self._MISSION) - 1}: "
                f"{step.label} — "
                f"speed={step.target_speed:.1f} m/s, "
                f"steering={math.degrees(step.steering_angle):.1f}°, "
                f"e-stop={step.emergency_stop}"
            )

        step = self._current_step()
        elapsed = now - self._step_start_time

        # Advance to next step if duration has elapsed (duration=0 means hold forever).
        if (
            not self._mission_complete
            and step.duration > 0.0
            and elapsed >= step.duration
            and self._step_index < len(self._MISSION) - 1
        ):
            self._advance_step()
            step = self._current_step()

        if step.duration == 0.0 and not self._mission_complete:
            self._mission_complete = True
            self.get_logger().info("[MISSION] Complete — vehicle parked.")

        self._publish_command(step)

    def _publish_command(self, step: MissionStep) -> None:
        msg = VehicleCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.target_speed = step.target_speed
        msg.steering_angle = step.steering_angle
        msg.gear = step.gear
        msg.emergency_stop = step.emergency_stop
        self._publisher.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VehicleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested — stopping VehicleCommander.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
