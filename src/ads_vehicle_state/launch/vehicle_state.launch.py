"""
Vehicle State Launch File

Brings up the vehicle state layer of the ADS pipeline:
  - vehicle_state_publisher : kinematic simulator → /vehicle/state
  - vehicle_state_monitor   : passive diagnostic observer

Usage
-----
  ros2 launch ads_vehicle_state vehicle_state.launch.py
  ros2 launch ads_vehicle_state vehicle_state.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ads_vehicle_state")
    config_path = os.path.join(pkg_share, "config", "vehicle_state_params.yaml")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="ROS2 logging level applied to all nodes in this launch file.",
    )
    log_level = LaunchConfiguration("log_level")

    publisher_node = Node(
        package="ads_vehicle_state",
        executable="vehicle_state_publisher",
        name="vehicle_state_publisher",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    monitor_node = Node(
        package="ads_vehicle_state",
        executable="vehicle_state_monitor",
        name="vehicle_state_monitor",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        log_level_arg,
        LogInfo(msg=["[ADS] Launching vehicle state layer — log_level=", log_level]),
        publisher_node,
        monitor_node,
    ])
