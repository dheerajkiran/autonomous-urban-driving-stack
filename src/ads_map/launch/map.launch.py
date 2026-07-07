"""
Map Layer Launch File

Launches the full map stack:
  - map_loader      : downloads / loads Tempe OSM and generates SUMO network
  - route_planner   : A* routing on the OSM graph
  - waypoint_publisher : tracks progress along active route
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ads_map")
    config_path = os.path.join(pkg_share, "config", "map_params.yaml")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for map layer nodes.",
    )
    log_level = LaunchConfiguration("log_level")

    map_loader = Node(
        package="ads_map",
        executable="map_loader",
        name="map_loader",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    route_planner = Node(
        package="ads_map",
        executable="route_planner",
        name="route_planner",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    waypoint_publisher = Node(
        package="ads_map",
        executable="waypoint_publisher",
        name="waypoint_publisher",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        log_level_arg,
        LogInfo(msg=["[ADS] Launching map layer — log_level=", log_level]),
        map_loader,
        route_planner,
        waypoint_publisher,
    ])
