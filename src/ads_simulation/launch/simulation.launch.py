"""
Simulation Layer Launch File

Launches the SUMO bridge (headless SUMO + ego spawning on click-to-drive
confirm) and the OpenStreetMap pygame viewer for route planning, S/E pin
placement, and visualization.

Usage
-----
  ros2 launch ads_simulation simulation.launch.py
  ros2 launch ads_simulation simulation.launch.py use_gui:=true   # SUMO GUI window
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ads_simulation")
    config_path = os.path.join(pkg_share, "config", "simulation_params.yaml")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for simulation nodes.",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="Launch sumo-gui (true) or headless sumo (false).",
    )

    log_level = LaunchConfiguration("log_level")
    use_gui   = LaunchConfiguration("use_gui")

    sumo_bridge = Node(
        package="ads_simulation",
        executable="sumo_bridge",
        name="sumo_bridge",
        parameters=[config_path, {"use_gui": use_gui}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    pygame_viewer = Node(
        package="ads_simulation",
        executable="pygame_viewer",
        name="pygame_viewer",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        log_level_arg,
        use_gui_arg,
        LogInfo(msg=["[ADS] Launching simulation layer — gui=", use_gui]),
        sumo_bridge,
        pygame_viewer,
    ])
