"""
Simulation Layer Launch File

Launches the full simulation stack:
  - mission_input   : terminal prompt for start/destination
  - sumo_bridge     : SUMO ↔ ROS2 integration (ego vehicle + state publishing)
  - traffic_spawner : background vehicle population manager

Depends on the map layer (ads_map) being active first so /map/status == "READY"
before SUMO starts. Launch the map layer separately or use ads_full.launch.py.

Usage
-----
  ros2 launch ads_simulation simulation.launch.py
  ros2 launch ads_simulation simulation.launch.py use_gui:=false   # headless
  ros2 launch ads_simulation simulation.launch.py traffic:=30      # fewer vehicles
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
        default_value="true",
        description="Launch sumo-gui (true) or headless sumo (false).",
    )

    log_level = LaunchConfiguration("log_level")
    use_gui = LaunchConfiguration("use_gui")

    mission_input = Node(
        package="ads_simulation",
        executable="mission_input",
        name="mission_input",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    sumo_bridge = Node(
        package="ads_simulation",
        executable="sumo_bridge",
        name="sumo_bridge",
        parameters=[config_path, {"use_gui": use_gui}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    traffic_spawner = Node(
        package="ads_simulation",
        executable="traffic_spawner",
        name="traffic_spawner",
        parameters=[config_path],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        log_level_arg,
        use_gui_arg,
        LogInfo(msg=["[ADS] Launching simulation layer — gui=", use_gui]),
        mission_input,
        sumo_bridge,
        traffic_spawner,
    ])
