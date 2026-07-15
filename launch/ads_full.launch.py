"""
ADS Full Stack Launch File

Single entry point that brings up the entire Autonomous Driving Stack:

  Layer 1 — Map        (ads_map)        : OSM loading + A* route planning
  Layer 2 — Simulation  (ads_simulation) : SUMO bridge + Pygame click-to-drive viewer

Startup sequence (enforced by /map/status topic):
  1. map_loader downloads / loads Tempe OSM and generates SUMO network.
  2. Once /map/status == "READY", sumo_bridge starts SUMO automatically.
  3. pygame_viewer lets the user place S/E pins and press SPACE to confirm.
  4. route_planner resolves the lat/lon goal and publishes the display route.
  5. sumo_bridge resolves start/end SUMO edges and spawns the ego vehicle.

Usage
-----
  ros2 launch launch/ads_full.launch.py
  ros2 launch launch/ads_full.launch.py use_gui:=true    # sumo-gui (not available on ARM64)
  ros2 launch launch/ads_full.launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logging level applied to all nodes."
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="false",
        description="Open SUMO GUI window (true) or run headless (false). "
                     "sumo-gui is not available on ARM64 — leave false on the VM."
    )

    log_level = LaunchConfiguration("log_level")
    use_gui = LaunchConfiguration("use_gui")

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ads_map"), "launch", "map.launch.py"])
        ]),
        launch_arguments={"log_level": log_level}.items(),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ads_simulation"), "launch", "simulation.launch.py"
            ])
        ]),
        launch_arguments={"log_level": log_level, "use_gui": use_gui}.items(),
    )

    return LaunchDescription([
        log_level_arg,
        use_gui_arg,
        LogInfo(msg=["[ADS] Starting full Autonomous Driving Stack"]),
        LogInfo(msg=["[ADS] Map layer initializing — downloading Tempe, AZ if needed..."]),
        map_launch,
        sim_launch,
    ])
