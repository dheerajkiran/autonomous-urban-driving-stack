from setuptools import find_packages, setup
from glob import glob

package_name = "ads_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dheeraj Kiran",
    maintainer_email="denna@asu.edu",
    description="Map loading and route planning for the Autonomous Driving Stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_loader = ads_map.map_loader:main",
            "route_planner = ads_map.route_planner:main",
            "waypoint_publisher = ads_map.waypoint_publisher:main",
        ],
    },
)
