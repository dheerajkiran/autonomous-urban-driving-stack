from setuptools import find_packages, setup
from glob import glob

package_name = "ads_simulation"

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
    description="SUMO simulation bridge for the Autonomous Driving Stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_input = ads_simulation.mission_input:main",
            "sumo_bridge = ads_simulation.sumo_bridge:main",
            "traffic_spawner = ads_simulation.traffic_spawner:main",
        ],
    },
)
