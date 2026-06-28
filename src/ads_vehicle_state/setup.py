from setuptools import find_packages, setup
from glob import glob

package_name = "ads_vehicle_state"

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
    description="Vehicle state simulation and monitoring for the Autonomous Driving Stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_state_publisher = ads_vehicle_state.vehicle_state_publisher:main",
            "vehicle_state_monitor = ads_vehicle_state.vehicle_state_monitor:main",
        ],
    },
)
