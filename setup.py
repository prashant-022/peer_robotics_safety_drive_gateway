from setuptools import find_packages, setup
import os
from glob import glob

package_name = "drive_safety_gateway"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="prashant",
    maintainer_email="pkumarofficial02@gmail.com",
    description="Drive Safety Gateway ROS2 node for arbitration between planner commands and safety constraints.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "drive_safety_gateway = drive_safety_gateway.gateway_node:main",
        ],
    },
)
