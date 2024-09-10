from setuptools import setup
import os
from glob import glob

package_name = "robot_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.world")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.xacro")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocotics",
    maintainer_email="598062@stud.hvl.no",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tb3_blob_tracker = robot_vision.tb3_blob_tracker:main",
            "command_sphere = robot_vision.command_sphere:main",
        ],
    },
)
