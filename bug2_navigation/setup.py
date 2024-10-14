from glob import glob
from setuptools import setup
import os

package_name = "bug2_navigation"

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
            "wallfollowercontroller = bug2_navigation.wallfollowercontroller:main",
            "wallfollowercontroller_v2 = bug2_navigation.wallfollowercontroller_v2:main",
            "gotopointcontroller = bug2_navigation.gotopointcontroller:main",
            "wall_follower = bug2_navigation.wall_follower:main",
            "go_to_point = bug2_navigation.go_to_point:main",
            "bug2_navigation = bug2_navigation.bug2_navigation:main",
        ],
    },
)
