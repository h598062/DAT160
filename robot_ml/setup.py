from setuptools import setup
import os
from glob import glob

package_name = "robot_ml"

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
            os.path.join(
                "share", package_name, "models", "painting", "materials", "scripts"
            ),
            glob("models/painting/materials/scripts/*"),
        ),
        (
            os.path.join(
                "share", package_name, "models", "painting", "materials", "textures"
            ),
            glob("models/painting/materials/textures/*"),
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
            "camera_viz = robot_ml.camera_viz:main",
            "project_ml_idea = robot_ml.project_ml_idea:main",
        ],
    },
)
