import os
from glob import glob

from setuptools import find_packages, setup


package_name = "frc_world_model"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FRC Autonomy Team",
    maintainer_email="robotics@example.com",
    description="Deterministic semantic tracking and FRC world snapshots",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "synthetic_perception = frc_world_model.synthetic_perception_node:main",
            "world_model = frc_world_model.world_model_node:main",
            "world_visualizer = frc_world_model.world_visualizer_node:main",
        ]
    },
)
