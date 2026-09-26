import os
from glob import glob

from setuptools import find_packages, setup


package_name = "frc_alliance_playbook"

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
    description="Validated teammate routes and time-dependent reservations",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "playbook = frc_alliance_playbook.cli:main",
            "alliance_playbook = frc_alliance_playbook.playbook_node:main",
            "alliance_visualizer = frc_alliance_playbook.visualizer_node:main",
        ]
    },
)
