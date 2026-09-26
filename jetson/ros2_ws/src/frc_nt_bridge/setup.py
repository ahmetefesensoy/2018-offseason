from setuptools import find_packages, setup


package_name = "frc_nt_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FRC Autonomy Team",
    maintainer_email="robotics@example.com",
    description="Fail-closed ROS 2 to FRC NT4 autonomy bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": ["bridge_node = frc_nt_bridge.bridge_node:main"]},
)
