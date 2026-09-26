import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("frc_bringup")
    description_share = get_package_share_directory("frc_robot_description")
    xacro_file = os.path.join(description_share, "urdf", "frc_2018_robot.urdf.xacro")
    rviz_file = os.path.join(description_share, "rviz", "autonomy.rviz")
    config_file = os.path.join(bringup_share, "config", "autonomy.yaml")

    use_rviz = LaunchConfiguration("use_rviz")
    use_fake_roborio = LaunchConfiguration("use_fake_roborio")
    use_fake_autonomy = LaunchConfiguration("use_fake_autonomy")
    arm_fake_autonomy = LaunchConfiguration("arm_fake_autonomy")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_fake_roborio", default_value="false"),
        DeclareLaunchArgument("use_fake_autonomy", default_value="false"),
        DeclareLaunchArgument("arm_fake_autonomy", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="frc_bringup",
            executable="fake_roborio",
            condition=IfCondition(use_fake_roborio),
            output="screen",
        ),
        Node(
            package="frc_nt_bridge",
            executable="bridge_node",
            parameters=[config_file],
            output="screen",
        ),
        Node(
            package="frc_bringup",
            executable="status_visualizer",
            output="screen",
        ),
        Node(
            package="frc_bringup",
            executable="fake_autonomy",
            condition=IfCondition(use_fake_autonomy),
            parameters=[
                config_file,
                {"armed": ParameterValue(arm_fake_autonomy, value_type=bool)},
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_file],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
