import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("frc_bringup")
    playbook_share = get_package_share_directory("frc_alliance_playbook")
    foundation = os.path.join(bringup_share, "launch", "foundation.launch.py")
    default_plan = os.path.join(playbook_share, "config", "example-plan.json")
    use_rviz = LaunchConfiguration("use_rviz")
    use_detector = LaunchConfiguration("use_detector")
    use_synthetic = LaunchConfiguration("use_synthetic_perception")
    use_fake_roborio = LaunchConfiguration("use_fake_roborio")
    plan_path = LaunchConfiguration("alliance_plan_path")
    plan_hash = LaunchConfiguration("alliance_plan_sha256")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="false"),
        DeclareLaunchArgument("use_synthetic_perception", default_value="true"),
        DeclareLaunchArgument("use_fake_roborio", default_value="true"),
        DeclareLaunchArgument("alliance_plan_path", default_value=default_plan),
        DeclareLaunchArgument("alliance_plan_sha256", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(foundation),
            launch_arguments={
                "use_rviz": use_rviz,
                "use_fake_roborio": use_fake_roborio,
                "use_fake_autonomy": "false",
                "arm_fake_autonomy": "false",
                "use_synthetic_perception": use_synthetic,
                "use_alliance_playbook": "true",
                "alliance_plan_path": plan_path,
                "alliance_plan_sha256": plan_hash,
                "use_dynamic_autonomy": "true",
                "use_detector": use_detector,
            }.items(),
        ),
    ])
