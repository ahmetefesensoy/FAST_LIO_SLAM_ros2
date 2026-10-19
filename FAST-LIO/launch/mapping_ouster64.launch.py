from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("fast_lio")
    params_file = PathJoinSubstitution(
        [package_share, "config", "ouster64_ros2.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz_cfg", "loam_livox.rviz"]
    )
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="laser_mapping",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fast_lio_rviz",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
