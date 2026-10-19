from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("aloam_velodyne"), "config", "pgo_ros2.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="aloam_velodyne",
                executable="alaserPGO",
                name="laser_pgo",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
