from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"]
            ),
        ),
    )

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pointcloud_to_laserscan"), "launch", "sample_pointcloud_to_laserscan_launch.py"]
            ),
        ),
    )


    return LaunchDescription(
        [
            livox_launch,
            pointcloud_to_laserscan_launch,
        ]
    )
