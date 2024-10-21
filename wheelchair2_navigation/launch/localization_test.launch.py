from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_description"),
                 "launch", "wheelchair_urdf.launch.py"]
            )
        )
    )

    return ld
