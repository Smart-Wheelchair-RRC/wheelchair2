
from re import L
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch",
                 "sllidar_a3_launch.py"]
            ),
        ),
    )

    angle_range_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("wheelchair2_navigation"),
                    "config",
                    "angle_range_filter.yaml",
                ]
            )
        ],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )

    ld = LaunchDescription()
    ld.add_action(sllidar_launch)
    ld.add_action(angle_range_filter_node)
