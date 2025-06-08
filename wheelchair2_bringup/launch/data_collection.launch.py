import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                description='Launch in simulation mode',
            )
        )
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_realsense',
            default_value='True',
            description='Launch Realsense',
        )
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_realsense = LaunchConfiguration("use_realsense")


    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_bringup"), "launch", "wheelchair2.launch.py"]
            ),
        ),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            ),
        ),
        condition=IfCondition(use_realsense),
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"]
            ),
        ),
        condition=UnlessCondition(use_sim_time),
    )

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pointcloud_to_laserscan"), "launch", "sample_pointcloud_to_laserscan_launch.py"]
            ),
        ),
    )

    nodes =[
        bringup_launch,
        realsense_launch,
        livox_launch,
        pointcloud_to_laserscan_launch
    ]

    return LaunchDescription(launch_arguments + nodes)
