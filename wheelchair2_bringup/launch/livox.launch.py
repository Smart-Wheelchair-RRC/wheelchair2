from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_arguments = []
    launch_arguments.append(
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                description='Launch in simulation mode'
            )
        )
    
    launch_arguments.append(
            DeclareLaunchArgument(
                'invert_lidar',
                default_value='True',
                description='Invert Livox Lidar Data'
            )
        )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    invert_lidar = LaunchConfiguration("invert_lidar")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"]
            ),
        ),
        launch_arguments={
            'invert_lidar': invert_lidar,
        }.items(),
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
        livox_launch,
        pointcloud_to_laserscan_launch
    ]

    return LaunchDescription(launch_arguments + nodes)
