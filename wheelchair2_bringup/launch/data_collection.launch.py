import os
import datetime
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.actions import TimerAction
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
    launch_arguments.append(
        DeclareLaunchArgument(
            'record',
            default_value='True',
            description='Record in rosbag'
        )
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_realsense = LaunchConfiguration("use_realsense")
    record = LaunchConfiguration("record")


    # set log output path
    get_current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_full_path = os.path.join('/home/wheelchair2/wheelchair2/src/records/', get_current_timestamp)
    rosbag_full_path = os.path.join(log_full_path, 'rosbag')
    package_path = get_package_share_directory('wheelchair2_bringup')


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

    invert_lidar_node = Node(
        package='wheelchair2_bringup',
        executable='invert_lidar.py',
        name='invert_lidar_node',
        output='screen'
    )

    rosbag_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_path, '/launch/rosbag_recorder.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbag_storage_dir': rosbag_full_path,
        }.items(),
        condition=IfCondition(record),
    )

    rosbag_with_delay = TimerAction(
        period=5.0,  # 5 second delay
        actions=[rosbag_recorder_launch],
        condition=IfCondition(record),
    )

    nodes =[
        bringup_launch,
        realsense_launch,
        livox_launch,
        pointcloud_to_laserscan_launch,
        invert_lidar_node,
        rosbag_with_delay,
    ]

    return LaunchDescription(launch_arguments + nodes)
