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
    PythonExpression,
)
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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

    launch_arguments.append(
            DeclareLaunchArgument(
                'ramp_nav',
                default_value='True',
                description='Enable ramp navigation logic (ground detection)'
            )
        )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    invert_lidar = LaunchConfiguration("invert_lidar")
    ramp_nav = LaunchConfiguration("ramp_nav")

    # If ramp_nav is True, use /points_nonground (output of C++ node)
    # otherwise use /livox/lidar directly
    pc_input_topic = PythonExpression([
        "'/points_nonground' if ", ramp_nav, " else '/livox/lidar'"
        # "'/patchworkpp/nonground' if ", ramp_nav, " else '/livox/lidar'"
    ])

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

    # Launching the C++ executable 'ground_detection_node'
    # instead of 'ground_detection_fast.py'
    ground_detection_node = Node(
        package='wheelchair2_bringup',
        executable='ground_detection_node_reuse_planes',
        # executable='ground_detection_node_reuse_planes',
        name='ground_detection',
        output='screen',
        parameters=[{
            'lidar_topic': '/livox/lidar_box',
            'publish_grid': True,
            'target_frame': 'base_footprint',
            'ground_thickness': 0.05,
            'max_step_height': 0.05,
            'cell_size': 0.25
        }],
        condition=IfCondition(ramp_nav)
    )
    
    # patchwork_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare("patchworkpp"), "launch", "patchworkpp.launch.py"]
    #         )
    #     ),
    #     launch_arguments={
    #         'visualize': 'False',
    #         'use_sim_time': use_sim_time,
    #         'cloud_topic': '/livox/lidar',
    #         'base_frame': 'livox_frame',
    #     }.items(),
    #     condition=IfCondition(ramp_nav),
    # )
    

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pointcloud_to_laserscan"), "launch", "sample_pointcloud_to_laserscan_launch.py"]
            ),
        ),
        launch_arguments={
            'cloud_in': pc_input_topic
        }.items(),
    )

    nodes =[
        livox_launch,
        ground_detection_node,
        # patchwork_launch,
        pointcloud_to_laserscan_launch
    ]

    return LaunchDescription(launch_arguments + nodes)
