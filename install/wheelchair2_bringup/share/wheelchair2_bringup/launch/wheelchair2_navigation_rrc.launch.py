import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument
)
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
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare launch arguments

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
            'use_keepout_zone',
            default_value='False',
            description='Launch with keepout zones'
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_joy',
            default_value='True',
            description='Use joystick control'
        )
    )

    #Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration('autostart', default="true")
    use_keepout_zone = LaunchConfiguration('use_keepout_zone')
    use_joy = LaunchConfiguration('use_joy')

    # lifecycle node for mask layer
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    
    wheelchair2_bringup_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_gazebo"), "launch", "wheelchair2_sim.launch.py"]
            ),
        ),
        condition=IfCondition(use_sim_time)
    )

    wheelchair2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_bringup"), "launch", "wheelchair2.launch.py"]
            ),
        ),
        condition=UnlessCondition(use_sim_time),
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch_ROS2", "msg_MID360_launch.py"]
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

    keepout_params_file = LaunchConfiguration(
        'keepout_params_file',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "keepout_params.yaml",
        ),
    )
                                                
    mask_yaml_file = LaunchConfiguration(
        'mask',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab_mask.yaml",
        ),
    )

    # Map and Nav2 config files
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab.yaml",
        ),
    )
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "nav2_params.yaml",
        ),
    )
    param_substitutions = {
            'use_sim_time': use_sim_time,
            'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
            source_file=keepout_params_file,
            param_rewrites=param_substitutions,
            convert_types=True)
    

    # Nav2 bringup launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
    )

    rviz_config = os.path.join(get_package_share_directory('wheelchair2_navigation'), 'rviz', 'navigation.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,  
            condition=IfCondition(use_keepout_zone),
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_keepout_zone),
            parameters=[configured_params])

    costmap_filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  
            condition=IfCondition(use_keepout_zone),
            parameters=[configured_params])

    nodes = [
            wheelchair2_bringup_sim,
            wheelchair2_bringup,
            livox_launch,
            pointcloud_to_laserscan_launch,
            nav2,
            rviz,
            lifecycle_manager,
            map_server,
            costmap_filter_info_server,  
    ]

    return LaunchDescription(launch_arguments + nodes)