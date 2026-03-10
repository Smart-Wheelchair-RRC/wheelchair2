import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    # Directories
    wheelchair_nav_dir = get_package_share_directory("wheelchair2_navigation")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_yaml = LaunchConfiguration(
        "map",
        # default=os.path.join(wheelchair_nav_dir, "maps", "rrc", "rrc_lab.yaml"),
        # default="/home/container_user/wheelchair2/src/change_detection/cartograph/aligned_results/probabilistic_master.yaml"
        default=os.path.join(wheelchair_nav_dir, "maps", "ramps", "rrc_outside_himalaya_ramps_fast_cleaned.yaml"),
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(wheelchair_nav_dir, "config", "nav2_params.yaml"),
    )
    use_keepout_zone = LaunchConfiguration('use_keepout_zone', default='False')
    keepout_params_file = LaunchConfiguration(
        'keepout_params_file',
        default=os.path.join(wheelchair_nav_dir, "config", "keepout_params.yaml"),
    )
    keepout_mask_file_path = LaunchConfiguration(
        'keepout_mask_file_path',
        default=os.path.join(wheelchair_nav_dir, "maps", "rrc", "rrc_lab_mask.yaml"),
    )
    rviz_config_file = LaunchConfiguration(
        'rviz_config_file',
        default=os.path.join(wheelchair_nav_dir, 'rviz', 'navigation.rviz')
    )
    run_rviz = LaunchConfiguration('run_rviz', default='true')

    # RewrittenYaml for keepout zones
    updated_keepout_params = RewrittenYaml(
        source_file=keepout_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask_file_path
        },
        convert_types=True
    )

    lifecycle_nodes_keepout = ['filter_mask_server', 'costmap_filter_info_server']
    lifecycle_nodes_nav = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("map", default_value=map_yaml, description="Full path to map file to load"),
        DeclareLaunchArgument("params_file", default_value=params_file, description="Full path to Nav2 parameters file"),
        DeclareLaunchArgument('use_keepout_zone', default_value='False', description='Launch with keepout zones'),
        DeclareLaunchArgument("keepout_params_file", default_value=keepout_params_file, description="Full path to keepout zone parameters file"),
        DeclareLaunchArgument("keepout_mask_file_path", default_value=keepout_mask_file_path, description="Full path to keepout zone mask file"),
        DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_file, description='Full path to the RVIZ config file to use'),
        DeclareLaunchArgument('run_rviz', default_value='true', description='Run RViz'),

        # Odometry (DLIO)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('direct_lidar_inertial_odometry'), 'launch', 'dlio.launch.py')
            ),
            launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()
        ),

        # Localization (3D-BBS and ICP)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheelchair2_3dbbs'), 'launch', '3dbbs.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('icp_localization_ros2'), 'launch', 'bringup.launch.py')
            ),
            launch_arguments={'rate': '3.0', 'use_sim_time': use_sim_time}.items()
        ),

        # Nav2 Stack
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_yaml}],
        ),
        GroupAction(
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='costmap_filter_info_server',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_keepout_zone),
                    parameters=[updated_keepout_params]
                ),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='filter_mask_server',
                    output='screen',
                    emulate_tty=True,
                    condition=IfCondition(use_keepout_zone),
                    parameters=[updated_keepout_params]
                ),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'True',
            }.items(),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_keepout_zone),
            parameters=[{'autostart': True, 'node_names': lifecycle_nodes_keepout, 'use_sim_time': use_sim_time}]
        ),

        #RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(run_rviz),
            output='screen'
        ),
    ])
