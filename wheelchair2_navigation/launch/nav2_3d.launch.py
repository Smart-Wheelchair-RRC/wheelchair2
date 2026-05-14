"""
Launch file for Nav2 stack, decoupled from localization.
- Enforces mandatory 2D map path from CLI.
- Runs box_filter immediately.
- Runs the map server and Nav2 stack (after a 2s delay).
- Optionally runs RViz for visualization (after a 4s delay).
"""

from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


NAVIGATION_PACKAGE_NAME = 'wheelchair2_navigation'
NAVIGATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(NAVIGATION_PACKAGE_NAME))

DEFAULT_PARAMETERS = {
    # core functionality parameters
    'use_navigation': 'true',
    'use_sim_time': 'false',
    'use_mapper_server': 'true',
    'nav2_rviz_required': 'true',
    'use_keepout_zone': 'false',

    # frames and TFs
    'main_map_frame': 'map',
    'main_odom_frame': 'odom',

    # File paths (Note: map is omitted here to make it a mandatory argument)
    'keepout_mask_file_path': str(NAVIGATION_PACKAGE_DIRECTORY / 'maps' / 'rrc' / 'rrc_lab_mask.yaml'),
    'nav2_params_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'nav2_params.yaml'),
    'nav2_rviz_config_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'rviz' / 'navigation.rviz'),
    'keepout_params_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'keepout_params.yaml'),

    'box_filter_config': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'box_filter.yaml'),
    'box_filter_config_2': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'box_filter_2.yaml'),
}

def declare_launch_arguments(param_defaults: dict):
    """Declare all launch arguments with values from parameters."""
    launch_args = []

    # 1. Mandatory argument for map (no default value)
    launch_args.append(DeclareLaunchArgument(
        'map',
        description="Mandatory: Full path to the 2D map yaml file to load"
    ))
    
    # 2. Add the rest of the arguments with defaults
    for param_name, default_value in param_defaults.items():
        launch_args.append(DeclareLaunchArgument(
            param_name,
            default_value=default_value,
            description=f"Launch argument for {param_name}"
        ))

    return launch_args

def create_nodes():
    """Create all the nodes and timed actions needed for the launch."""

    # core
    use_navigation = LaunchConfiguration('use_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_rviz_required = LaunchConfiguration('nav2_rviz_required')
    use_mapper_server = LaunchConfiguration('use_mapper_server')
    use_keepout_zone = LaunchConfiguration('use_keepout_zone')

    # frames and TFs
    main_map_frame_id = LaunchConfiguration('main_map_frame')
    main_odom_frame_id = LaunchConfiguration('main_odom_frame')

    # File paths
    map = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file')
    keepout_mask_file_path = LaunchConfiguration('keepout_mask_file_path')
    keepout_params_file = LaunchConfiguration('keepout_params_file')

    updated_keepout_params = RewrittenYaml(
        source_file=keepout_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask_file_path
        },
        convert_types=True
    )

    actions_list = []

    # ==========================================
    # Phase 1: Immediate Startup
    # ==========================================
    # ==========================================
    # Phase 2: Nav2 Stack (2 Second Delay)
    # ==========================================
    
    nav2_group_actions = []

    # Lifecycle manager for map
    nav2_group_actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(use_mapper_server)
    ))

    # Lifecycle manager for keepout zones
    lifecycle_nodes_keepout = ['filter_mask_server', 'costmap_filter_info_server']
    nav2_group_actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes_keepout,
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(use_keepout_zone)
    ))

    # Mapper server node
    nav2_group_actions.append(Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map,
            'frame_id': main_map_frame_id,
        }],
        condition=launch.conditions.IfCondition(use_mapper_server)
    ))

    # Keepout zone filter nodes
    nav2_group_actions.append(Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,  
        condition=launch.conditions.IfCondition(use_keepout_zone),
        parameters=[updated_keepout_params]
    ))
    
    nav2_group_actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        condition=launch.conditions.IfCondition(use_keepout_zone),
        parameters=[updated_keepout_params]
    ))

    # Nav2 bringup for navigation
    nav2_group_actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
        condition=launch.conditions.IfCondition(use_navigation)
    ))

    # Wrap Nav2 nodes in a 2-second TimerAction
    actions_list.append(TimerAction(
        period=2.0,
        actions=[GroupAction(actions=nav2_group_actions)]
    ))


    # ==========================================
    # Phase 3: RViz (4 Second Delay)
    # ==========================================
    
    # RViz node for nav2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav2',
        arguments=['-d', nav2_rviz_config_file],
        condition=launch.conditions.IfCondition(nav2_rviz_required)
    )

    # Wrap RViz in a 4-second TimerAction
    actions_list.append(TimerAction(
        period=4.0,
        actions=[rviz_node]
    ))

    return actions_list

def generate_launch_description():
    """Generate the launch description with all components."""

    launch_args = declare_launch_arguments(DEFAULT_PARAMETERS)
    actions = create_nodes()
    
    return launch.LaunchDescription(launch_args + actions)