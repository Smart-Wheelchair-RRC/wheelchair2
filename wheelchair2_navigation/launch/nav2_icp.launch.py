"""
Launch file for Nav2 stack, using custom localization from the Wheelchair2 project.
- Loads parameters from CLI if available, else uses defaults.
- Runs the map server.
- Runs the main Nav2 stack (except map server and AMCL).
- Optionally runs RViz for visualization.
"""

from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WHEELCHAIR_LOCALIZATION_PACKAGE_NAME = 'wheelchair2_localization'
WHEELCHAIR_LOCALIZATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(WHEELCHAIR_LOCALIZATION_PACKAGE_NAME))

NAVIGATION_PACKAGE_NAME = 'wheelchair2_navigation'
NAVIGATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(NAVIGATION_PACKAGE_NAME))

DEFAULT_PARAMETERS = {
    # core functionality parameters
    'use_navigation': 'true',
    'use_sim_time': 'false',
    'use_mapper_server': 'true',
    'nav2_rviz_required': 'true',

    # frames and TFs
    'main_map_frame': 'map',
    'main_odom_frame': 'odom',

    # File paths
    'map_file_path': str(NAVIGATION_PACKAGE_DIRECTORY / 'maps' / 'rrc' / 'rrc_lab.yaml'),
    'nav2_params_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'nav2_params.yaml'),
    'nav2_rviz_config_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'rviz' / 'navigation.rviz'),
}

def declare_launch_arguments(param_defaults: dict):
    """Declare all launch arguments with values from parameters."""
    launch_args = []
    
    for param_name, default_value in param_defaults.items():
        launch_args.append(DeclareLaunchArgument(
            param_name,
            default_value=default_value,
            description=f"Launch argument for {param_name}"
        ))

    return launch_args

def create_nodes():
    """Create all the nodes needed for the launch."""

    # core
    use_navigation = LaunchConfiguration('use_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_rviz_required = LaunchConfiguration('nav2_rviz_required')
    use_mapper_server = LaunchConfiguration('use_mapper_server')

    # frames and TFs
    main_map_frame_id = LaunchConfiguration('main_map_frame')
    main_odom_frame_id = LaunchConfiguration('main_odom_frame')

    # File paths
    map_file_path = LaunchConfiguration('map_file_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file')

    # Create nodes
    nodes = []
    group_actions = []

    # Localization node
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(WHEELCHAIR_LOCALIZATION_PACKAGE_DIRECTORY), '/launch', '/localization_icp.launch.py']
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    ))

    # Mapper server node
    nodes.append(Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_path,
            'frame_id': main_map_frame_id,
        }],
        condition=launch.conditions.IfCondition(use_mapper_server)
    ))

    # Mapper server lifecycle manager
    group_actions.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(use_mapper_server)
    ))

    # Nav2 bringup for navigation
    group_actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
        condition=launch.conditions.IfCondition(use_navigation)
    ))


    # RViz node for nav2 visualization
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav2',
        arguments=['-d', nav2_rviz_config_file],
        condition=launch.conditions.IfCondition(nav2_rviz_required)
    ))

    nodes.append(GroupAction(actions=group_actions))
    return nodes

def generate_launch_description():
    """Generate the launch description with all components."""

    launch_args = declare_launch_arguments(DEFAULT_PARAMETERS)
    nodes = create_nodes()
    launch_description = launch.LaunchDescription(launch_args + nodes)
    
    return launch_description