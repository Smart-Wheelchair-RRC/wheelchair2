"""
Launch file for the wheelchair2 localization package.
- Loads parameters from YAML file if available.
- Inverts the LiDAR data if required.
- Runs ICP localization node.
- Launches Nav2 map server and lifecycle manager.
"""

from pathlib import Path

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SELF_PACKAGE_NAME = 'wheelchair2_localization'
SELF_PACKAGE_DIRECTORY = Path(get_package_share_directory(SELF_PACKAGE_NAME))
SELF_CONFIG_PATH = SELF_PACKAGE_DIRECTORY / 'config'
SELF_YAML_CONFIG = SELF_CONFIG_PATH / 'localization_config.yaml'

NAVIGATION_PACKAGE_NAME = 'wheelchair2_navigation'
NAVIGATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(NAVIGATION_PACKAGE_NAME))

ICP_LOCALIZATION_PACKAGE_NAME = 'icp_localization_ros2'
ICP_LOCALIZATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(ICP_LOCALIZATION_PACKAGE_NAME))
ICP_LOCALIZATION_CONFIG_PATH = ICP_LOCALIZATION_PACKAGE_DIRECTORY / 'config'

def load_yaml_parameters():
    """Load parameters from YAML file if available."""
    
    # Default parameter values
    yaml_parameters = {
        # core functionality parameters
        'lidar_inverted': 'true',
        'use_navigation': 'true',
        'use_sim_time': 'false',
        'use_mapper_server': 'true',
        'nav2_rviz_required': 'true',
        'publish_pcd_map': 'false',

        # frames and TFs
        'main_map_frame': 'map',
        'main_odom_frame': 'odom',

        # Topics
        'input_lidar_topic': '/livox/lidar',
        'inverted_lidar_topic': '/livox/lidar_inverted',
        'imu_topic': '/livox/imu',
        'inverted_imu_topic': '/livox/imu_inverted',
        'final_lidar_topic': '/livox/lidar_inverted',
        'final_imu_topic': '/livox/imu_inverted',
        'input_odom_topic': '/odom',

        # File paths
        'pcd_map_path': str(NAVIGATION_PACKAGE_DIRECTORY / 'maps' / 'rrc' / 'lab_map_with_outside_corridor.pcd'),
        'map_file_path': str(NAVIGATION_PACKAGE_DIRECTORY / 'maps' / 'rrc' / 'rrc_lab.yaml'),
        'icp_config_file': str(ICP_LOCALIZATION_CONFIG_PATH / 'icp.yaml'),
        'icp_node_params_file': str(ICP_LOCALIZATION_CONFIG_PATH / 'node_params.yaml'),
        'nav2_params_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'config' / 'nav2_params.yaml'),
        'nav2_rviz_config_file': str(NAVIGATION_PACKAGE_DIRECTORY / 'rviz' / 'navigation.rviz'),
    }

    # Try to load parameters from YAML file if available
    if SELF_YAML_CONFIG.exists():
        with open(SELF_YAML_CONFIG, 'r') as f:
            yaml_params_from_config = yaml.safe_load(f)
            
            # Convert YAML parameters to flat dictionary with default string values
            if yaml_params_from_config:

                if 'core' in yaml_params_from_config:
                    core_params = yaml_params_from_config['core']

                    if 'lidar_inverted' in core_params:
                        yaml_parameters['lidar_inverted'] = str(core_params['lidar_inverted']).lower()

                    if 'use_navigation' in core_params:
                        yaml_parameters['use_navigation'] = str(core_params['use_navigation']).lower()

                    if 'use_sim_time' in yaml_params_from_config:
                        yaml_parameters['use_sim_time'] = str(yaml_params_from_config['use_sim_time']).lower()

                    if 'nav2_rviz_required' in core_params:
                        yaml_parameters['nav2_rviz_required'] = str(core_params['nav2_rviz_required']).lower()

                    if 'use_mapper_server' in core_params:
                        yaml_parameters['use_mapper_server'] = str(core_params['use_mapper_server']).lower()

                if 'publish_pcd_map' in yaml_params_from_config:
                    yaml_parameters['publish_pcd_map'] = str(yaml_params_from_config['publish_pcd_map']).lower()

                if 'frames' in yaml_params_from_config:
                    frames_params = yaml_params_from_config['frames']
                    
                    if 'main_map_frame' in frames_params:
                        yaml_parameters['main_map_frame'] = frames_params['main_map_frame']

                    if 'main_odom_frame' in frames_params:
                        yaml_parameters['main_odom_frame'] = frames_params['main_odom_frame']

                if 'topics' in yaml_params_from_config:
                    topics_params = yaml_params_from_config['topics']
                    
                    if 'input_lidar_topic' in topics_params:
                        yaml_parameters['lidar_cloud_topic'] = topics_params['input_lidar_topic']
                    
                    if 'inverted_lidar_topic' in topics_params:
                        yaml_parameters['inverted_lidar_topic'] = topics_params['inverted_lidar_topic']
                    
                    if 'imu_topic' in topics_params:
                        yaml_parameters['imu_topic'] = topics_params['imu_topic']
                    
                    if 'inverted_imu_topic' in topics_params:
                        yaml_parameters['inverted_imu_topic'] = topics_params['inverted_imu_topic']
                    
                    if 'final_lidar_topic' in topics_params:
                        yaml_parameters['final_lidar_topic'] = topics_params['final_lidar_topic']

                    if 'final_imu_topic' in topics_params:
                        yaml_parameters['final_imu_topic'] = topics_params['final_imu_topic']

                    if 'input_odom_topic' in topics_params:
                        yaml_parameters['input_odom_topic'] = topics_params['input_odom_topic']

                if 'files' in yaml_params_from_config:
                    files_params = yaml_params_from_config['files']
                    
                    if 'pcd_map_path' in files_params and files_params['pcd_map_path']:
                        yaml_parameters['pcd_map_path'] = files_params['pcd_map_path']

                    if 'map_file_path' in files_params and files_params['map_file_path']:
                        yaml_parameters['map_file_path'] = files_params['map_file_path']

                    if 'icp_config_file' in files_params and files_params['icp_config_file']:
                        yaml_parameters['icp_config_file'] = files_params['icp_config_file']

                    if 'icp_node_params_file' in files_params and files_params['icp_node_params_file']:
                        yaml_parameters['icp_node_params_file'] = files_params['icp_node_params_file']

                    if 'nav2_params_file' in files_params and files_params['nav2_params_file']:
                        yaml_parameters['nav2_params_file'] = files_params['nav2_params_file']

                    if 'nav2_rviz_config_file' in files_params and files_params['nav2_rviz_config_file']:
                        yaml_parameters['nav2_rviz_config_file'] = files_params['nav2_rviz_config_file']
    
    return yaml_parameters

def declare_launch_arguments(param_defaults):
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
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    use_navigation = LaunchConfiguration('use_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_rviz_required = LaunchConfiguration('nav2_rviz_required')
    use_mapper_server = LaunchConfiguration('use_mapper_server')
    publish_pcd_map = LaunchConfiguration('publish_pcd_map')

    # frames and TFs
    main_map_frame_id = LaunchConfiguration('main_map_frame')
    main_odom_frame_id = LaunchConfiguration('main_odom_frame')

    # Topics
    lidar_cloud_topic = LaunchConfiguration('input_lidar_topic')
    inverted_lidar_topic = LaunchConfiguration('inverted_lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    inverted_imu_topic = LaunchConfiguration('inverted_imu_topic')
    final_lidar_topic = LaunchConfiguration('final_lidar_topic')
    final_imu_topic = LaunchConfiguration('final_imu_topic')
    input_odom_topic = LaunchConfiguration('input_odom_topic')

    # File paths
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    map_file_path = LaunchConfiguration('map_file_path')
    icp_config_file = LaunchConfiguration('icp_config_file')
    icp_node_params_file = LaunchConfiguration('icp_node_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file')

    # Create nodes
    nodes = []
    group_actions = []

    # PCD map publisher node
    nodes.append(Node(
        package=SELF_PACKAGE_NAME,
        executable='vizualize_pcd_map',
        name='vizualize_pcd_map',
        output='screen',
        parameters=[
            {'pcd_map_path': pcd_map_path},
            {'map_topic': '/pcd_map'},
            {'downsampling_voxel_size': 0.2},  # Default downsampling voxel
        ],
        condition=launch.conditions.IfCondition(publish_pcd_map)
    ))
    
    # Invert LiDAR data node
    nodes.append(Node(
        package=SELF_PACKAGE_NAME,
        executable='invert_livox_data',
        name='invert_livox_data',
        output='screen',
        parameters=[
            {'input_lidar_topic': lidar_cloud_topic},
            {'input_imu_topic': imu_topic},
            {'inverted_lidar_topic': inverted_lidar_topic},
            {'inverted_imu_topic': inverted_imu_topic},
        ],
        condition=launch.conditions.IfCondition(lidar_inverted)
    ))

    # ICP localization node
    nodes.append(Node(
        package=ICP_LOCALIZATION_PACKAGE_NAME,
        executable='icp_localization',
        output='screen',
        parameters=[
            icp_node_params_file,
            {
                'icp_config_path': icp_config_file,
                'input_filters_config_path': str(ICP_LOCALIZATION_CONFIG_PATH / 'input_filters_mid360.yaml'),
                'pcd_file_path': pcd_map_path,
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('/cloud_registered_body', final_lidar_topic),
            ('/imu/data', final_imu_topic),
            ('/Odometry', input_odom_topic),
        ]
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
            'pcd_file_path': pcd_map_path,
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

    param_defaults = load_yaml_parameters()
    launch_args = declare_launch_arguments(param_defaults)
    nodes = create_nodes()
    launch_description = launch.LaunchDescription(launch_args + nodes)
    
    return launch_description