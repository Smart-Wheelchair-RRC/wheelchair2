"""
Launch file for the wheelchair2 fast_lio odometry package.
- Inverts the LiDAR data if required.
- Publishes a static transform from `map` to `camera_init` if required.
- Launches the fast_lio node with a specified configuration file.
- Loads parameters from YAML file if available.
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


def load_yaml_parameters():
    """Load parameters from YAML file if available."""
    SELF_PACKAGE_DIRECTORY = Path(get_package_share_directory('wheelchair2_fast_lio'))
    config_path = SELF_PACKAGE_DIRECTORY / 'config'
    yaml_config_path = config_path / 'fast_lio_package_params.yaml'
    
    # Default parameter values
    DEFAULT_PARAMATER_VALUES = {
        'lidar_inverted': 'true',
        'init_frame_tf_required': 'true',
        'fast_lio_config': 'fast_lio_mapping_mid360.yaml',
        'fast_lio_config_path': str(config_path),
        'use_sim_time': 'false',
        'fast_lio_rviz_required': 'false',
        'fast_lio_rviz_config_file': str(Path(get_package_share_directory('fast_lio')) / 'rviz' / 'fastlio.rviz'),
        'use_localization': 'true',
        'pcd_map_path': 'you-super-accurate-map.pcd',
        'publish_3d_map': 'false',
        'use_map_cropping': 'true',
        'map_crop_distance': '20.0',
        'icp_threshold': '0.8',
        'icp_max_iterations': '50',
        'lidar_cloud_topic': '/livox/lidar',
        'fast_lio_odom_topic': '/Odometry',
        'initialpose_topic': '/initialpose',
        'localization_pose_topic': '/localization_pose',
        'odom_frame_id': 'odom',
        'initial_frame_id': 'camera_init',
        'loc_broadcast_tf': 'true',
        'loc_global_frame': 'map',
        'navigation_mode': 'true',
        'nav2_params_file': str(Path(get_package_share_directory('wheelchair2_navigation')) / 'config' / 'nav2_params.yaml'),
        'map_file_path': str(Path(get_package_share_directory('wheelchair2_navigation')) / 'maps' / 'rrc' / 'rrc_lab.yaml'),
        'nav2_rviz_config_file': str(Path(get_package_share_directory('wheelchair2_navigation')) / 'rviz' / 'navigation.rviz'),
        'nav2_rviz_required': 'false',
    }

    # Try to load parameters from YAML file if available
    if yaml_config_path.exists():
        with open(yaml_config_path, 'r') as f:
            yaml_params = yaml.safe_load(f)
            
            # Convert YAML parameters to flat dictionary with default string values
            if yaml_params:
                # Use sim time
                if 'use_sim_time' in yaml_params:
                    DEFAULT_PARAMATER_VALUES['use_sim_time'] = str(yaml_params['use_sim_time']).lower()
                
                # Fast-LIO odometry parameters
                if 'fast_lio_odometry' in yaml_params:
                    fast_lio_params = yaml_params['fast_lio_odometry']
                    
                    if 'config_file' in fast_lio_params:
                        DEFAULT_PARAMATER_VALUES['fast_lio_config'] = fast_lio_params['config_file']
                    
                    if 'config_path' in fast_lio_params and fast_lio_params['config_path']:
                        DEFAULT_PARAMATER_VALUES['fast_lio_config_path'] = fast_lio_params['config_path']
                    
                    if 'init_frame_required' in fast_lio_params:
                        DEFAULT_PARAMATER_VALUES['init_frame_tf_required'] = str(fast_lio_params['init_frame_required']).lower()
                    
                    if 'lidar_inverted' in fast_lio_params:
                        DEFAULT_PARAMATER_VALUES['lidar_inverted'] = str(fast_lio_params['lidar_inverted']).lower()
                    
                    if 'odom_frame_id' in fast_lio_params: # Shared odom frame
                        DEFAULT_PARAMATER_VALUES['odom_frame_id'] = fast_lio_params['odom_frame_id']
                    
                    if 'initial_frame_id' in fast_lio_params: # For TF node
                        DEFAULT_PARAMATER_VALUES['initial_frame_id'] = fast_lio_params['initial_frame_id']

                    if 'rviz' in fast_lio_params:
                        if 'required' in fast_lio_params['rviz']:
                            DEFAULT_PARAMATER_VALUES['fast_lio_rviz_required'] = str(fast_lio_params['rviz']['required']).lower()
                        if 'config_file' in fast_lio_params['rviz'] and fast_lio_params['rviz']['config_file']:
                            DEFAULT_PARAMATER_VALUES['fast_lio_rviz_config_file'] = fast_lio_params['rviz']['config_file']
                
                # Localization parameters
                if 'localization' in yaml_params:
                    localization_params = yaml_params['localization']
                    
                    if 'enabled' in localization_params:
                        DEFAULT_PARAMATER_VALUES['use_localization'] = str(localization_params['enabled']).lower()
                    
                    if 'pcd_map_path' in localization_params:
                        DEFAULT_PARAMATER_VALUES['pcd_map_path'] = localization_params['pcd_map_path']
                    
                    # Add the new localization parameters
                    if 'publish_3d_map' in localization_params:
                        DEFAULT_PARAMATER_VALUES['publish_3d_map'] = str(localization_params['publish_3d_map']).lower()
                    
                    if 'use_map_cropping' in localization_params:
                        DEFAULT_PARAMATER_VALUES['use_map_cropping'] = str(localization_params['use_map_cropping']).lower()
                    
                    if 'map_crop_distance' in localization_params:
                        DEFAULT_PARAMATER_VALUES['map_crop_distance'] = str(localization_params['map_crop_distance'])
                    
                    if 'icp_threshold' in localization_params:
                        DEFAULT_PARAMATER_VALUES['icp_threshold'] = str(localization_params['icp_threshold'])
                    
                    if 'icp_max_iterations' in localization_params:
                        DEFAULT_PARAMATER_VALUES['icp_max_iterations'] = str(localization_params['icp_max_iterations'])
                
                    # Localization specific TF parameters
                    if 'broadcast_tf' in localization_params:
                        DEFAULT_PARAMATER_VALUES['loc_broadcast_tf'] = str(localization_params['broadcast_tf']).lower()
                    
                    if 'global_frame' in localization_params:
                        DEFAULT_PARAMATER_VALUES['loc_global_frame'] = localization_params['global_frame']
                    
                    # If localization.odom_frame_id is present in YAML and differs, 
                    # it will be overridden by fast_lio_odometry.odom_frame_id
                    # to ensure sharing, as per current logic.
                    # If it's critical for localization to have a *different* odom_frame_id,
                    # then the sharing logic needs to be revisited.
                    # For now, we assume they share the same odom_frame_id.

                # Topic parameters
                if 'topics' in yaml_params:
                    topics = yaml_params['topics']
                    
                    if 'lidar_cloud' in topics:
                        DEFAULT_PARAMATER_VALUES['lidar_cloud_topic'] = topics['lidar_cloud']
                    
                    if 'fast_lio_odom' in topics:
                        DEFAULT_PARAMATER_VALUES['fast_lio_odom_topic'] = topics['fast_lio_odom']
                    
                    if 'initialpose' in topics:
                        DEFAULT_PARAMATER_VALUES['initialpose_topic'] = topics['initialpose']
                    
                    if 'localization_pose' in topics:
                        DEFAULT_PARAMATER_VALUES['localization_pose_topic'] = topics['localization_pose']

                # Nav2 parameters
                if 'nav2' in yaml_params:
                    nav2_params = yaml_params['nav2']

                    if 'navigation_mode' in nav2_params:
                        DEFAULT_PARAMATER_VALUES['navigation_mode'] = str(nav2_params['navigation_mode']).lower()
                    
                    if 'params_file' in nav2_params:
                        DEFAULT_PARAMATER_VALUES['nav2_params_file'] = nav2_params['params_file']
                    
                    if 'map_file_path' in nav2_params:
                        DEFAULT_PARAMATER_VALUES['map_file_path'] = nav2_params['map_file_path']
                    
                    if 'rviz_config_file' in nav2_params:
                        DEFAULT_PARAMATER_VALUES['nav2_rviz_config_file'] = nav2_params['rviz_config_file']

                    if 'rviz_required' in nav2_params:
                        DEFAULT_PARAMATER_VALUES['nav2_rviz_required'] = str(nav2_params['rviz_required']).lower()
    
    return DEFAULT_PARAMATER_VALUES

def declare_launch_arguments(param_defaults):
    """Declare all launch arguments with values from parameters."""
    launch_args = []
    
    # FAST-LIO odometry parameters
    launch_args.append(DeclareLaunchArgument(
        'lidar_inverted',
        default_value=param_defaults['lidar_inverted'],
        description='Whether to invert the lidar data'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'init_frame_tf_required',
        default_value=param_defaults['init_frame_tf_required'],
        description='Whether to publish tf from map to camera_init'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_config',
        default_value=param_defaults['fast_lio_config'],
        description='Path to the fast_lio config file'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_config_path',
        default_value=param_defaults['fast_lio_config_path'],
        description='Path to the fast_lio config directory'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value=param_defaults['use_sim_time'],
        description='Use simulation time'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_rviz_required',
        default_value=param_defaults['fast_lio_rviz_required'],
        description='Whether to launch RViz with fast_lio'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_rviz_config_file', 
        default_value=param_defaults['fast_lio_rviz_config_file'],
        description='RViz config file path'
    ))
    
    # FAST-LIO localization parameters
    launch_args.append(DeclareLaunchArgument(
        'use_localization',
        default_value=param_defaults['use_localization'],
        description='Whether to use localization mode'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'pcd_map_path',
        default_value=param_defaults['pcd_map_path'],
        description='Path to the PCD map file'
    ))
    
    # New localization parameters
    launch_args.append(DeclareLaunchArgument(
        'publish_3d_map',
        default_value=param_defaults['publish_3d_map'],
        description='Whether to publish the 3D map for visualization'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'use_map_cropping',
        default_value=param_defaults['use_map_cropping'],
        description='Whether to crop the map to a specified distance from robot'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'map_crop_distance',
        default_value=param_defaults['map_crop_distance'],
        description='The radius in meters for map cropping'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'icp_threshold',
        default_value=param_defaults['icp_threshold'],
        description='ICP convergence threshold'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'icp_max_iterations',
        default_value=param_defaults['icp_max_iterations'],
        description='Maximum number of ICP iterations'
    ))
    
    # Topic parameters
    launch_args.append(DeclareLaunchArgument(
        'lidar_cloud_topic',
        default_value=param_defaults['lidar_cloud_topic'],
        description='Topic for the LiDAR point cloud data'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_odom_topic',
        default_value=param_defaults['fast_lio_odom_topic'],
        description='Topic for the FAST-LIO odometry data'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'initialpose_topic',
        default_value=param_defaults['initialpose_topic'],
        description='Topic for the initial pose estimate'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'localization_pose_topic',
        default_value=param_defaults['localization_pose_topic'],
        description='Topic for the localization pose output'
    ))

    # Shared odom_frame_id for both localization and tf_init_frame node
    launch_args.append(DeclareLaunchArgument(
        'odom_frame_id',
        default_value=param_defaults['odom_frame_id'],
        description='Shared odometry frame ID for TF and localization nodes'
    ))

    # Frame ID for the initial camera pose (for tf_fast_lio_init_frame node)
    launch_args.append(DeclareLaunchArgument(
        'initial_frame_id',
        default_value=param_defaults['initial_frame_id'],
        description='Initial camera frame ID for tf_fast_lio_init_frame node'
    ))
    
    # TF parameters for localization node
    launch_args.append(DeclareLaunchArgument(
        'loc_broadcast_tf',
        default_value=param_defaults['loc_broadcast_tf'],
        description='Whether localization node broadcasts map to odom TF'
    ))
    launch_args.append(DeclareLaunchArgument(
        'loc_global_frame',
        default_value=param_defaults['loc_global_frame'],
        description='Global frame ID for localization node (e.g., map)'
    ))

    # Nav2 parameters
    launch_args.append(DeclareLaunchArgument(
        'navigation_mode',
        default_value=param_defaults['navigation_mode'],
        description='Whether to enable navigation mode with Nav2'
    ))

    launch_args.append(DeclareLaunchArgument(
        'nav2_params_file',
        default_value=str(Path(get_package_share_directory('wheelchair2_navigation')) / 'config' / 'nav2_params.yaml'),
        description='Path to the Nav2 parameters file'
    ))

    launch_args.append(DeclareLaunchArgument(
        'map_file_path',
        default_value=str(Path(get_package_share_directory('wheelchair2_navigation')) / 'maps' / 'rrc' / 'rrc_lab.yaml'),
        description='Path to the map file for Nav2'
    ))

    launch_args.append(DeclareLaunchArgument(
        'nav2_rviz_config_file',
        default_value=str(Path(get_package_share_directory('wheelchair2_navigation')) / 'rviz' / 'navigation.rviz'),
        description='Path to the RViz config file for Nav2'
    ))
    
    launch_args.append(DeclareLaunchArgument(
        'nav2_rviz_required',
        default_value='false',
        description='Whether to launch RViz for Nav2 visualization'
    ))

    return launch_args

def create_nodes():
    """Create all the nodes needed for the launch."""
    # Create LaunchConfigurations
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    init_fram_tf_required = LaunchConfiguration('init_frame_tf_required')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config')
    fast_lio_config_path = LaunchConfiguration('fast_lio_config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_rviz_required = LaunchConfiguration('fast_lio_rviz_required')
    fast_lio_rviz_config_file = LaunchConfiguration('fast_lio_rviz_config_file')
    use_localization = LaunchConfiguration('use_localization')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    
    # New localization parameters
    publish_3d_map = LaunchConfiguration('publish_3d_map')
    use_map_cropping = LaunchConfiguration('use_map_cropping')
    map_crop_distance = LaunchConfiguration('map_crop_distance')
    icp_threshold = LaunchConfiguration('icp_threshold')
    icp_max_iterations = LaunchConfiguration('icp_max_iterations')
    
    # Topic parameters
    lidar_cloud_topic = LaunchConfiguration('lidar_cloud_topic')
    fast_lio_odom_topic = LaunchConfiguration('fast_lio_odom_topic')
    initialpose_topic = LaunchConfiguration('initialpose_topic')
    localization_pose_topic = LaunchConfiguration('localization_pose_topic')

    # Shared and specific frame LaunchConfigurations
    odom_frame_id = LaunchConfiguration('odom_frame_id') # Shared
    initial_frame_id = LaunchConfiguration('initial_frame_id') # For TF node
    loc_broadcast_tf = LaunchConfiguration('loc_broadcast_tf') # For localization node
    loc_global_frame = LaunchConfiguration('loc_global_frame') # For localization node

    # Nav2 parameters
    navigation_mode = LaunchConfiguration('navigation_mode')
    nav2_parameters_file = LaunchConfiguration('nav2_params_file')
    map_file_path = LaunchConfiguration('map_file_path')
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file')
    nav2_rviz_required = LaunchConfiguration('nav2_rviz_required')

    
    # Create nodes
    nodes = []
    group_actions = []
    
    # Invert LiDAR data node
    nodes.append(Node(
        package='wheelchair2_fast_lio',
        executable='invert_livox_data',
        name='invert_livox_data',
        output='screen',
        condition=launch.conditions.IfCondition(lidar_inverted)
    ))
    
    # TF for Fast-LIO init frame
    nodes.append(Node(
        package='wheelchair2_fast_lio',
        executable='tf_fast_lio_init_frame',
        name='tf_fast_lio_init_frame',
        output='screen',
        parameters=[
            {'odom_frame_id': odom_frame_id}, # Use shared odom_frame_id
            {'initial_frame_id': initial_frame_id}
        ],
        condition=launch.conditions.IfCondition(init_fram_tf_required)
    ))
    
    # Fast-LIO node
    nodes.append(Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            launch.substitutions.PathJoinSubstitution([fast_lio_config_path, fast_lio_config_file]),
            {'use_sim_time': use_sim_time,}
        ]
    ))
    
    # Localization node with updated parameters
    nodes.append(Node(
        package='wheelchair2_fast_lio',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[
            {'pcd_map_path': pcd_map_path},
            {'lidar_cloud_topic': lidar_cloud_topic},
            {'fast_lio_odom_topic': fast_lio_odom_topic},
            {'initialpose_topic': initialpose_topic},
            {'localization_pose_topic': localization_pose_topic},
            {'publish_3d_map': publish_3d_map},
            {'use_map_cropping': use_map_cropping},
            {'map_crop_distance': map_crop_distance},
            {'icp_threshold': icp_threshold},
            {'icp_max_iterations': icp_max_iterations},
            # TF parameters for localization node
            {'broadcast_tf': loc_broadcast_tf},
            {'global_frame': loc_global_frame},
            {'odom_frame_id': odom_frame_id} # Use shared odom_frame_id
        ],
        condition=launch.conditions.IfCondition(use_localization)
    ))

    # Mapper server node
    nodes.append(Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_path,
            'frame_id': loc_global_frame
        }],
        condition=launch.conditions.IfCondition(navigation_mode)
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
        condition=launch.conditions.IfCondition(navigation_mode)
    ))

    # Nav2 bringup for navigation
    group_actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
        condition=launch.conditions.IfCondition(navigation_mode)
    ))

    # RViz node for FAST-LIO visualization
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_fast_lio',
        arguments=['-d', fast_lio_rviz_config_file],
        condition=launch.conditions.IfCondition(fast_lio_rviz_required)
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