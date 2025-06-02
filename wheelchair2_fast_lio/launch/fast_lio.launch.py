"""
Launch file for the wheelchair2 fast_lio odometry package.
- Inverts the LiDAR data if required.
- Publishes a static transform from `map` to `camera_init` if required.
- Launches the fast_lio node with a specified configuration file.
- Loads parameters from YAML file if available.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def load_yaml_parameters():
    """Load parameters from YAML file if available."""
    package_dir = Path(get_package_share_directory('wheelchair2_fast_lio'))
    config_path = package_dir / 'config'
    yaml_config_path = config_path / 'fast_lio_package_params.yaml'
    
    # Default parameter values
    param_defaults = {
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
    }

    # Try to load parameters from YAML file if available
    if yaml_config_path.exists():
        with open(yaml_config_path, 'r') as f:
            yaml_params = yaml.safe_load(f)
            
            # Convert YAML parameters to flat dictionary with default string values
            if yaml_params:
                # Use sim time
                if 'use_sim_time' in yaml_params:
                    param_defaults['use_sim_time'] = str(yaml_params['use_sim_time']).lower()
                
                # Fast-LIO odometry parameters
                if 'fast_lio_odometry' in yaml_params:
                    fast_lio_params = yaml_params['fast_lio_odometry']
                    
                    if 'config_file' in fast_lio_params:
                        param_defaults['fast_lio_config'] = fast_lio_params['config_file']
                    
                    if 'config_path' in fast_lio_params and fast_lio_params['config_path']:
                        param_defaults['fast_lio_config_path'] = fast_lio_params['config_path']
                    
                    if 'init_frame_required' in fast_lio_params:
                        param_defaults['init_frame_tf_required'] = str(fast_lio_params['init_frame_required']).lower()
                    
                    if 'lidar_inverted' in fast_lio_params:
                        param_defaults['lidar_inverted'] = str(fast_lio_params['lidar_inverted']).lower()
                    
                    if 'rviz' in fast_lio_params:
                        if 'required' in fast_lio_params['rviz']:
                            param_defaults['fast_lio_rviz_required'] = str(fast_lio_params['rviz']['required']).lower()
                        if 'config_file' in fast_lio_params['rviz'] and fast_lio_params['rviz']['config_file']:
                            param_defaults['fast_lio_rviz_config_file'] = fast_lio_params['rviz']['config_file']
                
                # Localization parameters
                if 'localization' in yaml_params:
                    localization_params = yaml_params['localization']
                    
                    if 'enabled' in localization_params:
                        param_defaults['use_localization'] = str(localization_params['enabled']).lower()
                    
                    if 'pcd_map_path' in localization_params:
                        param_defaults['pcd_map_path'] = localization_params['pcd_map_path']
                    
                    # Add the new localization parameters
                    if 'publish_3d_map' in localization_params:
                        param_defaults['publish_3d_map'] = str(localization_params['publish_3d_map']).lower()
                    
                    if 'use_map_cropping' in localization_params:
                        param_defaults['use_map_cropping'] = str(localization_params['use_map_cropping']).lower()
                    
                    if 'map_crop_distance' in localization_params:
                        param_defaults['map_crop_distance'] = str(localization_params['map_crop_distance'])
                    
                    if 'icp_threshold' in localization_params:
                        param_defaults['icp_threshold'] = str(localization_params['icp_threshold'])
                    
                    if 'icp_max_iterations' in localization_params:
                        param_defaults['icp_max_iterations'] = str(localization_params['icp_max_iterations'])
                
                # Topic parameters
                if 'topics' in yaml_params:
                    topics = yaml_params['topics']
                    
                    if 'lidar_cloud' in topics:
                        param_defaults['lidar_cloud_topic'] = topics['lidar_cloud']
                    
                    if 'fast_lio_odom' in topics:
                        param_defaults['fast_lio_odom_topic'] = topics['fast_lio_odom']
                    
                    if 'initialpose' in topics:
                        param_defaults['initialpose_topic'] = topics['initialpose']
                    
                    if 'localization_pose' in topics:
                        param_defaults['localization_pose_topic'] = topics['localization_pose']
    
    return param_defaults

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
    
    # Create nodes
    nodes = []
    
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
            {'icp_max_iterations': icp_max_iterations}
        ],
        condition=launch.conditions.IfCondition(use_localization)
    ))
    
    # RViz node
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', fast_lio_rviz_config_file],
        condition=launch.conditions.IfCondition(fast_lio_rviz_required)
    ))
    
    return nodes

def generate_launch_description():
    """Generate the launch description with all components."""

    param_defaults = load_yaml_parameters()
    launch_args = declare_launch_arguments(param_defaults)
    nodes = create_nodes()
    launch_description = launch.LaunchDescription(launch_args + nodes)
    
    return launch_description