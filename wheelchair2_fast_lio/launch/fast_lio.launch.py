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
                if 'lidar' in yaml_params and 'inverted' in yaml_params['lidar']:
                    param_defaults['lidar_inverted'] = str(yaml_params['lidar']['inverted']).lower()
                
                if 'tf' in yaml_params and 'init_frame_required' in yaml_params['tf']:
                    param_defaults['init_frame_tf_required'] = str(yaml_params['tf']['init_frame_required']).lower()
                
                if 'fast_lio' in yaml_params:
                    if 'config_file' in yaml_params['fast_lio']:
                        param_defaults['fast_lio_config'] = yaml_params['fast_lio']['config_file']
                    if 'config_path' in yaml_params['fast_lio'] and yaml_params['fast_lio']['config_path']:
                        param_defaults['fast_lio_config_path'] = yaml_params['fast_lio']['config_path']
                    if 'rviz' in yaml_params['fast_lio']:
                        if 'required' in yaml_params['fast_lio']['rviz']:
                            param_defaults['fast_lio_rviz_required'] = str(yaml_params['fast_lio']['rviz']['required']).lower()
                        if 'config_file' in yaml_params['fast_lio']['rviz'] and yaml_params['fast_lio']['rviz']['config_file']:
                            param_defaults['fast_lio_rviz_config_file'] = yaml_params['fast_lio']['rviz']['config_file']
                
                if 'use_sim_time' in yaml_params:
                    param_defaults['use_sim_time'] = str(yaml_params['use_sim_time']).lower()
                
                if 'localization' in yaml_params:
                    if 'enabled' in yaml_params['localization']:
                        param_defaults['use_localization'] = str(yaml_params['localization']['enabled']).lower()
                    if 'pcd_map_path' in yaml_params['localization']:
                        param_defaults['pcd_map_path'] = yaml_params['localization']['pcd_map_path']
                
                if 'topics' in yaml_params:
                    if 'lidar_cloud' in yaml_params['topics']:
                        param_defaults['lidar_cloud_topic'] = yaml_params['topics']['lidar_cloud']
                    if 'fast_lio_odom' in yaml_params['topics']:
                        param_defaults['fast_lio_odom_topic'] = yaml_params['topics']['fast_lio_odom']
                    if 'initialpose' in yaml_params['topics']:
                        param_defaults['initialpose_topic'] = yaml_params['topics']['initialpose']
                    if 'localization_pose' in yaml_params['topics']:
                        param_defaults['localization_pose_topic'] = yaml_params['topics']['localization_pose']
    
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
        description='Path to the PCD map directory'
    ))
    
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
    
    # Localization node
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
            {'localization_pose_topic': localization_pose_topic}
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