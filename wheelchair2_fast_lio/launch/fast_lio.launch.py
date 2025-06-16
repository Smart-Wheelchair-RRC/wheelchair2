"""
Launch file for the wheelchair2 fast_lio odometry package.
- Inverts the LiDAR data if required.
- Publishes a static transform from `map` to `camera_init` if required.
- Launches the fast_lio node with a specified configuration file.
- Loads parameters from YAML file if available.

Glossary
- `flodom`: FAST-LIO odometry frame, this is where FAST-LIO thinks the robot is with respect to the scanned point cloud.
- `fll`: FAST-LIO localization frame, this is where the localization node thinks the scanned point cloud is with respect to the known map.
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

SELF_PACKAGE_DIRECTORY = Path(get_package_share_directory('wheelchair2_fast_lio'))
SELF_CONFIG_PATH = SELF_PACKAGE_DIRECTORY / 'config'
SELF_YAML_CONFIG = SELF_CONFIG_PATH / 'fast_lio_package_params.yaml'

def load_yaml_parameters():
    """Load parameters from YAML file if available."""
    
    # Default parameter values
    yaml_parameters = {
        # core functionality parameters
        'use_sim_time': 'true',
        'lidar_inverted': 'true',
        'use_localization': 'true',
        'navigation_mode': 'true',

        # frames and TFs
        'initial_flodom_id': 'camera_init',
        'global_frame_id': 'map',
        'flodom_id': 'body',
        'lidar_mount_frame_id': 'livox_frame',
        'base_frame_id': 'base_footprint',

        # TFs enabled
        'global_frame_to_fll_tf': 'true',
        'flodom_to_lidar_mount_tf': 'true',

        # Topics
        'input_lidar_topic': '/livox/lidar',
        'inverted_lidar_topic': '/livox/lidar_inverted',
        'imu_topic': '/livox/imu',
        'inverted_imu_topic': '/livox/imu_inverted',
        'flodom_topic': '/Odometry',
        'localized_post_topic': '/localization_pose',
        'initial_pose_topic': '/initialpose',
        'pcd_3d_map_topic': '/pcd_3d_map',

        # Localization parameters
        'use_map_cropping': 'true',
        'map_crop_distance': '20.0',  # Distance in meters for map cropping
        'icp_threshold': '0.8',  # ICP convergence threshold
        'icp_max_iterations': '50',  # Maximum number of ICP iterations
        'localization_publish_frequency': '30.0',  # Frequency for publishing localization poses
        'publish_localized_pose': 'true',  # Whether to publish the localized pose

        # File paths
        'pcd_map_path': 'you-super-accurate-map.pcd',
        'fast_lio_mapping_config': str(SELF_CONFIG_PATH / 'fast_lio_mapping_mid360.yaml'),
        'fast_lio_localization_config': str(SELF_CONFIG_PATH / 'fast_lio_package_params.yaml'),
        'fast_lio_rviz_config_file': str(Path(get_package_share_directory('fast_lio')) / 'rviz' / 'fastlio.rviz'),
        'nav2_params_file': str(SELF_CONFIG_PATH / 'nav2_fastlio_params.yaml'),
        'map_file_path': str(Path(get_package_share_directory('wheelchair2_navigation')) / 'maps' / 'rrc' / 'rrc_lab.yaml'),
        'nav2_rviz_config_file': str(Path(get_package_share_directory('wheelchair2_navigation')) / 'rviz' / 'navigation.rviz'),

        # RViz visualization
        'fast_lio_rviz_required': 'false',  # Whether to launch RViz for Fast-LIO visualization
        'nav2_rviz_required': 'false',  # Whether to launch RViz for Nav2 visualization
        'publish_3d_pcd_map': 'fals',  # Whether to publish the 3D point cloud map
    }

    # Try to load parameters from YAML file if available
    if SELF_YAML_CONFIG.exists():
        with open(SELF_YAML_CONFIG, 'r') as f:
            yaml_params_from_config = yaml.safe_load(f)
            
            # Convert YAML parameters to flat dictionary with default string values
            if yaml_params_from_config:

                if 'core' in yaml_params_from_config:
                    core_params = yaml_params_from_config['core']
                    if 'use_sim_time' in core_params:
                        yaml_parameters['use_sim_time'] = str(core_params['use_sim_time']).lower()

                    if 'lidar_inverted' in core_params:
                        yaml_parameters['lidar_inverted'] = str(core_params['lidar_inverted']).lower()

                    if 'use_localization' in core_params:
                        yaml_parameters['use_localization'] = str(core_params['use_localization']).lower()

                    if 'navigation_mode' in core_params:
                        yaml_parameters['navigation_mode'] = str(core_params['navigation_mode']).lower()

                if 'frames' in yaml_params_from_config:
                    frames_params = yaml_params_from_config['frames']
                    
                    if 'initial_flodom_id' in frames_params:
                        yaml_parameters['initial_lidar_frame_id'] = frames_params['initial_flodom_id']
                    
                    if 'global_frame_id' in frames_params:
                        yaml_parameters['global_frame_id'] = frames_params['global_frame_id']
                    
                    if 'flodom_id' in frames_params:
                        yaml_parameters['live_lidar_frame_id'] = frames_params['flodom_id']
                    
                    if 'lidar_mount_frame_id' in frames_params:
                        yaml_parameters['lidar_mounting_frame_id'] = frames_params['lidar_mount_frame_id']

                    if 'base_frame_id' in frames_params:
                        yaml_parameters['base_frame_id'] = frames_params['base_frame_id']

                if 'tfs' in yaml_params_from_config:
                    tfs_params = yaml_params_from_config['tfs']
                    
                    if 'global_frame_to_fll_tf' in tfs_params:
                        yaml_parameters['global_frame_to_fll_tf'] = str(tfs_params['global_frame_to_fll_tf']).lower()
                    
                    if 'flodom_to_lidar_mount_tf' in tfs_params:
                        yaml_parameters['flodom_to_lidar_mount_tf'] = str(tfs_params['flodom_to_lidar_mount_tf']).lower()

                
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
                    
                    if 'flodom_topic' in topics_params:
                        yaml_parameters['fast_lio_odom_topic'] = topics_params['flodom_topic']
                    
                    if 'localized_post_topic' in topics_params:
                        yaml_parameters['localization_pose_topic'] = topics_params['localized_post_topic']
                    
                    if 'initial_pose_topic' in topics_params:
                        yaml_parameters['initialpose_topic'] = topics_params['initial_pose_topic']
                    
                    if 'pcd_3d_map_topic' in topics_params:
                        yaml_parameters['pcd_3d_map_topic'] = topics_params['pcd_3d_map_topic']

                if 'localization' in yaml_params_from_config:
                    localization_params = yaml_params_from_config['localization']
                    
                    if 'use_map_cropping' in localization_params:
                        yaml_parameters['use_map_cropping'] = str(localization_params['use_map_cropping']).lower()
                    
                    if 'map_crop_distance' in localization_params:
                        yaml_parameters['map_crop_distance'] = str(localization_params['map_crop_distance'])
                    
                    if 'icp_threshold' in localization_params:
                        yaml_parameters['icp_threshold'] = str(localization_params['icp_threshold'])
                    
                    if 'icp_max_iterations' in localization_params:
                        yaml_parameters['icp_max_iterations'] = str(localization_params['icp_max_iterations'])
                    
                    if 'localization_publish_frequency' in localization_params:
                        yaml_parameters['localization_publish_frequency'] = str(localization_params['localization_publish_frequency'])

                    if 'publish_localized_pose' in localization_params:
                        yaml_parameters['publish_localized_pose'] = str(localization_params['publish_localized_pose']).lower()

                if 'files' in yaml_params_from_config:
                    files_params = yaml_params_from_config['files']
                    
                    if 'pcd_map_path' in files_params and files_params['pcd_map_path'].strip():
                        yaml_parameters['pcd_map_path'] = files_params['pcd_map_path']
                    
                    if 'fast_lio_mapping_config' in files_params and files_params['fast_lio_mapping_config'].strip():
                        yaml_parameters['fast_lio_config'] = files_params['fast_lio_mapping_config']
                    
                    if 'fast_lio_rviz_config_file' in files_params and files_params['fast_lio_rviz_config_file'].strip():
                        yaml_parameters['fast_lio_rviz_config_file'] = files_params['fast_lio_rviz_config_file']
                    
                    if 'nav2_params_file' in files_params and files_params['nav2_params_file'].strip():
                        yaml_parameters['nav2_params_file'] = files_params['nav2_params_file']
                    
                    if 'map_file_path' in files_params and files_params['map_file_path'].strip():
                        yaml_parameters['map_file_path'] = files_params['map_file_path']
                    
                    if 'nav2_rviz_config_file' in files_params and files_params['nav2_rviz_config_file'].strip():
                        yaml_parameters['nav2_rviz_config_file'] = files_params['nav2_rviz_config_file']

                if 'rviz' in yaml_params_from_config:
                    rviz_params = yaml_params_from_config['rviz']
                    
                    if 'fast_lio_rviz_required' in rviz_params:
                        yaml_parameters['fast_lio_rviz_required'] = str(rviz_params['fast_lio_rviz_required']).lower()
                    
                    if 'nav2_rviz_required' in rviz_params:
                        yaml_parameters['nav2_rviz_required'] = str(rviz_params['nav2_rviz_required']).lower()

                    if 'publish_3d_pcd_map' in rviz_params:
                        yaml_parameters['publish_3d_pcd_map'] = str(rviz_params['publish_3d_pcd_map']).lower()
    
    return yaml_parameters

def declare_launch_arguments(param_defaults):
    """Declare all launch arguments with values from parameters."""
    launch_args = []
    
    # Declare core functionality parameters
    launch_args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value=param_defaults['use_sim_time'],
        description='Use simulation time'
    ))

    launch_args.append(DeclareLaunchArgument(
        'lidar_inverted',
        default_value=param_defaults['lidar_inverted'],
        description='Whether to invert LiDAR data'
    ))

    launch_args.append(DeclareLaunchArgument(
        'use_localization',
        default_value=param_defaults['use_localization'],
        description='Whether to use localization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'navigation_mode',
        default_value=param_defaults['navigation_mode'],
        description='Whether to enable navigation mode'
    ))

    # Declare frame and TF parameters
    launch_args.append(DeclareLaunchArgument(
        'initial_lidar_frame_id',
        default_value=param_defaults['initial_flodom_id'],
        description='Initial LiDAR frame ID'
    ))

    launch_args.append(DeclareLaunchArgument(
        'global_frame_id',
        default_value=param_defaults['global_frame_id'],
        description='Global frame ID'
    ))

    launch_args.append(DeclareLaunchArgument(
        'live_lidar_frame_id',
        default_value=param_defaults['flodom_id'],
        description='Live LiDAR frame ID'
    ))

    launch_args.append(DeclareLaunchArgument(
        'lidar_mount_frame_id',
        default_value=param_defaults['lidar_mount_frame_id'],
        description='LiDAR mounting frame ID'
    ))

    launch_args.append(DeclareLaunchArgument(
        'base_frame_id',
        default_value=param_defaults['base_frame_id'],
        description='Base frame ID for the robot'
    ))

    # Declare TFs enabled
    launch_args.append(DeclareLaunchArgument(
        'global_frame_to_fll_tf',
        default_value='true',
        description='Whether to publish static transform from global frame to Fast-LIO frame'
    ))

    launch_args.append(DeclareLaunchArgument(
        'flodom_to_lidar_mount_tf',
        default_value='true',
        description='Whether to publish static transform from Fast-LIO frame to LiDAR mounting frame'
    ))

    # Declare topic parameters
    launch_args.append(DeclareLaunchArgument(
        'lidar_cloud_topic',
        default_value=param_defaults['input_lidar_topic'],
        description='Input LiDAR cloud topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'inverted_lidar_topic',
        default_value=param_defaults['inverted_lidar_topic'],
        description='Inverted LiDAR cloud topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'imu_topic',
        default_value=param_defaults['imu_topic'],
        description='Input IMU topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'inverted_imu_topic',
        default_value=param_defaults['inverted_imu_topic'],
        description='Inverted IMU topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'fast_lio_odom_topic',
        default_value=param_defaults['flodom_topic'],
        description='Fast-LIO odometry topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'localization_pose_topic',
        default_value=param_defaults['localized_post_topic'],
        description='Localization pose topic'
    ))

    launch_args.append(DeclareLaunchArgument(
        'initialpose_topic',
        default_value=param_defaults['initial_pose_topic'],
        description='Initial pose topic for localization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'pcd_3d_map_topic',
        default_value=param_defaults['pcd_3d_map_topic'],
        description='3D point cloud map topic for localization'
    ))

    # Declare localization parameters
    launch_args.append(DeclareLaunchArgument(
        'use_map_cropping',
        default_value=param_defaults['use_map_cropping'],
        description='Whether to use map cropping in localization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'map_crop_distance',
        default_value=param_defaults['map_crop_distance'],
        description='Distance for map cropping in meters'
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

    launch_args.append(DeclareLaunchArgument(
        'localization_publish_frequency',
        default_value=param_defaults['localization_publish_frequency'],
        description='Frequency for publishing localization poses'
    ))

    launch_args.append(DeclareLaunchArgument(
        'publish_localized_pose',
        default_value=param_defaults['publish_localized_pose'],
        description='Whether to publish the localized pose'
    ))

    # Declare file paths
    launch_args.append(DeclareLaunchArgument(
        'pcd_map_path',
        default_value=param_defaults['pcd_map_path'],
        description='Path to the 3D point cloud map file'
    ))

    launch_args.append(DeclareLaunchArgument(
        'fast_lio_mapping_config',
        default_value=param_defaults['fast_lio_mapping_config'],
        description='Configuration file for Fast-LIO mapping'
    ))

    launch_args.append(DeclareLaunchArgument(
        'fast_lio_localization_config',
        default_value=param_defaults['fast_lio_localization_config'],
        description='Configuration file for Fast-LIO localization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'fast_lio_rviz_config_file',
        default_value=param_defaults['fast_lio_rviz_config_file'],
        description='RViz configuration file for Fast-LIO visualization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'nav2_rviz_config_file',
        default_value=param_defaults['nav2_rviz_config_file'],
        description='RViz configuration file for Nav2 visualization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'nav2_params_file',
        default_value=param_defaults['nav2_params_file'],
        description='Nav2 parameters file for navigation'
    ))

    launch_args.append(DeclareLaunchArgument(
        'map_file_path',
        default_value=param_defaults['map_file_path'],
        description='Path to the map file for Nav2'
    ))

    # Declare RViz visualization parameters
    launch_args.append(DeclareLaunchArgument(
        'fast_lio_rviz_required',
        default_value=param_defaults['fast_lio_rviz_required'],
        description='Whether to launch RViz for Fast-LIO visualization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'nav2_rviz_required',
        default_value=param_defaults['nav2_rviz_required'],
        description='Whether to launch RViz for Nav2 visualization'
    ))

    launch_args.append(DeclareLaunchArgument(
        'publish_3d_pcd_map',
        default_value=param_defaults['publish_3d_pcd_map'],
        description='Whether to publish the 3D point cloud map'
    ))
        

    return launch_args

def create_nodes():
    """Create all the nodes needed for the launch."""

    # Core functionality parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    use_localization = LaunchConfiguration('use_localization')
    navigation_mode = LaunchConfiguration('navigation_mode')

    # Frame and TF parameters
    initial_lidar_frame_id = LaunchConfiguration('initial_lidar_frame_id')
    global_frame_id = LaunchConfiguration('global_frame_id')
    live_lidar_frame_id = LaunchConfiguration('live_lidar_frame_id')
    lidar_mount_frame_id = LaunchConfiguration('lidar_mount_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    # TFs
    global_frame_to_fll_tf = LaunchConfiguration('global_frame_to_fll_tf')
    flodom_to_lidar_mount_tf = LaunchConfiguration('flodom_to_lidar_mount_tf')

    # Topic parameters
    lidar_cloud_topic = LaunchConfiguration('lidar_cloud_topic')
    inverted_lidar_topic = LaunchConfiguration('inverted_lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    inverted_imu_topic = LaunchConfiguration('inverted_imu_topic')
    fast_lio_odom_topic = LaunchConfiguration('fast_lio_odom_topic')
    localization_pose_topic = LaunchConfiguration('localization_pose_topic')
    initialpose_topic = LaunchConfiguration('initialpose_topic')
    pcd_3d_map_topic = LaunchConfiguration('pcd_3d_map_topic')

    # Localization parameters
    use_map_cropping = LaunchConfiguration('use_map_cropping')
    map_crop_distance = LaunchConfiguration('map_crop_distance')
    icp_threshold = LaunchConfiguration('icp_threshold')
    icp_max_iterations = LaunchConfiguration('icp_max_iterations')
    localization_publish_frequency = LaunchConfiguration('localization_publish_frequency')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    fast_lio_mapping_config = LaunchConfiguration('fast_lio_mapping_config')
    fast_lio_localization_config = LaunchConfiguration('fast_lio_localization_config')
    fast_lio_rviz_config_file = LaunchConfiguration('fast_lio_rviz_config_file')
    fast_lio_rviz_required = LaunchConfiguration('fast_lio_rviz_required')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_file_path = LaunchConfiguration('map_file_path')
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file')
    publish_localized_pose = LaunchConfiguration('publish_localized_pose')

    # Visualization parameters
    fast_lio_rviz_required = LaunchConfiguration('fast_lio_rviz_required')
    nav2_rviz_required = LaunchConfiguration('nav2_rviz_required')
    publish_3d_pcd_map = LaunchConfiguration('publish_3d_pcd_map')
    
    # Create nodes
    nodes = []
    group_actions = []
    
    # Invert LiDAR data node
    nodes.append(Node(
        package='wheelchair2_fast_lio',
        executable='invert_livox_data',
        name='invert_livox_data',
        output='screen',
        parameters=[
            {'input_lidar_topic': lidar_cloud_topic},
            {'input_imu_topic': imu_topic},
            {'inverted_lidar_topic': inverted_lidar_topic},
            {'inverted_imu_topic': inverted_imu_topic},
            {'lidar_mount_frame_id': lidar_mount_frame_id},
        ],
        condition=launch.conditions.IfCondition(lidar_inverted)
    ))
    
    # TF for Fast-LIO init frame
    nodes.append(Node(
        package='wheelchair2_fast_lio',
        executable='fast_lio_tfs',
        name='fast_lio_tfs',
        output='screen',
        parameters=[
            {'flodom_frame_id': live_lidar_frame_id}, # Use shared odom_frame_id
            {'lidar_mount_frame_id': lidar_mount_frame_id},
            {'base_frame_id': base_frame_id},
        ],
        condition=launch.conditions.IfCondition(flodom_to_lidar_mount_tf)
    ))
    
    # Fast-LIO node
    nodes.append(Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            fast_lio_mapping_config,
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
            {'publish_3d_map': publish_3d_pcd_map},
            {'pcd_3d_map_topic': pcd_3d_map_topic},
            {'lidar_cloud_topic': lidar_cloud_topic},
            {'fast_lio_odom_topic': fast_lio_odom_topic},
            {'initialpose_topic': initialpose_topic},
            {'localization_pose_topic': localization_pose_topic},
            {'localization_publish_frequency': localization_publish_frequency},
            {'publish_localized_pose': publish_localized_pose},
            {'icp_threshold': icp_threshold},
            {'icp_max_iterations': icp_max_iterations},
            {'use_map_cropping': use_map_cropping},
            {'map_crop_distance': map_crop_distance},
            {'broadcast_tf': global_frame_to_fll_tf},
            {'global_frame': global_frame_id},
            {'fll_frame_id': initial_lidar_frame_id},
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
            'frame_id': global_frame_id
        }],
        # condition=launch.conditions.IfCondition(navigation_mode)
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
        # condition=launch.conditions.IfCondition(navigation_mode)
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