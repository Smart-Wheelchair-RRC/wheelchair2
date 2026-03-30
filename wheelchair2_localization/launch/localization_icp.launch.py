"""
Launch file for the wheelchair2 localization package.
- Loads parameters from CLI if available, else uses defaults.
- Broadcasts the PCD map if required.
- Inverts the LiDAR data if required.
- Runs ICP localization node.
"""

from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SELF_PACKAGE_NAME = 'wheelchair2_localization'

NAVIGATION_PACKAGE_NAME = 'wheelchair2_navigation'
NAVIGATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(NAVIGATION_PACKAGE_NAME))

ICP_LOCALIZATION_PACKAGE_NAME = 'icp_localization_ros2'
ICP_LOCALIZATION_PACKAGE_DIRECTORY = Path(get_package_share_directory(ICP_LOCALIZATION_PACKAGE_NAME))
ICP_LOCALIZATION_CONFIG_PATH = ICP_LOCALIZATION_PACKAGE_DIRECTORY / 'config'

DEFAULT_PARAMETERS = {
    # core functionality parameters
    'lidar_inverted': 'true',
    'use_sim_time': 'false',
    'publish_pcd_map': 'false',

    # Topics
    'input_lidar_topic': '/livox/lidar',
    'inverted_lidar_topic': '/livox/lidar_inverted',
    'imu_topic': '/livox/imu',
    'inverted_imu_topic': '/livox/imu_inverted',
    'final_lidar_topic': '/livox/lidar_inverted',
    'final_imu_topic': '/livox/imu_inverted',
    'input_odom_topic': '/Odometry',

    # File paths
    # 'pcd_map_path': str(NAVIGATION_PACKAGE_DIRECTORY / 'maps' / 'rrc' / 'lab_map_with_outside_corridor.pcd'),
    'pcd_map_path': '/home/container_user/wheelchair2/src/lab_map_with_outside_corridor.pcd',
    'icp_config_file': str(ICP_LOCALIZATION_CONFIG_PATH / 'icp.yaml'),
    'icp_node_params_file': str(ICP_LOCALIZATION_CONFIG_PATH / 'node_params.yaml'),
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
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_pcd_map = LaunchConfiguration('publish_pcd_map')

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
    icp_config_file = LaunchConfiguration('icp_config_file')
    icp_node_params_file = LaunchConfiguration('icp_node_params_file')

    # Create nodes
    nodes = []

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
            ('/livox/lidar_inverted', final_lidar_topic),
            ('/livox/imu_inverted', final_imu_topic),
            ('/Odometry', input_odom_topic),
        ]
    ))

    return nodes

def generate_launch_description():
    """Generate the launch description with all components."""

    launch_args = declare_launch_arguments(DEFAULT_PARAMETERS)
    nodes = create_nodes()
    launch_description = launch.LaunchDescription(launch_args + nodes)
    
    return launch_description