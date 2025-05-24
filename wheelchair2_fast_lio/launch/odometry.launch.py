"""
Launch file for the wheelchair2 fast_lio odometry package.
- Inverts the LiDAR data if required.
- Publishes a static transform from `map` to `camera_init` if required.
- Launches the fast_lio node with a specified configuration file.
"""

from pathlib import Path

import launch_ros
from ament_index_python.packages import get_package_share_directory

import launch

def generate_launch_description():

    # create launch configurations
    lidar_inverted = launch.substitutions.LaunchConfiguration('lidar_inverted')
    init_fram_tf_required = launch.substitutions.LaunchConfiguration('init_frame_tf_required')
    fast_lio_config_file = launch.substitutions.LaunchConfiguration('fast_lio_config')
    fast_lio_config_path = launch.substitutions.LaunchConfiguration('fast_lio_config_path')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    fast_lio_rviz_required = launch.substitutions.LaunchConfiguration('fast_lio_rviz_required')
    fast_lio_rviz_config_file = launch.substitutions.LaunchConfiguration('fast_lio_rviz_config_file')

    # declare launch arguments
    lidar_inverted_arg = launch.actions.DeclareLaunchArgument(
        'lidar_inverted',
        default_value='true',
        description='Whether to invert the lidar data'
    )

    init_frame_tf_required_arg = launch.actions.DeclareLaunchArgument(
        'init_frame_tf_required',
        default_value='true',
        description='Whether to publish tf from map to camera_init'
    )

    fast_lio_config_file_arg = launch.actions.DeclareLaunchArgument(
        'fast_lio_config',
        default_value='fast_lio_mid360.yaml',
        description='Path to the fast_lio config file'
    )

    fast_lio_config_path_arg = launch.actions.DeclareLaunchArgument(
        'fast_lio_config_path',
        default_value=str(Path(get_package_share_directory('wheelchair2_fast_lio'), 'config')),
        description='Path to the fast_lio config directory'
    )

    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    fast_lio_rviz_required_arg = launch.actions.DeclareLaunchArgument(
        'fast_lio_rviz_required',
        default_value='false',
        description='Whether to launch RViz with fast_lio'
    )

    declare_rviz_config_path_cmd = launch.actions.DeclareLaunchArgument(
        'fast_lio_rviz_config_file', default_value=str(Path(get_package_share_directory('fast_lio'), 'rviz', 'fastlio.rviz')),
        description='RViz config file path'
    )

    # create nodes
    invert_livox_data_node = launch_ros.actions.Node(
        package='wheelchair2_fast_lio',
        executable='invert_livox_data',
        name='invert_livox_data',
        output='screen',
        condition=launch.conditions.IfCondition(lidar_inverted)
    )

    tf_fast_lio_init_frame_node = launch_ros.actions.Node(
        package='wheelchair2_fast_lio',
        executable='tf_fast_lio_init_frame',
        name='tf_fast_lio_init_frame',
        output='screen',
        condition=launch.conditions.IfCondition(init_fram_tf_required)
    )

    fast_lio_node = launch_ros.actions.Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            launch.substitutions.PathJoinSubstitution([fast_lio_config_path, fast_lio_config_file]),
            {'use_sim_time': use_sim_time,}
        ]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', fast_lio_rviz_config_file],
        condition=launch.conditions.IfCondition(fast_lio_rviz_required)
    )

    # create launch description
    return launch.LaunchDescription([
        lidar_inverted_arg,
        init_frame_tf_required_arg,
        fast_lio_config_file_arg,
        fast_lio_config_path_arg,
        use_sim_time_arg,
        fast_lio_rviz_required_arg,
        declare_rviz_config_path_cmd,
        invert_livox_data_node,
        tf_fast_lio_init_frame_node,
        fast_lio_node,
        rviz_node
    ])