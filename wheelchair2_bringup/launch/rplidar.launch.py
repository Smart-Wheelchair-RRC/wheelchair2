import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    config_path = get_package_share_directory('wheelchair2_bringup')
    s2_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar_s2_launch.py'
    )
    box_filter_config = os.path.join(config_path, 'config', 'box_filter.yaml')
    box_filter_config_1 = os.path.join(config_path, 'config', 'box_filter_1.yaml')

    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src='/scan', dst='/scan_rp'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(s2_launch))
            ]
        ),
        Node(
            package='wheelchair2_bringup',
            executable='box_filter.py',
            name='box_filter',
            output='screen'
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[box_filter_config, {'use_sim_time': False}],
            remappings=[
                ('scan', 'scan_rp'),       
                ('scan_filtered', 'scan_rp_filtered_1') 
            ],
            output='screen'
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_1',
            parameters=[box_filter_config_1, {'use_sim_time': False}],
            remappings=[
                ('scan', 'scan_rp_filtered_1'),       
                ('scan_filtered', 'scan_rp_filtered')    
            ],
            output='screen'
        )
    ])