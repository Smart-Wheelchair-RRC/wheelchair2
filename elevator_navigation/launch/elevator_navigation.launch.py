import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('elevator_navigation')
    default_config_file = os.path.join(pkg_share, 'config', 'campus_config.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the elevator navigation YAML configuration file'
    )

    config_file_config = LaunchConfiguration('config_file')

    state_manager_node = Node(
        package='elevator_navigation',
        executable='elevator_nav_node',
        name='nav_state_manager',
        output='screen',
        parameters=[{
            'yaml_config_path': config_file_config
        }]
    )

    elevator_sensor_node = Node(
        package='elevator_navigation',
        executable='elevator_floor_detection_manual',
        name='elevator_floor_detection_manual',
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        state_manager_node,
        elevator_sensor_node
    ])