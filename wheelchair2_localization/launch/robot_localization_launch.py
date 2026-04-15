from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch_ros.actions import Node

def generate_launch_description():
    
    ekf_config_path = os.path.join(
        get_package_share_directory("wheelchair2_localization"), 
        'config', 
        'ekf2.yaml'
    )

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/cmd_vel', '/wheelchair2_base_controller/cmd_vel_unstamped')
        ]
    )

    imu_node = Node(
        package='wheelchair2_localization',
        executable='adafruit_imu',
        name='adafruit_imu_node',
        output='screen'
    )
        
    return LaunchDescription([
        ekf_odom,
        imu_node               
    ])