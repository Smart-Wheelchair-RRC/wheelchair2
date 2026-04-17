import os
import time

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Generate timestamp for dynamic rosbag folder naming
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    bag_output_dir = f"/home/container_user/bags/mapping_{timestamp}"

    # 2. Get package directories
    bringup_dir = get_package_share_directory('wheelchair2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # 3. Define Actions
    
    # ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'livox.launch.py')
        ),
        launch_arguments={'ramp_nav': 'True'}.items()
    )

    # ros2 run wheelchair2_bringup crop_lidar
    crop_lidar_node = Node(
        package='wheelchair2_bringup',
        executable='crop_lidar.py',
        output='screen'
    )

    # ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=blacky use_sim_time:=False
    wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'wheelchair2.launch.py')
        ),
        launch_arguments={
            'robot_name': 'blacky',
            'use_sim_time': 'False'
        }.items()
    )

    # ros2 launch fast_lio mapping.launch.py
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        )
    )

    # rosbag2 record /livox/imu /livox/lidar_box_cropped /odom /tf /tf_static
    # Note: Added /tf_static alongside /tf as it is typically required for complete transform trees in ROS2
    record_rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_output_dir,
            '/livox/imu',
            '/livox/lidar_box_cropped',
            '/odom',
            '/tf',
            '/tf_static'
        ],
        output='screen'
    )

    # 4. Return the complete LaunchDescription
    return LaunchDescription([
        livox_launch,
        crop_lidar_node,
        wheelchair_launch,
        fast_lio_launch,
        record_rosbag
    ])