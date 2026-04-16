import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    s2_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar_s2_launch.py'
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src='/scan', dst='/scan_rp'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(s2_launch))
            ]
        )
    ])