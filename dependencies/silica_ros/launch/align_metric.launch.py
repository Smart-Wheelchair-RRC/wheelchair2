import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. RealSense Camera Node ---
    rs_pkg_dir = get_package_share_directory('realsense2_camera')
    rs_launch_file = os.path.join(rs_pkg_dir, 'launch', 'rs_launch.py')
    
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_file),
        launch_arguments={
            'camera_name': 'camera1',
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    # --- 2. Silica Glass Reconstructor Node ---
    silica_pkg_dir = get_package_share_directory('silica_ros')
    config_file = os.path.join(silica_pkg_dir, 'config', 'align_metric.yaml')

    silica_node = Node(
        package='silica_ros',
        executable='align_metric',  # This assumes your setup.py entry point is named 'align_metric'
        name='glass_reconstructor_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        realsense_node,
        silica_node
    ])