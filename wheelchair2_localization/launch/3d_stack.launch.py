import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    loc_dir = get_package_share_directory("wheelchair2_localization")

    # Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    run_rviz = LaunchConfiguration('run_rviz')

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"),
        
        # --- Globally Inherited Arguments ---
        # These do not need to be explicitly passed to IncludeLaunchDescription.
        # As long as they are provided via CLI, the child launch files will pick them up.
        DeclareLaunchArgument('pcd_map_path', description='Mandatory: Absolute path to the pointcloud map file (.pcd)'),
        DeclareLaunchArgument('nature', description='Mandatory: set to "indoor" or "outdoor"'),

        # Odometry (DLIO)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('direct_lidar_inertial_odometry'), 'launch', 'dlio.launch.py')
            ),
            launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()
        ),

        # Localization (3D-BBS)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheelchair2_3dbbs'), 'launch', '3dbbs.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'nature': LaunchConfiguration('nature'),             # Explicitly pass
                'pcd_map_path': LaunchConfiguration('pcd_map_path')  # Explicitly pass
            }.items()
        ),

        # Localization (ICP)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('icp_localization_ros2'), 'launch', 'bringup.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'period': '2.0',
                'pcd_map_path': LaunchConfiguration('pcd_map_path')  # Explicitly pass
            }.items()
        ),
    ])