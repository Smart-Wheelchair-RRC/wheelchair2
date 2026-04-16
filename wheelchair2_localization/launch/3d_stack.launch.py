import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Directories
    loc_dir = get_package_share_directory("wheelchair2_localization")

    # Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_imu = LaunchConfiguration("use_imu")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value="false", 
            description="Use simulation (Gazebo) clock if true"
        ),
        
        DeclareLaunchArgument(
            "use_imu", 
            default_value="false", 
            description="True: Only IMU/EKF mode. False: Only 3D-BBS/ICP/DLIO mode."
        ),
        
        DeclareLaunchArgument(
            'pcd_map_path', 
            default_value='',
            description='Mandatory: Absolute path to the pointcloud map file (.pcd)'
        ),
        
        DeclareLaunchArgument(
            'nature', 
            default_value='indoor',
            description='Mandatory: set to "indoor" or "outdoor"'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_dir, 'launch', 'robot_localization_launch.py')
            ),
            condition=IfCondition(use_imu)
        ),


        # Odometry (DLIO)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('direct_lidar_inertial_odometry'), 'launch', 'dlio.launch.py')
            ),
            condition=UnlessCondition(use_imu), 
            launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()
        ),

        # Localization (3D-BBS)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheelchair2_3dbbs'), 'launch', '3dbbs.launch.py')
            ),
            condition=UnlessCondition(use_imu),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
                'nature': LaunchConfiguration('nature'),
                'pcd_map_path': LaunchConfiguration('pcd_map_path')
            }.items()
        ),

        # Localization (ICP)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('icp_localization_ros2'), 'launch', 'bringup.launch.py')
            ),
            condition=UnlessCondition(use_imu),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'period': '2.0',
                'pcd_map_path': LaunchConfiguration('pcd_map_path')
            }.items()
        ),
    ])