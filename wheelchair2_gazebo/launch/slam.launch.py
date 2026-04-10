import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_wheelchair_gazebo = get_package_share_directory('wheelchair2_gazebo')
    
    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Start your existing Simulation 
    # (MAKE SURE wheelchair2_controllers.yaml has publish_odom_tf: true)
    start_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheelchair_gazebo, 'launch', 'wheelchair2_sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Start SLAM Toolbox with specific parameter overrides
    # We use the Node action directly instead of the included launch 
    # to ensure no namespace conflicts and correct frame mapping.
    start_slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping',
                # Add these lines to fix the "dropping messages" error:
                'transform_timeout': 0.1,
                'minimum_time_interval': 0.1,
                'lookup_transform_timeout': 0.2,
                'transform_publish_period': 0.02,
                # Force Best Effort QoS
                'scan_queue_size': 10,
            }
        ]
    )

    # 3. Start RViz 
    # If nav2_bringup isn't installed, you can just use 'rviz2' without the -d arg
    try:
        rviz_config_dir = os.path.join(
            get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
        rviz_args = ['-d', rviz_config_dir]
    except:
        rviz_args = []

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        start_simulation,
        start_slam_toolbox,
        start_rviz
    ])