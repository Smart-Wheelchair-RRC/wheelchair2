import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
 
    # Todo: Added this for pose storage
    map_start_pose = LaunchConfiguration('map_start_pose')

    # Todo: Path to the box filter configuration
    box_filter_config = os.path.join(
        get_package_share_directory("wheelchair2_navigation"),
        "config",
        "box_filter.yaml",
    )
    box_filter_config_1 = os.path.join(
        get_package_share_directory("wheelchair2_navigation"),
        "config",
        "box_filter_2.yaml",
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("wheelchair2_navigation"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Todo: Declare the map_start_pose argument
    declare_map_start_pose_cmd = DeclareLaunchArgument(
        'map_start_pose',
        default_value='[0.0, 0.0, 0.0]',
        description='Start pose for the robot in [x, y, yaw]')

    # Todo: Added the Laser Filter Node
    # This takes raw scan data and removes points hitting the wheelchair frame
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[box_filter_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', 'scan_rp'),                # Input from RPLidar
            ('scan_filtered', 'scan_rp_filtered') # Output for SLAM
        ],
        output='screen'
    )
    laser_filter_node_1 = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_1',
        parameters=[box_filter_config_1],
        remappings=[
            ('scan', 'scan_rp_filtered_1'),        # Input: Raw data from RPLidar
            ('scan_filtered', 'scan_rp_filtered')    # Output: Clean data for Nav2/AMCL
        ],
        output='screen'
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {
            'use_sim_time': use_sim_time,
            'scan_topic': 'scan_rp_filtered',
            'map_start_pose': map_start_pose, # [NEW] Pass the argument to the node
            'map_start_at_dock': False        # [NEW] Force this to False
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
 
    ld = LaunchDescription()
 
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    # Todo : Added start pose node 
    ld.add_action(declare_map_start_pose_cmd) # [NEW] Add action
    # Todo : Added node to launch description
    ld.add_action(laser_filter_node)
    ld.add_action(laser_filter_node_1)
    ld.add_action(start_async_slam_toolbox_node)
 
    return ld