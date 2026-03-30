from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
              'frame_id':'base_link',
              'subscribe_depth':True,
              'subscribe_rgb':True,
              'subscribe_scan':True,
              'approx_sync':True,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
              'sync_queue_size':10,
              
              # RTAB-Map's internal parameters are strings:
              'RGBD/NeighborLinkRefining': 'true', #Correct odometry using the input lidar topic using ICP
              'RGBD/ProximityBySpace': 'true', 
              'RGBD/AngularUpdate': '0.01',
              'RGBD/LinearUpdate': '0.01',
              'RGBD/OptimizeFromGraphEnd': 'false',
              'Optimizer/Slam2D': 'true',
              'Reg/Strategy': '1',
              'Reg/Force3DoF': 'true',
              'Vis/MinInliers': '7',
              'Rtabmap/TimeThr': '700',
              'Mem/RehearsalSimilarity': '0.45'
            }
    remappings=[
              ('rgb/image', '/camera/camera/color/image_raw'),
              ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
              ('rgb/camera_info', '/camera/camera/color/camera_info'),
              ('scan', '/scan'),
              ('odom', '/odom'),
              ('grid_map', 'map')
            ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
         
        # Nodes to launch

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=[
              '-d' # This will delete the previous database (~/.ros/rtabmap.db)
            ]), 

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),
     
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':'base_link',
              'odom_frame_id':'odom',
              'approx_sync':True,
              'use_sim_time':use_sim_time,
            }],
            remappings=remappings
            ),
    ])
    
    