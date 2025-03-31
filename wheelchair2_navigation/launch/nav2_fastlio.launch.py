import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction, GroupAction,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fastlio_localization_dir = get_package_share_directory('fast_lio_localization')
    fastlio_localization_launch = os.path.join(fastlio_localization_dir, 'launch', 'localization.launch.py')
    fastlio_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_localization_launch),
        launch_arguments={"pcd_map_topic": "/cloud_pcd",
                          "map": os.path.join(fastlio_localization_dir, "PCD", "lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd"),
                          "rviz":"false",
            }.items(),
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "lab_map_with_outside_corridor.yaml",
        ),
    )
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "nav2_params_fastlio.yaml",
        ),
    )
    rviz_config_path = os.path.join(get_package_share_directory("wheelchair2_navigation"), "rviz", 'navigation_fastlio.rviz')
    nav2_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )
    
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("wheelchair2_navigation"), "launch"),
                "/map_server_fastlio.launch.py",
            ]
        ),
    )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nav2_bringup"), "launch"),
                "/navigation_launch.py",
            ]
        ),
        launch_arguments={
            # "use_localization": "false",
            # "slam": "false",
            # "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
    )
    
    livox_custom_msg_to_pcd = Node(
        package="wheelchair2_laser",
        executable="livox_custom_msg_to_pointcloud2.py",
        name="livox_custom_msg_to_pointcloud2",
        output="screen",
    )
    
    tf_node1 = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_body_to_base_footprint", output="screen",
                    arguments=["--frame-id" , "body",
                               "--child-frame-id", "base_footprint",
                               "--z", "-0.9"], )
    
    tf_node2 = Node(package="tf2_ros", executable="static_transform_publisher", name="tf_body_to_base_footprint", output="screen",
                    arguments=["--frame-id" , "map",
                               "--child-frame-id", "map_",
                               "--z", "-0.9"], )
    
    group_Action_cmd = GroupAction([map_server, nav2])
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=map_yaml,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_parameters_file,
                description="Path for nav2 config file",
            ),
            nav2_rviz,
            livox_custom_msg_to_pcd,
            fastlio_localization,
            tf_node1,
            tf_node2,
            TimerAction(period=15.0, actions=[group_Action_cmd]),
            # TimerAction(period=20.0, actions=[nav2]),
        ]
    )