import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab.yaml",
        ),
    )
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "nav2_params.yaml",
        ),
    )
    # rviz_config_path = os.path.join(("rviz"), 'voxel.rviz')
    # nav2_rviz = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         arguments=['--display-config', rviz_config_path]
    #     )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nav2_bringup"), "launch"),
                "/bringup_launch.py",
            ]
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
    )
    
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
            nav2,
            # nav2_rviz,
        ]
    )