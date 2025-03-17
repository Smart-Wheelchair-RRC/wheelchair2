import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Map and Nav2 config files
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

    # Keepout Filter Parameters
    keepout_params_file = LaunchConfiguration(
        "keepout_params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "keepout_params.yaml",
        ),
    )

    keepout_mask_yaml = LaunchConfiguration(
        "keepout_mask",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab_mask.yaml",
        ),
    )

    # Keepout Filter Nodes
    filter_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="filter_mask_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": keepout_mask_yaml}],
    )

    costmap_filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": keepout_mask_yaml}],
    )

    lifecycle_manager_costmap_filters = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["filter_mask_server", "costmap_filter_info_server"],
        }],
    )

    # Delay Nav2 startup to ensure filters are ready
    nav2 = TimerAction(
        period=10.0,  # 5-second delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_parameters_file,
                    "keepout_params_file": keepout_params_file,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"),
            DeclareLaunchArgument("map", default_value=map_yaml, description="Full path to map file to load"),
            DeclareLaunchArgument("params_file", default_value=nav2_parameters_file, description="Path for Nav2 config file"),
            DeclareLaunchArgument("keepout_params_file", default_value=keepout_params_file, description="Path for keepout filter parameters"),
            DeclareLaunchArgument("keepout_mask", default_value=keepout_mask_yaml, description="Path to keepout mask file"),
            
            # Start keepout filters first
            filter_mask_server,
            costmap_filter_info_server,
            lifecycle_manager_costmap_filters,

            # Start Nav2 after delay
            nav2,
        ]
    )
