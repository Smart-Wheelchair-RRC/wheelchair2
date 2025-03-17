import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    namespace = LaunchConfiguration('namespace', default="")

    keepout_mask_yaml = LaunchConfiguration(
        "keepout_mask_yaml",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab_mask.yaml",
        ),
    )

    # Keepout filter parameters
    keepout_params_file = LaunchConfiguration(
        "keepout_params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "keepout_params.yaml",
        ),
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': keepout_mask_yaml}

    configured_params = RewrittenYaml(
        source_file=keepout_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", 
                                  default_value="false", 
                                  description="Use simulation (Gazebo) clock if true"),
            DeclareLaunchArgument("keepout_mask", 
                                  default_value=keepout_mask_yaml, 
                                  description="Path to keepout mask file"),
            DeclareLaunchArgument("keepout_params_file", 
                                  default_value=keepout_params_file, 
                                  description="Path for keepout filter parameters"),
            
            # Start filter nodes
            filter_mask_server,
            costmap_filter_info_server,
            lifecycle_manager_costmap_filters,
        ]
    )
