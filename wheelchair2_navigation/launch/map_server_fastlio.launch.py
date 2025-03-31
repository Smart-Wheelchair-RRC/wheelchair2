import os

import launch_ros.actions
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [DeclareLaunchArgument("in_simulation", default_value="false", description="Flag to set use_sim_time")]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    use_sim_time = LaunchConfiguration("in_simulation")

    # map file
    map_file_path = os.path.join(
        get_package_share_directory("wheelchair2_navigation"), "maps", "rrc", "lab_map_with_outside_corridor.yaml"
    )

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file_path, "use_sim_time": use_sim_time, "frame_id": "map_"}],
    )

    lifecycle_nodes = ["map_server"]
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
