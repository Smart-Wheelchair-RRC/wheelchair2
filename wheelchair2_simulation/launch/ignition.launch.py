import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time", default_value="true", choices=["true", "false"]
    ),
    DeclareLaunchArgument(
        "world", default_value="warehouse", description="Ignition World"
    ),
]


def generate_launch_description():

    # Package Directories
    pkg_wheelchair2_simulation = get_package_share_directory("wheelchair2_simulation")
    pkg_wheelchair2_description = get_package_share_directory("wheelchair2_description")
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.join(pkg_wheelchair2_simulation, "worlds")],
    )

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py"]
    )

    # Launch Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ("ign_args", [LaunchConfiguration("world"), ".sdf", "-r", "-v 4"])
        ],
    )

    # Clock Bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock]"],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)

    return ld
