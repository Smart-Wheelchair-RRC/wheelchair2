from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch
import os

ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="",
                          description="Robot namespace"),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start rviz.",
    ),
    DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("wheelchair2_gazebo"),
            "worlds",
            "empty_world.world",
        ),
        description="Ignition World",
    ),
    DeclareLaunchArgument(
        "x", default_value="0.0", description="x component of the robot pose."
    ),
    DeclareLaunchArgument(
        "y", default_value="0.0", description="y component of the robot pose."
    ),
    DeclareLaunchArgument(
        "z", default_value="0.0", description="z component of the robot pose."
    ),
    DeclareLaunchArgument(
        "yaw", default_value="0.0", description="yaw component of the robot pose."
    ),
]


def generate_launch_description():
    pkg_wheelchair2_simulations = get_package_share_directory(
        "wheelchair2_simulation")

    gz_sim_launch = PathJoinSubstitution(
        [pkg_wheelchair2_simulations, "launch", "gz_sim.launch.py"]
    )
    wheelchair2_spawn_launch = PathJoinSubstitution(
        [pkg_wheelchair2_simulations, "launch", "wheelchair2_spawn.launch.py"]
    )

    # Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ("world", LaunchConfiguration("world"))
        ],
    )

    spawn_wheelchair = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([wheelchair2_spawn_launch]),
        launch_arguments=[
            ("rviz", LaunchConfiguration("rviz")),
            ("x", LaunchConfiguration("x")),
            ("y", LaunchConfiguration("y")),
            ("z", LaunchConfiguration("z")),
            ("yaw", LaunchConfiguration("yaw")),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    ld.add_action(spawn_wheelchair)
    return ld
