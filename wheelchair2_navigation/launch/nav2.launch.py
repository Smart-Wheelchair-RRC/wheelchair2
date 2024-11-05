from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

ARGUMENTS = [
    DeclareLaunchArgument(
        "in_simulation",
        choices=["true", "false"],
        default_value="true",
    ),
    DeclareLaunchArgument(
        "world",
        default_value="hospital",
        description='Gazebo world name'
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory(
                "wheelchair2_navigation"), "config", "nav2_params.yaml"
        ),
        description='Path for nav2 config file'
    )
]


def generate_launch_description():

    in_simulation = LaunchConfiguration("in_simulation")
    use_sim_time = in_simulation
    params_file = LaunchConfiguration('params_file')
    # world = LaunchConfiguration('world')
    map_yaml = os.path.join(get_package_share_directory(
        "wheelchair2_navigation"), "maps", "hospital", "hospital.yaml")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"),
                         "launch/bringup_launch.py")
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_launch)

    return ld
