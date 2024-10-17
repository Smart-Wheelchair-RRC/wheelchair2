from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="", description="Robot Namespace"),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start Rviz",
    ),
    DeclareLaunchArgument(
        "world", default_value="warehouse", description="Ignition World"
    ),
]

for pose_element in ["x", "y", "z", "yaw"]:
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value="0.0",
            description=f"{pose_element} component of the robot pose.",
        )
    )


def generate_launch_description():

    # Directories
    pkg_wheelchair2_simulation = get_package_share_directory("wheelchair2_simulation")

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_wheelchair2_simulation, "launch", "ignition.launch.py"]
    )
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_wheelchair2_simulation, "launch", "wheelchair2_spawn.launch"]
    )

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[("world", LaunchConfiguration("world"))],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ("namespace", LaunchConfiguration("namespace")),
            ("rviz", LaunchConfiguration("rviz")),
            ("x", LaunchConfiguration("x")),
            ("y", LaunchConfiguration("y")),
            ("z", LaunchConfiguration("z")),
            ("yaw", LaunchConfiguration("yaw")),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(ignition)
    ld.add_action(robot_spawn)

    return ld
