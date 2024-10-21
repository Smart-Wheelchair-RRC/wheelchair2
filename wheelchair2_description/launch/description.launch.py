from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get wheelchair2_description_path

    pkg_wheelchair2_description = get_package_share_directory(
        "wheelchair2_description")
    # Get URDF via xacro
    robot_description = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [pkg_wheelchair2_description, "urdf",
                            "wheelchair2_livox.urdf.xacro"]
                    ),
                ]
            ),
            value_type=str,
        )
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
        ]
    )
