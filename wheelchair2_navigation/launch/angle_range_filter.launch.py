from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("wheelchair2_navigation"),
                            "config",
                            "angle_range_filter.yaml",
                        ]
                    )
                ],
                remappings=[("scan", "/scan"), ("/scan_filtered", "/scan_out")],
            )
        ]
    )
