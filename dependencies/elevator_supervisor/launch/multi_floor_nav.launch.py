"""
multi_floor_nav.launch.py

Launches:
  1. elevator_supervisor_node   ← the new coordinator
  2. icp_localization_ros2      ← your existing ICP node (unchanged)
  3. DLIO                       ← continuous odometry (unchanged, not shown here)
  4. 3D-BBS                     ← global relocalization (unchanged, not shown here)

Usage:
  ros2 launch elevator_supervisor multi_floor_nav.launch.py

To trigger a floor change manually (for testing):
  ros2 topic pub /elevator/target_floor std_msgs/msg/Int32 "{data: 4}" --once
  ros2 topic pub /elevator/is_inside    std_msgs/msg/Bool  "{data: true}" --once
  # ... wait for lift to arrive ...
  ros2 topic pub /elevator/is_inside    std_msgs/msg/Bool  "{data: false}" --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ── Argument: path to the stacked multi-floor PCD map ────────────────────
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="/maps/stacked_all_floors.pcd",
        description="Path to the pre-compiled stacked Z multi-floor PCD map."
    )

    # ── Argument: ICP config ──────────────────────────────────────────────────
    icp_config_arg = DeclareLaunchArgument(
        "icp_config",
        default_value="",
        description="Path to the ICP YAML config file."
    )

    # ── elevator_supervisor_node ──────────────────────────────────────────────
    elevator_params = PathJoinSubstitution([
        FindPackageShare("elevator_supervisor"),
        "config",
        "elevator_params.yaml"
    ])

    elevator_node = Node(
        package    = "elevator_supervisor",
        executable = "elevator_supervisor_node",
        name       = "elevator_supervisor",
        output     = "screen",
        parameters = [elevator_params],
    )

    # ── icp_localization_ros2 node ────────────────────────────────────────────
    # This is your EXISTING node — we add NO new parameters to it.
    # The pause/reset services it already exposes are called by the supervisor.
    icp_node = Node(
        package    = "icp_localization_ros2",
        executable = "icp_localization_ros2_node",
        name       = "icp_localization",
        output     = "screen",
        parameters = [
            {"icp_config_path": LaunchConfiguration("icp_config")},
            # All other ICP params come from your existing params file.
            # Add: parameters=[your_existing_icp_params.yaml, {...}]
        ],
    )

    return LaunchDescription([
        map_path_arg,
        icp_config_arg,
        elevator_node,
        icp_node,
        # Add your DLIO and 3D-BBS nodes here as normal
    ])