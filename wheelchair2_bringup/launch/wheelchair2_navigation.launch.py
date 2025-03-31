from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set permissions for the Kangaroo motor controller
    set_permissions = ExecuteProcess(
        cmd=["sudo", "chmod", "777", "/dev/ttyACM0"],
        output="screen",
    )

    # Launch wheelchair bringup
    wheelchair_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_bringup"), "launch", "wheelchair2.launch.py"]
            )
        ),
    )

    # Launch 3D LiDAR and pointcloud to laserscan
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_bringup"), "launch", "livox.launch.py"]
            )
        ),
    )

    # Launch navigation stack
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wheelchair2_navigation"), "launch", "nav2_amcl.launch.py"]
            )
        ),
    )

    # Launch RViz with the specified configuration
    rviz_launch = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("wheelchair2_navigation"),
                    "rviz",
                    "navigation_amcl.rviz",  # Ensure the file exists in this location
                ]
            ),
        ],
        output="screen",
    )

    # Add delays between launches
    return LaunchDescription(
        [
            set_permissions,
            TimerAction(period=5.0, actions=[wheelchair_bringup]),
            TimerAction(period=10.0, actions=[livox_launch]),
            TimerAction(period=15.0, actions=[navigation_stack]),
            TimerAction(period=20.0, actions=[rviz_launch]),
        ]
    )
