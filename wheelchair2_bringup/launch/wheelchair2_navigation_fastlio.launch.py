from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
# from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Set permissions for the Kangaroo motor controller
    set_permissions = ExecuteProcess(
        cmd=["sudo", "chmod", "777", "/dev/ttyACM0"],
        output="screen",
    )
    
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    declare_rviz_cfg = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=os.path.join(
            get_package_share_directory("wheelchair2_navigation"), "rviz", "navigation_fastlio.rviz"
        ),
        description="Path to RViz config file",
    )

    # Launch wheelchair bringup
    wheelchair_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("wheelchair2_bringup"), "launch", "wheelchair2_fastlio.launch.py"]
            )
        ),
    )


    # Launch 3D LiDAR
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("fast_lio_localization"), "launch", "livox.launch.py"]
            ),
        ),
        launch_arguments={
            "inverted": "true",
            "xfer_format": "1",
            "rviz": "false",
        }.items(),
    )
    
    # Launch livox custom msg to pcd
    livox_custom_msg_to_pcd = Node(
        package="wheelchair2_laser",
        executable="livox_custom_msg_to_pointcloud2.py",
        name="livox_custom_msg_to_pointcloud2",
        output="screen",
    )

    # Launch navigation stack
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("wheelchair2_navigation"), "launch", "nav2_fastlio.launch.py"]
            )
        ),
    )

    
    # Launch RViz with the specified configuration
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], name="rviz2", output="screen")
    

    # Add delays between launches
    ld = LaunchDescription()
    ld.add_action(declare_rviz_cfg)
    ld.add_action(set_permissions)
    ld.add_action(TimerAction(period=5.0, actions=[wheelchair_bringup]))
    ld.add_action(TimerAction(period=10.0, actions=[livox_launch, livox_custom_msg_to_pcd]))
    ld.add_action(TimerAction(period=15.0, actions=[navigation_stack]))
    ld.add_action(TimerAction(period=20.0, actions=[rviz_node]))
    return ld