from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('x', default_value='0.0',
                          description='x component of the robot pose.'),
    DeclareLaunchArgument('y', default_value='0.0',
                          description='y component of the robot pose.'),
    DeclareLaunchArgument('z', default_value='0.0',
                          description='z component of the robot pose.'),
]


def generate_launch_description():
    pkg_wheelchair2_description = get_package_share_directory(
        "wheelchair2_description")
    pkg_wheelchair2_simulation = get_package_share_directory(
        "wheelchair2_simulation")

    # Paths
    wheelchair2_ros_gz_bridge = PathJoinSubstitution(
        [pkg_wheelchair2_simulation, "launch", "ros_gz_bridge.launch.py"]
    )

    # TODO: RVIZ LaunchDescription
    # rviz_launch = PathJoinSubstitution(
    # [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])

    robot_description_launch = PathJoinSubstitution(
        [pkg_wheelchair2_description, "launch", "wheelchair_description.launch.py"]
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration(
        'y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    spawn_robot_group_action = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[
                ('use_sim_time', LaunchConfiguration('use_sim_time'))]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', "wheelchair2",
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),

        # ros_gz_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wheelchair2_ros_gz_bridge]),
            launch_arguments=[
                ('use_sim_time', use_sim_time)
            ]
        )
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)

    return ld
