import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='use_sim_time'
    ),
    DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory(
            "wheelchair2_gazebo"), "worlds", "empty_world.world"),
        description="Ignition World"
    )
]


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    resource_path = os.path.join(get_package_share_directory(
        "wheelchair2_description"), "models")
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", resource_path
    )

    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )

    # Ignition LaunchConfiguration
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration("world")]), '-r']
    )

    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' +
                                'ignition.msgs.Clock'])

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(gz_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    return ld
