from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='wheelchair2',
                          description='Robot name'),
]


def generate_launch_description():
    pkg_wheelchair2_description = get_package_share_directory(
        "wheelchair2_description")

    xacro_file = PathJoinSubstitution(
        [pkg_wheelchair2_description, "urdf", "gz_sim", "wheelchair2_camera_lidar.urdf.xacro"])

    xacro_data = ParameterValue(
        Command(["xacro", " ", xacro_file, " ", "gazebo:=ignition"]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': xacro_data},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld
