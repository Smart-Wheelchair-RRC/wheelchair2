import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    world = os.path.join(get_package_share_directory(
        "wheelchair2_gazebo"), "worlds", "empty_world.world")

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory(
            "wheelchair2_description"), "models"),
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g "}.items(),
    )

    urdf_path = os.path.join(get_package_share_directory(
        "wheelchair2_description"), "models",  "model.sdf")

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
        }],
    )
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "wheelchair2",
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2'
        ],
        output='screen',
    )

    bridge_params = os.path.join(get_package_share_directory(
        "wheelchair2_description"), "params", "wheelchair_bridge.yaml")

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/image', 'camera/depth_image'],
        output='screen'
    )

    # robot_description = launch_ros.descriptions.ParameterValue(
    #     Command(["xacro", get]), value_type=str
    # )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"robot_description": robot_description}],)
    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(bridge)
    # ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(robot_state_pub_node)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
