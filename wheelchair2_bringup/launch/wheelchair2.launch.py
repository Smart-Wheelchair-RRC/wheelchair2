import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    OpaqueFunction
)
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joy = LaunchConfiguration('use_joy')
    use_ducky = LaunchConfiguration("use_ducky")
    bringup_dir = get_package_share_directory("wheelchair2_description")



    if use_ducky.perform(context)== "True":
        xacro_path = os.path.join(bringup_dir, "urdf", "ducky_core.urdf.xacro")
    else:
        xacro_path = os.path.join(bringup_dir, "urdf", "wheelchair2_core.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    xacro_path,
                    " use_sim_time:=",
                    use_sim_time,
                ]
            ),
            value_type=str,
        )
    }

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # CONTROLLER MANAGER NODE
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("wheelchair2_control"),
                "config",
                "wheelchair2_controllers.yaml",
            ]),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=[
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # JOINT STATE BROADCASTER
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # WHEELCHAIR CONTROLLER
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheelchair2_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # JOYSTICK TELEOP
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("wheelchair2_control"), 'launch', 'wheelchair_teleop.launch.py'
        ])),
        condition=IfCondition(use_joy),
    )

    # delay start of joint_state_broadcaster_spawner after robot_state_publisher
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # delay start of robot_controller_spawner after joint_state_broadcaster_spawner
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return [
        robot_state_publisher,
        ros2_control,
        joystick,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
    ]


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Launch in simulation mode'
        ),
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Use joystick control'
        ),
        DeclareLaunchArgument(
            'use_ducky',
            default_value='false',
            description='Launch base description'
        ),
    ]

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
