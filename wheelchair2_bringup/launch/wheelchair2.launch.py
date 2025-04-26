import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Launch in simulation mode'
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_joy',
            default_value='True',
            description='Use joystick control'
        )
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joy = LaunchConfiguration('use_joy')

    # ROBOT DESCRIPTION
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wheelchair2_description"),
                            "urdf",
                            "wheelchair2_core.urdf.xacro",
                        ]
                    ),
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
        parameters=[robot_description, 
                    {"use_sim_time": use_sim_time}],
    )

    # CONTROLLER MANAGER NODE
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("wheelchair2_control"),
                    "config",
                    "wheelchair2_controllers.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=[
            # ("/wheelchair2_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
        condition=UnlessCondition(use_sim_time),
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

    #JOYSTICK TELEOP
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [
                FindPackageShare("wheelchair2_control"), 'launch', 'wheelchair_teleop.launch.py'
            ])
        ),
        condition=IfCondition(use_joy),
    )

    # delay start of joint_state_broadcaster_spawner after ros2 control 
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[joint_state_broadcaster_spawner],
                )
            )
    
    #delay start of robot_controller_spawner after joint_state_broadcaster_spawner
    delay_robot_controller_spawner = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_controller_spawner],
                )
            )
    
    nodes = [
            robot_state_publisher,
            ros2_control,
            joystick,
            delay_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner,
    ]
    
    return LaunchDescription(launch_arguments + nodes)