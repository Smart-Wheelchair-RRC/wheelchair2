from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    in_simulation = LaunchConfiguration("in_simulation")
    world = LaunchConfiguration("world")

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
                            "wheelchair2_camera_lidar.urdf.xacro",
                        ]
                    ),
                    " is_sim:=",
                    in_simulation,
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
        parameters=[robot_description, {"use_sim_time": in_simulation}],
    )

    # CONTROLLER MANAGER NODE (Invoke only irl)
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("wheelchair2_bringup"),
                    "config",
                    "wheelchair2_controllers.yaml",
                ]
            ),
            {"use_sim_time": in_simulation},
        ],
        output="screen",
        remappings=[
            # ("/wheelchair2_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
        condition=IfCondition(NotSubstitution(in_simulation)),
    )

    # GAZEBO (SIMULATION)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            ),
        ),
        launch_arguments={"world": world}.items(),
        condition=IfCondition(in_simulation),
    )

    # URDF SPAWNER
    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "wheelchair2", "-topic", "robot_description"],
        output="screen",
        condition=IfCondition(in_simulation),
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
        [FindPackageShare("wheelchair2_control"), 'launch', 'wheelchair_teleop.launch.py'])))

    return LaunchDescription(
        [
            # ARGUMENTS
            DeclareLaunchArgument(
                "in_simulation",
                choices=["true", "false"],
                default_value="false",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("wheelchair2_simulation"),
                        "worlds",
                        "hospital.world",
                    ]
                ),
                description="Gazebo world file",
            ),
            robot_state_publisher,
            gazebo,
            joystick,
            

            # URDF/ROS2 CONTROL
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[ros2_control, robot_spawner],
                )
            ),
            # JOINT STATE BROADCASTER
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_spawner,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            # WHEELCHAIR CONTROLLER
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_controller_spawner],
                )
            ),
        ]
    )