from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.conditions import IfCondition
import launch_ros
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument(
            "in_simulation",
            choices=["true", "false"],
            default_value="true",
        )
    )
    in_simulation = LaunchConfiguration("in_simulation")

    launch_description.add_action(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("wheelchair2_simulation"),
                 "worlds", "hospital.world"]
            ),
            description='Gazebo world file'
        )
    )
    world = LaunchConfiguration("world")

    # ROBOT DESCRIPTION
    robot_description_content = Command(
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
    )
    robot_description = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ROBOT STATE PUBLISHER
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    launch_description.add_action(robot_state_publisher_node)

    # CONTROLLER MANAGER
    #   NODE
    controller_manager_node = Node(
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
        ],
        output="both",
        remappings=[
            ("/wheelchair2_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
        condition=IfCondition(NotSubstitution(in_simulation)),
    )

    #   GAZEBO
    source_gazebo_command = Command(["source ", "/usr/share/gazebo/setup.sh"])
    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            ),

        ),
        launch_arguments={
            "world": world,
            # "verbose": "true"
        }.items(),
        condition=IfCondition(in_simulation)
    )
    launch_description.add_action(gazebo_launcher)

    #   URDF SPAWNER
    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "wheelchair2", "-topic", "robot_description"],
        output="screen",
        condition=IfCondition(in_simulation),
    )

    #   Delay controller manager start to when robot state publisher is ready
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[controller_manager_node, robot_spawner],
            )
        )
    )

    #   JOINT STATE BROADCASTER
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
    #   Delay launch of joint state broadcaster after either controller manager or gazebo spawner has started
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        )
    )
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_spawner,
                on_start=[joint_state_broadcaster_spawner],
            )
        )
    )

    #   WHEELCHAIR2 CONTROLLER
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
    #   Delay start of robot_controller after `joint_state_broadcaster`
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    #   RVIZ2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("wheelchair2_bringup"), "rviz", "wheelchair2.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        )
    )

    return launch_description
