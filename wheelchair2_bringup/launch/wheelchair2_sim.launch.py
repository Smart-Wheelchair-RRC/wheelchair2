from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_description = LaunchDescription()

    # ROBOT DESCRIPTION
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("wheelchair2_description"),
                    "urdf",
                    "wheelchair2_livox.urdf.xacro",
                ]
            ),
            " is_sim:=true",
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

    # CONTROLLER MANAGER (launched by gazebo implicitly)
    #   GAZEBO
    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            ),
        )
    )
    launch_description.add_action(gazebo_launcher)

    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "wheelchair2", "-topic", "robot_description"],
        output="screen",
    )
    #   Delay robot spawn/controller manager start to when robot state publisher is ready
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_publisher_node, on_start=[robot_spawner]
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
    #   Delay launch of joint state broadcaster after gazebo entity has spawned
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawner,
                on_exit=[joint_state_broadcaster_spawner],
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

    return launch_description
