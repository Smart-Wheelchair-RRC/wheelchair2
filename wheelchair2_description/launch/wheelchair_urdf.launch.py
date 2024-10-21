from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition


from launch_ros.actions import Node

# NOTE: Publishes robot description on the /robot_description topic


def generate_launch_description():
    ld = LaunchDescription()

    wheelchair2_description_path = FindPackageShare("wheelchair2_description")
    default_model_path = PathJoinSubstitution(
        [wheelchair2_description_path, "urdf",
            "wheelchair2_camera_lidar.urdf.xacro"]
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI",
    )
    ld.add_action(gui_arg)

    world_path_arg = DeclareLaunchArgument(
        "world_path",
        default_value="",
        description="The world path, by default is empty.world",
    )
    ld.add_action(world_path_arg)

    # These parameters are maintained for backwards compatibility
    jsp_gui_arg = DeclareLaunchArgument(
        name="jsp_gui",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    ld.add_action(jsp_gui_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Path to robot urdf file relative to wheelchair2_description package",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch",
                 "display.launch.py"]
            ),
            launch_arguments={
                "urdf_package": "wheelchair2_description",
                "urdf_package_path": LaunchConfiguration("model"),
                "jsp_gui": LaunchConfiguration("jsp_gui"),
            }.items(),
        )
    )

    world_path = LaunchConfiguration("world_path")

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world_path,
        ],
        output="screen",
    )
    ld.add_action(gzserver)
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    ld.add_action(gzclient)

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_wheelchair2",
        arguments=["-entity", "wheelchair2", "-topic", "robot_description"],
        output="screen",
    )
    ld.add_action(spawn_robot)
    return ld
