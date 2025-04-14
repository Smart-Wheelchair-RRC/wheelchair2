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
    
    use_sim_time = LaunchConfiguration("use_sim_time")


    # ROBOT DESCRIPTION
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wheelchair2_gazebo"),
                            "urdf",
                            "wheelchair2_camera_livox_gazebo.urdf.xacro",
                        ]
                    ),
                    # " is_sim:=",
                    # use_sim_time,
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
                    FindPackageShare("wheelchair2_bringup"),
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
        condition=IfCondition(NotSubstitution(use_sim_time)),
    )

    default_world = os.path.join(
        get_package_share_directory('wheelchair2_gazebo'),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'wheelchair2',
                                   '-z', '0.1'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("wheelchair2_control"), 'launch', 'wheelchair_teleop.launch.py'])))

    bridge_params = os.path.join(get_package_share_directory('wheelchair2_gazebo'), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                        'use_sim_time',
                        default_value='True',
                        description='Launch in simulation mode'
                    ),
            robot_state_publisher,
            joystick,   
            world_arg,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            ros_gz_bridge,

        
                
            # # URDF/ROS2 CONTROL
            # RegisterEventHandler(
            #     event_handler=OnProcessStart(
            #         target_action=robot_state_publisher,
            #         on_start=[ros2_control],
            #     )
            # ),
            # # JOINT STATE BROADCASTER
            # RegisterEventHandler(
            #     event_handler=OnProcessStart(
            #         target_action=ros2_control,
            #         on_start=[joint_state_broadcaster_spawner],
            #     )
            # ),
            # # WHEELCHAIR CONTROLLER
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=joint_state_broadcaster_spawner,
            #         on_exit=[robot_controller_spawner],
            #     )
            # ),
        ]
    )