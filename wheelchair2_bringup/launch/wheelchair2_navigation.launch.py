import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("livox_ros_driver2"), "launch_ROS2", "msg_MID360_launch.py"]
            ),
        ),
    )

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pointcloud_to_laserscan"), "launch", "sample_pointcloud_to_laserscan_launch.py"]
            ),
        ),
    )

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
                            "wheelchair2_camera_livox.urdf.xacro",
                        ]
                    ),
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
                    ],
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
        ],
        output="screen",
        remappings=[
            # ("/wheelchair2_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
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

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    autostart = LaunchConfiguration('autostart', default="true")

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']


    keepout_params_file = LaunchConfiguration(
        'keepout_params_file',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "keepout_params.yaml",
        ),
    )
                                                
    mask_yaml_file = LaunchConfiguration(
        'mask',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab_mask.yaml",
        ),
    )

    # Map and Nav2 config files
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab.yaml",
        ),
    )
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "nav2_params.yaml",
        ),
    )
    param_substitutions = {
            'use_sim_time': use_sim_time,
            'yaml_filename': mask_yaml_file}

    configured_params = RewrittenYaml(
            source_file=keepout_params_file,
            param_rewrites=param_substitutions,
            convert_types=True)
    

    # Nav2 bringup launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
    )

    rviz_config = os.path.join(get_package_share_directory('wheelchair2_navigation'), 'launch', 'navigation.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d' + rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,  
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True, 
            parameters=[configured_params])

    costmap_filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  
            parameters=[configured_params])

    return LaunchDescription(
        [   
            livox_launch,
            pointcloud_to_laserscan_launch,
            robot_state_publisher,
            joystick,
            nav2,
            rviz,
            lifecycle_manager,
            map_server,
            costmap_filter_info_server,       

            # URDF/ROS2 CONTROL
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[ros2_control],
                )
            ),
            # JOINT STATE BROADCASTER
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control,
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