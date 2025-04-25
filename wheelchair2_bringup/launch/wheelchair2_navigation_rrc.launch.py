import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument
)
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
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare launch arguments

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
            'use_keepout_zone',
            default_value='False',
            description='Launch with keepout zones'
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_joy',
            default_value='True',
            description='Use joystick control'
        )
    )

    #Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration('autostart', default="true")
    use_keepout_zone = LaunchConfiguration('use_keepout_zone')
    use_joy = LaunchConfiguration('use_joy')

    # lifecycle node for mask layer
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']


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

    #CONTROLLER MANAGER NODE
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
        ],
        output="screen",
        remappings=[
            # ("/wheelchair2_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/wheelchair2_base_controller/odom", "/odom"),
        ],
    )

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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


    #JOYSTICK TELEOP
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [
                FindPackageShare("wheelchair2_control"), 'launch', 'wheelchair_teleop.launch.py'
            ])
        ),
        condition=IfCondition(use_joy),
    )


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

    rviz_config = os.path.join(get_package_share_directory('wheelchair2_navigation'), 'rviz', 'navigation.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,  
            condition=IfCondition(use_keepout_zone),
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_keepout_zone),
            parameters=[configured_params])

    costmap_filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  
            condition=IfCondition(use_keepout_zone),
            parameters=[configured_params])
    
    # delay start of ros2_control after robot_state_publisher
    delay_ros2_control = RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[ros2_control],
                )
            )
    # delay start of joint_state_broadcaster_spawner after ros2 control 
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control,
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
            livox_launch,
            pointcloud_to_laserscan_launch,
            robot_state_publisher,
            delay_ros2_control,
            delay_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner,
            joystick,
            nav2,
            rviz,
            lifecycle_manager,
            map_server,
            costmap_filter_info_server,  
    ]

    return LaunchDescription(launch_arguments + nodes)