import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

def generate_launch_description():

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

    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
                "map",
                default_value=map_yaml,
                description="Full path to map file to load",
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
            "keepout_params_file",
            default_value=os.path.join(
                get_package_share_directory("wheelchair2_navigation"),
                "config",
                "keepout_params.yaml",
            ),
            description="Full path to keepout zone parameters file",
        )
    )

    launch_arguments.append(
        DeclareLaunchArgument(
            "keepout_mask_file_path",
            default_value=os.path.join(
                get_package_share_directory("wheelchair2_navigation"),
                "maps",
                "rrc",
                "rrc_lab_mask.yaml",
            ),
            description="Full path to keepout zone mask file",
        )
    )
            
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_keepout_zone = LaunchConfiguration('use_keepout_zone', default='False')


    
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "nav2_params.yaml",
        ),
    )

    keepout_params_file = LaunchConfiguration(
        'keepout_params_file',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "config",
            "keepout_params.yaml",
        ),
    )

    keepout_mask_file_path = LaunchConfiguration(
        'keepout_mask_file_path',
        default=os.path.join(
            get_package_share_directory("wheelchair2_navigation"),
            "maps",
            "rrc",
            "rrc_lab_mask.yaml",
        ),
    )

    updated_keepout_params = RewrittenYaml(
        source_file=keepout_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask_file_path
        },
        convert_types=True
    )

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

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

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,  
        condition=IfCondition(use_keepout_zone),
        parameters=[updated_keepout_params]
    )
    
    filtered_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_keepout_zone),
        parameters=[updated_keepout_params]
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,  
        condition=IfCondition(use_keepout_zone),
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    rviz_config = os.path.join(get_package_share_directory('wheelchair2_navigation'), 'rviz', 'navigation.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    nodes = [
        nav2,
        rviz,
        filtered_map_server,
        lifecycle_manager,
        costmap_filter_info_server
    ]

    return LaunchDescription(launch_arguments + nodes)
