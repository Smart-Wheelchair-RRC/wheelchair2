import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
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
            nav2,
            lifecycle_manager,
            map_server,
            costmap_filter_info_server,
        ]
    )
