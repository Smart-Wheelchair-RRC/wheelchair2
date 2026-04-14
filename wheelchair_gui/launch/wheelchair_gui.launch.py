import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    rosbridge_launch_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_path)
    )

    # 3. Wheelchair GUI Node
    gui_node = Node(
        package='wheelchair_gui',
        executable='wheelchair_gui',
        name='wheelchair_gui',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        gui_node
    ])