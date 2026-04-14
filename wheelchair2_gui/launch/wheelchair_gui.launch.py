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
        package='wheelchair2_gui',
        executable='wheelchair2_gui',
        name='wheelchair2_gui',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        gui_node
    ])