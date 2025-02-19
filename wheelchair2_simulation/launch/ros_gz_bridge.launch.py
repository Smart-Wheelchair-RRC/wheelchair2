from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            [
                '/camera/image' +
                '@sensor_msgs/msg/Image' +
                'gz.msgs.Image'],
            [
                '/camera/depth_image' +
                '@sensor_msgs/msg/Image' +
                'gz.msgs.Image'],
            [
                '/camera/points' +
                '@sensor_msgs/msg/PointCloud2' +
                'gz.msgs.PointCloudPacked'],
            [
                '/camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                'gz.msgs.CameraInfo'],
        ],
        remappings=[
            ('/camera/image',
             'camera/color/image_raw'),
            ('/camera/depth_image',
             '/camera/depth/image_raw'),
            ('/camera/points',
             '/camera/depth/points'),
            ('/camera/camera_info',
             'camera/color/camera_info')
        ]
    )

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ["/scan" +
             "@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"]
        ]
    )
    bridge_params = os.path.join(get_package_share_directory(
        "wheelchair2_simulation"), "params", "wheelchair_bridge.yaml")

    bridge_general = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="bridge_general",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(lidar_bridge)
    ld.add_action(camera_bridge)
    ld.add_action(bridge_general)

    return ld
