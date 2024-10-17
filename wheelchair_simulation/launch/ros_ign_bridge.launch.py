from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "robot_name", default_value="wheelchair2", description="Ignition model name"
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument("world", default_value="warehouse", description="World name"),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")
    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")

    # lidar bridge
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/rplidar_link/sensor/rplidar/scan"
                + "@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ]
        ],
        remappings=[
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/rplidar_link/sensor/rplidar/scan",
                ],
                "scan",
            )
        ],
    )

    # Camera sensor bridge
    oakd_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points"
                + "@sensor_msgs/msg/PointCloud2"
                + "[ignition.msgs.PointCloudPacked",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info"
                + "@sensor_msgs/msg/CameraInfo"
                + "[ignition.msgs.CameraInfo",
            ],
        ],
        remappings=[
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image",
                ],
                "oakd/rgb/preview/image_raw",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image",
                ],
                "oakd/rgb/preview/depth",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points",
                ],
                "oakd/rgb/preview/depth/points",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info",
                ],
                "oakd/rgb/preview/camera_info",
            ),
        ],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(lidar_bridge)
    ld.add_action(oakd_camera_bridge)
    return ld
