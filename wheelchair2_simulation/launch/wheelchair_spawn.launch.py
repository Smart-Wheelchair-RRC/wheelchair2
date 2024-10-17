from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start rviz.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="use_sim_time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    # DeclareLaunchArgument(
    #     "localization",
    #     default_value="false",
    #     choices=["true", "false"],
    #     description="Whether to launch localization",
    # ),
    # DeclareLaunchArgument(
    #     "slam",
    #     default_value="false",
    #     choices=["true", "false"],
    #     description="Whether to launch SLAM",
    # ),
    # DeclareLaunchArgument(
    #     "nav2",
    #     default_value="false",
    #     choices=["true", "false"],
    #     description="Whether to launch Nav2",
    # ),
]

for pose_element in ["x", "y", "z", "yaw"]:
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value="0.0",
            description=f"{pose_element} component of the robot pose.",
        )
    )


def generate_launch_description():

    # Dirs
    pkg_wheelchair2_simulation = get_package_share_directory("wheelchair2_simulation")
    pkg_wheelchair2_description = get_package_share_directory("wheelchair2_description")
    pkg_wheelchair2_description = get_package_share_directory("wheelchair2_description")

    # Paths
    wheelchair2_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_wheelchair2_simulation, "launch", "ros_ign_bridge.launch.py"]
    )
    wheelchair2_description_launch = PathJoinSubstitution(
        [pkg_wheelchair2_description, "launch", "robot_description.launch.py"]
    )

    # Launch Configurations
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x, y, z = (
        LaunchConfiguration("x"),
        LaunchConfiguration("y"),
        LaunchConfiguration("z"),
    )
    yaw = LaunchConfiguration("yaw")

    spawn_robot_group_action = GroupAction(
        [
            PushRosNamespace(namespace),
            # Robot description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([wheelchair2_description_launch]),
                launch_arguments=[
                    ("use_sim_time", LaunchConfiguration("use_sim_time"))
                ],
            ),
            Node(
                package="ros_ign_gazebo",
                executable="create",
                arguments=[
                    "-name",
                    "wheelchair2",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                    "-Y",
                    yaw,
                    "-topic",
                    "robot_description",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([wheelchair2_ros_ign_bridge_launch]),
                launch_arguments=[
                    ("model", LaunchConfiguration("model")),
                    ("robot_name", "wheelchair2"),
                    ("namespace", namespace),
                ],
            ),
            # RPLIDAR static transforms
            Node(
                name="rplidar_stf",
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0.0",
                    "rplidar_link",
                    ["wheelchair2", "/rplidar_link/rplidar"],
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
            # OAKD static transform
            # Required for pointcloud. See https://github.com/gazebosim/gz-sensors/issues/239
            Node(
                name="camera_stf",
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "1.5707",
                    "-1.5707",
                    "0",
                    "oakd_rgb_camera_optical_frame",
                    ["wheelchair2", "/oakd_rgb_camera_frame/rgbd_camera"],
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)

    return ld
