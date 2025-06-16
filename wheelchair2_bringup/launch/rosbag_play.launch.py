import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_latest_rosbag_dir(records_root):
    try:
        subdirs = [
            os.path.join(records_root, d)
            for d in os.listdir(records_root)
            if os.path.isdir(os.path.join(records_root, d))
        ]
        if not subdirs:
            return None
        latest_dir = max(subdirs, key=os.path.getmtime)
        rosbag_dir = os.path.join(latest_dir, "rosbag")
        if os.path.isdir(rosbag_dir):
            return rosbag_dir
        else:
            return None
    except Exception as e:
        print(f"Error finding latest rosbag dir: {e}")
        return None

def launch_playback(context, *args, **kwargs):
    records_root = '/home/wheelchair2/wheelchair2/src/records'
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    if bag_dir and os.path.isdir(bag_dir):
        selected_bag_dir = bag_dir
    else:
        selected_bag_dir = get_latest_rosbag_dir(records_root)
    if not selected_bag_dir:
        print("No rosbag found in records directory.")
        return []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', get_package_share_directory('wheelchair2_bringup') + '/rviz/data_collection.rviz'],
    )
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-l', selected_bag_dir],
        output='screen'
    )
    return [rosbag_play, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_dir',
            default_value='',
            description='Full path to the rosbag directory to play (leave empty for latest)'
        ),
        OpaqueFunction(function=launch_playback)
    ])