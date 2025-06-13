"""
TF broadcaster for the initial camera frame in Fast LIO.
- publishes a static transform from `odom` to `camera_init`
- the transform is set to identity (0 translation, 0 rotation)
"""

import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class TfFastLioInitFrame(Node):

    def __init__(self, transformation):
        super().__init__('fast_lio_tfs')

        # Declare parameters
        self.declare_parameter('flodom_frame_id', 'body')
        self.declare_parameter('lidar_mount_frame_id', 'livox_frame')

        # Get parameters
        self.flodom_frame_id = self.get_parameter('flodom_frame_id').get_parameter_value().string_value
        self.lidar_mount_frame_id = self.get_parameter('lidar_mount_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.flodom_to_lidar_mount_tf()

    def flodom_to_lidar_mount_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.flodom_frame_id
        t.child_frame_id = self.lidar_mount_frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfFastLioInitFrame(sys.argv)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
