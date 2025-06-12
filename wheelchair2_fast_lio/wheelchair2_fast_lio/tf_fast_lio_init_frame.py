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
        super().__init__('tf_fast_lio_init_frame')

        # Declare parameters
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('initial_frame_id', 'camera_init')

        # Get parameters
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.initial_frame_id = self.get_parameter('initial_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info(f"tf_fast_lio_init_frame node started, publishing transform from '{self.odom_frame_id}' to '{self.initial_frame_id}'")
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.initial_frame_id
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
