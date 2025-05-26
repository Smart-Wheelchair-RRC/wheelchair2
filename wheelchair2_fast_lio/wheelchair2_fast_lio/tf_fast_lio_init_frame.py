"""
TF broadcaster for the initial camera frame in Fast LIO.
- publishes a static transform from `map` to `camera_init`
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
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("tf_fast_lio_init_frame node started")
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = transformation[1]
        t.header.frame_id = 'odom'
        t.child_frame_id = 'camera_init'
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
