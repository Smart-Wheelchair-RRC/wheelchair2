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
        self.declare_parameter('base_frame_id', 'base_footprint')

        # Get parameters
        self.flodom_frame_id = self.get_parameter('flodom_frame_id').get_parameter_value().string_value
        self.lidar_mount_frame_id = self.get_parameter('lidar_mount_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms
        self.flodom_to_base_tf()


    def flodom_to_base_tf(self):
        TRANSLATION = [-0.082, -0.018, -1.361]
        ROTATION = [0.000, 0.007, 0.000, 1.000]

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.flodom_frame_id  # body
        t.child_frame_id = self.base_frame_id  # base_footprint
        t.transform.translation.x = TRANSLATION[0]
        t.transform.translation.y = TRANSLATION[1]
        t.transform.translation.z = TRANSLATION[2]
        t.transform.rotation.x = ROTATION[0]
        t.transform.rotation.y = ROTATION[1]
        t.transform.rotation.z = ROTATION[2]
        t.transform.rotation.w = ROTATION[3]

        self.tf_broadcaster.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = TfFastLioInitFrame(sys.argv)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
