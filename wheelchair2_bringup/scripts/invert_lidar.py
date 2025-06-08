#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Ensure reliable message delivery
    history=HistoryPolicy.KEEP_LAST,        # Keep the last few messages
    depth=10                                # Increase buffer size
)

class InvertLivoxData(Node):

    def __init__(self):
        super().__init__('invert_livox_data')

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, qos_profile=qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, '/livox/imu', self.imu_callback, qos_profile=qos_profile)
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/livox/lidar_inverted', qos_profile=qos_profile)
        self.imu_pub = self.create_publisher(
            Imu, '/livox/imu_inverted', qos_profile=qos_profile)
        self.get_logger().info("invert_livox_data node started")

    def lidar_callback(self, original_message: PointCloud2):
        """
        - extract the x, y, z coordinates from the PointCloud2 message
        - invert the y and z coordinates (180 degrees rotation around x axis)
        - create a new PointCloud2 message with the inverted coordinates
        - publish the new PointCloud2 message
        :param original_message: PointCloud2 message from the /livox/lidar topic
        """

        # Convert PointCloud2 to numpy array
        xyz_data = point_cloud2.read_points(original_message, skip_nans=True)

        # do the inversion
        xyz_data['y'] = -xyz_data['y']
        xyz_data['z'] = -xyz_data['z']

        # Convert back to PointCloud2
        inverted_message = point_cloud2.create_cloud(
            original_message.header, original_message.fields, xyz_data)
        self.lidar_pub.publish(inverted_message)

    def imu_callback(self, original_message: Imu):
        """
        - invert the orientation of the IMU message (180 degrees rotation around x axis)
        - invert the angular velocity of the IMU message (180 degrees rotation around x axis)
        - publish the inverted IMU message
        :param original_message: Imu message from the /livox/imu topic
        """

        original_message.angular_velocity.y = -original_message.angular_velocity.y
        original_message.angular_velocity.z = -original_message.angular_velocity.z

        self.imu_pub.publish(original_message)


def main(args=None):
    rclpy.init(args=args)
    node = InvertLivoxData()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
