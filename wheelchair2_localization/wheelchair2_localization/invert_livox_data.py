"""
Livox lidar data inversion node
- IMU data inversion is currently disabled (commented out) since it is not used in the localization pipeline
- subscribes to /livox/lidar (PointCloud2) and /livox/imu (Imu)
- publishes to /livox/lidar_inverted (PointCloud2) and /livox/imu_inverted (Imu)
- inversion is defined as a 180 degrees rotation around the x axis
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Ensure reliable message delivery
    history=HistoryPolicy.KEEP_LAST,        # Keep the last few messages
    depth=1000                              # Increase buffer size
)

class InvertLivoxData(Node):

    def __init__(self):
        super().__init__('invert_livox_data')

        self.setup_parameters()

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_lidar_topic').get_parameter_value().string_value,
            self.lidar_callback,
            qos_profile=qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('input_imu_topic').get_parameter_value().string_value,
            self.imu_callback,
            qos_profile=qos_profile
        )

        self.lidar_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('inverted_lidar_topic').get_parameter_value().string_value,
            qos_profile=qos_profile
        )

        self.imu_pub = self.create_publisher(
            Imu,
            self.get_parameter('inverted_imu_topic').get_parameter_value().string_value,
            qos_profile=qos_profile
        )

        self.get_logger().info("invert_livox_data node started")

    def setup_parameters(self):
        self.declare_parameter('input_lidar_topic', '/livox/lidar')
        self.declare_parameter('input_imu_topic', '/livox/imu')
        self.declare_parameter('inverted_lidar_topic', '/livox/lidar_inverted')
        self.declare_parameter('inverted_imu_topic', '/livox/imu_inverted')
        self.declare_parameter('lidar_mount_frame_id', 'livox_frame')

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
        
        # inverted_message.header.frame_id = self.get_parameter('lidar_monut_frame_id').get_parameter_value().string_value

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

        # original_message.header.frame_id = self.get_parameter('lidar_monut_frame_id').get_parameter_value().string_value

        self.imu_pub.publish(original_message)


def main(args=None):
    rclpy.init(args=args)
    node = InvertLivoxData()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("InvertLivoxData node stopped by user")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()