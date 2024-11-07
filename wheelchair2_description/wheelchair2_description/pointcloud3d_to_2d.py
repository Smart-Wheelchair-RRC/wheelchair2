#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.duration import Duration


class Pointcloud3Dto2D(Node):
    def __init__(self, target_frame="chassis", z_range=[-0.3, 1.5]):
        super().__init__('pointcloud_3d_to_2d')

        self.target_frame = target_frame
        self.z_range = z_range
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]

        self.pub = self.create_publisher(
            PointCloud2,
            '/pcd_2d',
            10
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/mid360_PointCloud2',
            self.pointcloud_callback,
            10
        )

        self.get_logger().info('Pointcloud 3D to 2D node started...')

    def transform_pointcloud(self, msg: PointCloud2):

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time(),
                Duration(seconds=1)
            )

            # Transformation Matrix
            t = np.eye(4)
            q = tf.transform.rotation
            x = tf.transform.translation
            t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            t[:3, 3] = [x.x, x.y, x.z]

            # Transform
            points_msg = pc2.read_points_numpy(msg)
            points_map = np.ones((len(points_msg), 4))
            points_map[:, :3] = points_msg
            points_map = np.dot(t, points_map.T).T

            mask = (points_map[:, 2] >= self.z_range[0]) & (
                points_map[:, 2] <= self.z_range[1])
            filtered_points = points_map[mask]

            # Remove z-axis
            filtered_points[:, 2] = 0.0

            header = Header()
            header.frame_id = self.target_frame
            header.stamp = self.get_clock().now().to_msg()

            cloud_msg = pc2.create_cloud(
                header, self.fields, filtered_points[:, :3])

            return cloud_msg

        except Exception as e:
            self.get_logger().error(f'Transform failed: {str(e)}')
            return None

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Process incoming pointcloud messages
        """
        transformed_msg = self.transform_pointcloud(msg)
        if transformed_msg is not None:
            self.pub.publish(transformed_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Pointcloud3Dto2D()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    except Exception as e:
        print(f'Error initializing node: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
