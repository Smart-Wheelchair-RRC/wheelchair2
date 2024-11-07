#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as point_cloud2
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from geometry_msgs.msg import TransformStamped
from transforms3d.affines import compose
from transforms3d.quaternions import quat2mat
from rclpy.time import Time
from rclpy.duration import Duration


class Pointcloud3Dto2D(Node):
    def __init__(self, target_frame="chassis", z_range=[-0.3, 1.5]):
        super().__init__('pointcloud_3d_to_2d')

        self.target_frame = target_frame
        self.z_range = z_range

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define point cloud fields
        self.fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]

        # Create publisher and subscriber
        self.pub = self.create_publisher(
            PointCloud2,
            '/livox/pcd_2d',
            1
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            1
        )

        self.get_logger().info('Pointcloud 3D to 2D node initialized')

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """Convert a TransformStamped to a 4x4 transformation matrix."""
        translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]

        rotation = [
            transform.transform.rotation.w,  # quat format: w, x, y, z
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        ]

        # Convert quaternion to rotation matrix
        rotation_matrix = quat2mat(rotation)

        # Create transformation matrix
        transform_matrix = compose(translation, rotation_matrix, np.ones(3))
        return transform_matrix

    def transform_points(self, points: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
        """Transform points using the transformation matrix."""
        # Add homogeneous coordinate (1) to each point
        homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))

        # Transform points
        transformed_points = (transform_matrix @ homogeneous_points.T).T

        # Return to 3D coordinates
        return transformed_points[:, :3]

    def transform_pointcloud(self, pointcloud_msg: PointCloud2):
        # Wait for the transform between frames
        trans = self.tf_buffer.lookup_transform(
            self.target_frame,
            pointcloud_msg.header.frame_id,  # Source frame
            Time(),  # Get the latest available transform
            Duration(seconds=1)  # Timeout
        )

        # Convert transform to matrix
        transform_matrix = self.transform_to_matrix(trans)

        # Get points from PointCloud2
        points = point_cloud2.read_points(
            pointcloud_msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )
        points_array = np.array(list(points))

        if points_array.size == 0:
            return None

        # Transform points
        transformed_points = self.transform_points(
            points_array, transform_matrix)

        return transformed_points, pointcloud_msg.header.frame_id

    def pointcloud_callback(self, pcd_msg: PointCloud2):
        transform_result = self.transform_pointcloud(pcd_msg)

        if transform_result is not None:
            xyz_array, original_frame = transform_result

            if xyz_array.size == 0:
                self.get_logger().warn('Empty point cloud received')
                return

            self.get_logger().debug(f'Point cloud shape: {xyz_array.shape}')

            # Filter points based on z-range
            mask = np.where(
                (xyz_array[:, 2] >= self.z_range[0]) &
                (xyz_array[:, 2] <= self.z_range[1])
            )
            xyz_array = xyz_array[mask]

            if xyz_array.size == 0:
                self.get_logger().warn('No points remaining after z-range filter')
                return

            # Set z coordinates to 0
            xyz_array[:, 2] = 0

            # Create and publish the 2D point cloud
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.target_frame

            out_pcd_msg = point_cloud2.create_cloud_xyz32(header, xyz_array)
            self.pub.publish(out_pcd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Pointcloud3Dto2D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
