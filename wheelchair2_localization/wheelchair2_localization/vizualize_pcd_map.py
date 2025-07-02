#!/usr/bin/env python3

import open3d as o3d
import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import rclpy
from rclpy.node import Node


class VizualizePCDMap(Node):

    map_topic: str = None
    downsampling_voxel_size: float = None
    pcd_map_path: str = None

    def __init__(self):
        super().__init__('vizualize_pcd_map')

        self.declare_parameter('map_topic', '/pcd_map')
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.declare_parameter('downsampling_voxel_size', 0.2)
        self.downsampling_voxel_size = self.get_parameter('downsampling_voxel_size').get_parameter_value().double_value

        self.declare_parameter('pcd_map_path', '')
        self.pcd_map_path = self.get_parameter('pcd_map_path').get_parameter_value().string_value

        self.map_publisher = self.create_publisher(
                msg_type=PointCloud2,
                topic=self.map_topic,
                qos_profile=QoSProfile(
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
        )
        self.load_pcd_map()


    def load_pcd_map(self):
            """Loads the PCD map from the specified file path."""

            try:
                self.get_logger().info(f"Loading PCD map from {self.pcd_map_path}")
                map_pcd = o3d.io.read_point_cloud(self.pcd_map_path)
                if map_pcd.is_empty():
                    self.get_logger().error(f"PCD map at {self.pcd_map_path} is empty.")
                    map_pcd = None
                else:
                    self.get_logger().info(f"Successfully loaded PCD map from {self.pcd_map_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load PCD map: {e}")

            # downsample the map for faster processing
            self.map_pcd = map_pcd.voxel_down_sample(voxel_size=self.downsampling_voxel_size)

            if self.map_pcd is not None:
                self.get_logger().info(f"Map loaded with {len(self.map_pcd.points)} points after downsampling.")
                # Publish the map point cloud for visualization

                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "map"

                map_msg = point_cloud2.create_cloud_xyz32(
                    header=header,
                    points=np.asarray(self.map_pcd.points)
                )
                self.map_publisher.publish(map_msg)

            else:
                self.get_logger().error("Failed to load or process the PCD map. Localization will not work without a valid map.")

def main(args=None):
    rclpy.init(args=args)
    node = VizualizePCDMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()