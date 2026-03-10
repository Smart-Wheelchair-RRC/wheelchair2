#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
import numpy as np

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class LivoxBoxFilter(Node):
    def __init__(self):
        super().__init__('livox_box_filter')

        # Box Dimensions (Meters)
        self.dx = 0.5  # 5cm long
        self.dy = 0.5  # 5cm wide
        self.dz = 1.0  # 30cm high
        self.frame_id = "livox_frame"

        # Publishers & Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, qos_profile=qos_profile)
        
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/livox/lidar_box', qos_profile=qos_profile)
        
        # New Marker Publisher for RViz
        self.marker_pub = self.create_publisher(Marker, '/filter_box_visual', 10)
            
        self.get_logger().info("Livox Box Filter with Visualizer started")

        # Timer to publish the box visualization at 1Hz (it doesn't need to be fast)
        self.timer = self.create_timer(1.0, self.publish_box_marker)

    def lidar_callback(self, msg: PointCloud2):
        pc_data = point_cloud2.read_points(msg, skip_nans=True)

        # Erase logic (Masking points INSIDE the box)
        mask = (
            (pc_data['x'] >= -self.dx/2) & (pc_data['x'] <= self.dx/2) &
            (pc_data['y'] >= -self.dy/2) & (pc_data['y'] <= self.dy/2) &
            (pc_data['z'] >= -self.dz/2) & (pc_data['z'] <= self.dz/2)
        )

        filtered_data = pc_data[~mask]

        processed_message = point_cloud2.create_cloud(
            msg.header, 
            msg.fields, 
            filtered_data
        )
        self.lidar_pub.publish(processed_message)

    def publish_box_marker(self):
        """Publishes a semi-transparent cube to RViz to show the filter area."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filter_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position (Centered at origin)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Scale (Matches your filter dimensions)
        marker.scale.x = self.dx
        marker.scale.y = self.dy
        marker.scale.z = self.dz

        # Color: Semi-transparent Red (R, G, B, Alpha)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.4  # 40% opacity

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = LivoxBoxFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()