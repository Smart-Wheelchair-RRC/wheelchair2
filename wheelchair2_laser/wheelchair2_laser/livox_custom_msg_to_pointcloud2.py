#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from livox_ros_driver2.msg import CustomMsg

class LivoxLaserToPointcloud(Node):
    def __init__(self):
        super().__init__("livox_laser_to_pointcloud")

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.pub = self.create_publisher(PointCloud2, "/livox/pcd", 10)
        self.sub = self.create_subscription(CustomMsg, "/livox/lidar", self.process_livox_custom_msg, 10)

    def process_livox_custom_msg(self, msg: CustomMsg):
        points = [[p.x, p.y, p.z, p.reflectivity] for p in msg.points]
        pointcloud_msg = point_cloud2.create_cloud(msg.header, self.fields, points)
        self.pub.publish(pointcloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxLaserToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()