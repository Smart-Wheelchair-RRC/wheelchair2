#!/usr/bin/env python3
"""
ROS2 node that:
 - subscribes to a PointCloud2 (LiDAR) in LiDAR frame
 - Reflects points with z > 0 off a virtual conical mirror
 - Publishes modified PointCloud2
 - Publishes a Marker for debugging
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
import struct
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

# In ROS 2, point_cloud2 functions are in the sensor_msgs_py library
# You may need to install it: sudo apt install ros-<distro>-sensor-msgs-py
from sensor_msgs_py import point_cloud2 as pc2


def pack_color(r, g, b):
    """Packs RGB into a single float for PointCloud2."""
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
    packed = struct.pack("I", rgb_int)
    return struct.unpack("f", packed)[0]


class ConicalMirrorReflector(Node):
    def __init__(self):
        super().__init__('conical_mirror_reflector')

        # 1. Declare Parameters (ROS 2 requires declaration before retrieval)
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/mirror/points')
        self.declare_parameter('mirror_angle_deg', 27.5)
        self.declare_parameter('mirror_base_radius', 0.1)
        self.declare_parameter('cone_direction', 'down')
        self.declare_parameter('max_intersect_dist', 50.0)

        # 2. Get Parameters
        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        self.alpha_deg = self.get_parameter('mirror_angle_deg').value
        self.r0 = self.get_parameter('mirror_base_radius').value
        self.cone_dir = self.get_parameter('cone_direction').value
        self.max_dist = self.get_parameter('max_intersect_dist').value

        # 3. Math setup
        self.alpha = math.radians(self.alpha_deg)
        self.k = math.tan(self.alpha)
        if self.cone_dir == "down":
            self.k = -abs(self.k)
        else:
            self.k = abs(self.k)

        # 4. Publishers & Subscribers
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)
        
        # QoS for Latching behavior (Transient Local)
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, '~/mirror_marker', latch_qos)
        
        self.create_subscription(PointCloud2, self.in_topic, self.cloud_cb, 10)

        self.get_logger().info("ConicalMirrorReflector: Reflecting all points with z > 0.")
        self.get_logger().info(f"Params: alpha={self.alpha_deg:.3f} deg, r0={self.r0:.3f}m, cone_dir={self.cone_dir}")

        # Pre-pack colors
        self.color_red = pack_color(255, 0, 0)
        self.color_white = pack_color(200, 200, 200)

        # 5. Schedule marker publication (don't sleep in __init__)
        self.create_timer(0.5, self.publish_cone_marker_once)
        self._marker_published = False

    def publish_cone_marker_once(self):
        if not self._marker_published:
            self.publish_cone_marker()
            self._marker_published = True

    def publish_cone_marker(self):
        m = Marker()
        m.header.frame_id = "base_link" # Make sure this frame exists or change to lidar frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "conical_mirror"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        
        vis_height = 0.1
        top_r = self.r0 + vis_height * abs(math.tan(self.alpha))
        
        m.scale.x = 2.0 * top_r
        m.scale.y = 2.0 * top_r
        m.scale.z = vis_height
        
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = vis_height / 2.0
        m.pose.orientation.w = 1.0
        
        m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.3)
        
        self.marker_pub.publish(m)
        self.get_logger().info("Published cone marker (approx).")

    def intersect_cone(self, d):
        dx, dy, dz = d[0], d[1], d[2]
        s = math.hypot(dx, dy)
        denom = dz - s * self.k
        if abs(denom) < 1e-12:
            return None
        t = (-self.r0 * self.k) / denom
        if t <= 1e-6 or t > self.max_dist:
            return None
        return t * d

    def local_normal(self, p):
        x, y, _ = p
        r = math.hypot(x, y)
        if r < 1e-9:
            return np.array([0.0, 0.0, 1.0]) if self.k > 0 else np.array([0.0, 0.0, -1.0])
        nx = -self.k * (x / r)
        ny = -self.k * (y / r)
        nz = 1.0
        n = np.array([nx, ny, nz])
        return n / np.linalg.norm(n)

    def reflect_vec(self, incoming_dir, normal):
        return incoming_dir - 2.0 * np.dot(incoming_dir, normal) * normal

    def cloud_cb(self, msg):
        # pc2.read_points returns a generator
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        out_pts_with_color = []

        for p in pts:
            # pts usually comes as tuples (x, y, z) or named tuples
            x, y, z = p[0], p[1], p[2]

            if z > -0.05:
                # Reflection logic
                total_dist = math.sqrt(x * x + y * y + z * z)
                if total_dist < 1e-6:
                    continue

                d = np.array([x / total_dist, y / total_dist, z / total_dist])
                p_int = self.intersect_cone(d)
                if p_int is None:
                    continue

                dist_to_mirror = np.linalg.norm(p_int)
                dist_remaining = total_dist - dist_to_mirror
                if dist_remaining < 0:
                    continue

                n = self.local_normal(p_int)
                d_ref = self.reflect_vec(d, n)
                p_actual = p_int + d_ref * dist_remaining

                # Append RED point
                pt_data = [float(p_actual[0]), float(p_actual[1]), float(p_actual[2]), self.color_red]
                out_pts_with_color.append(pt_data)
            else:
                # Direct view (White/Gray)
                pt_data = [float(x), float(y), float(z), self.color_white]
                out_pts_with_color.append(pt_data)

        # Create output header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # In sensor_msgs_py, create_cloud takes (header, fields, points) similar to ROS1
        pc2_msg = pc2.create_cloud(header, fields, out_pts_with_color)
        self.pub.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConicalMirrorReflector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()