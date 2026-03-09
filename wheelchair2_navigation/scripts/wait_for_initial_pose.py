#!/usr/bin/env python3
"""
Wait for an initial pose estimate on /initialpose, save it to a temp file, and exit.

This script is used by the launch system to gate the startup of ICP and DLIO nodes
until the user provides an initial pose estimate (e.g., via RViz "2D Pose Estimate").
When the process exits (code 0), the launch event handler triggers ICP/DLIO startup.
"""

import json
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

POSE_FILE = "/tmp/wheelchair2_initial_pose.json"


class InitialPoseWaiter(Node):
    def __init__(self):
        super().__init__("initial_pose_waiter")
        self.get_logger().info(
            "Waiting for initial pose estimate on /initialpose ... "
            "(use RViz '2D Pose Estimate' to provide one)"
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._pose_callback,
            1,
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        pose_data = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
            "frame_id": msg.header.frame_id,
        }
        with open(POSE_FILE, "w") as f:
            json.dump(pose_data, f)

        self.get_logger().info(
            f"Initial pose received and saved: "
            f"x={pose_data['x']:.3f}, y={pose_data['y']:.3f}, z={pose_data['z']:.3f}"
        )
        raise SystemExit(0)


def main():
    rclpy.init()
    node = InitialPoseWaiter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
