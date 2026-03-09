#!/usr/bin/env python3
"""
Republish the saved initial pose estimate on /initialpose for the ICP node.

This script is launched (with a delay) after ICP starts, so the ICP node's
subscriber on /initialpose is ready to receive the pose. It reads the pose
saved by wait_for_initial_pose.py and publishes it, then exits.
"""

import json
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

POSE_FILE = "/tmp/wheelchair2_initial_pose.json"


class InitialPoseRepublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_republisher")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        # Use a one-shot timer to allow the publisher to be fully set up
        self._publish_count = 0
        self._max_publishes = 3
        self._timer = self.create_timer(1.0, self._publish_pose)

    def _publish_pose(self):
        try:
            with open(POSE_FILE, "r") as f:
                pose_data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Failed to read saved pose: {e}")
            raise SystemExit(1)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose_data.get("frame_id", "map")
        msg.pose.pose.position.x = pose_data["x"]
        msg.pose.pose.position.y = pose_data["y"]
        msg.pose.pose.position.z = pose_data["z"]
        msg.pose.pose.orientation.x = pose_data["qx"]
        msg.pose.pose.orientation.y = pose_data["qy"]
        msg.pose.pose.orientation.z = pose_data["qz"]
        msg.pose.pose.orientation.w = pose_data["qw"]

        self.publisher.publish(msg)
        self._publish_count += 1
        self.get_logger().info(
            f"Republished initial pose ({self._publish_count}/{self._max_publishes}): "
            f"x={pose_data['x']:.3f}, y={pose_data['y']:.3f}"
        )

        if self._publish_count >= self._max_publishes:
            self._timer.cancel()
            self.get_logger().info("Done republishing initial pose. Exiting.")
            raise SystemExit(0)


def main():
    rclpy.init()
    node = InitialPoseRepublisher()
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
