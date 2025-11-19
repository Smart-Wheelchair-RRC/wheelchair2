#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_bno055


class BNO055ImuPublisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_publisher')

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Imu, 'ada_imu/data', 10)

        # BNO055 sensor
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Publish at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_imu)

        self.get_logger().info("BNO055 IMU Publisher started...")

    def publish_imu(self):
        quat = self.sensor.quaternion  # (x, y, z, w)

        if quat is None:
            self.get_logger().warn("Quaternion not available!")
            return

        msg = Imu()

        # ---- Fill orientation ----
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]

        # ---- Zero angular velocity ----
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # ---- Zero linear acceleration ----
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        # ---- Zero covariance matrices (required by some filters) ----
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        # ---- Header ----
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Publish
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
