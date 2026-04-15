#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_bno055
import math
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as _I2C

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class I2C_Bus_Wrapper(_I2C):
    def try_lock(self):
        return True
    def unlock(self):
        pass

class BNO055ImuPublisher(Node):
    def __init__(self):
        super().__init__('bno055_imu_publisher')

        # ROS2 IMU publisher
        self.publisher_ = self.create_publisher(Imu, 'ada_imu/data', 10)

        # TF broadcaster (dynamic)
        self.tf_broadcaster = TransformBroadcaster(self)


        i2c_bus = I2C_Bus_Wrapper(7)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c_bus)

        # Publish at 100 Hz (per your 1.0/100.0 timer)
        self.timer = self.create_timer(1.0 / 100.0, self.publish_imu)

        self.get_logger().info("BNO055 IMU Publisher initialized on Bus 7")

    def publish_imu(self):

        quat = self.sensor.quaternion        # (x, y, z, w)
        gyro = self.sensor.gyro              # (gx, gy, gz) in rad/sec
        accel = self.sensor.linear_acceleration
        
        # Check for None values (common during sensor initialization)
        if quat[0] is None:
            return

        # -------------------- IMU MESSAGE --------------------
        msg = Imu()

        # Using your original mapping
        msg.orientation.x = quat[2]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[3]
        msg.orientation.w = quat[0]

        # Angular velocity
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # Linear acceleration
        msg.linear_acceleration.x = accel[0] 
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        # Covariance (per your original code)
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [
            0.0001, 0.0,    0.0,
            0.0,    0.0001, 0.0,
            0.0,    0.0,    0.0009
        ]

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        self.publisher_.publish(msg)

        # -------------------- DYNAMIC TF --------------------
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "imu_link"

        ## Set your IMU mounting offset here
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2   

        ## Orientation from IMU quaternion (Using your original mapping)
        t.transform.rotation.x = quat[2]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]




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
