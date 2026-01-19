#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from math import isnan, isinf


class SerialIMU(Node):
    def __init__(self):
        super().__init__('serial_imu_node')

        # ROS2 IMU publisher
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # Serial configuration
        self.port = '/dev/ttyUSB1' 
        self.baud = 115200

        # Line counters
        self.good_lines = 0
        self.bad_lines = 0
        self.startup_ignore = 50  # ignore first 50 lines for EKF stability

        # Connect to serial
        self.open_serial()

        # Timer for 50Hz reading
        self.timer = self.create_timer(0.02, self.read_imu)
        self.get_logger().info("✅ Serial IMU Node started → publishing /imu/data")

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"✅ Serial connected: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().fatal(f"❌ Serial open failed: {e}")
            raise

    def reconnect_serial(self):
        self.get_logger().warn("⚠️ Serial disconnected, trying reconnect...")
        try:
            self.ser.close()
        except:
            pass

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info("✅ Serial reconnected")
        except Exception as e:
            self.get_logger().error(f"❌ Reconnect failed: {e}")

    def read_imu(self):
        try:
            raw = self.ser.readline()
            if not raw:
                self.get_logger().debug("⏳ No serial data received")
                return

            try:
                line = raw.decode(errors='ignore').strip()
            except Exception as e:
                self.get_logger().warn(f"⚠️ Decode error: {e}")
                return

            self.get_logger().debug(f"📥 Raw serial: {line}")

            parts = line.split(',')

            # Expecting 10 values: qx,qy,qz,qw,gx,gy,gz,ax,ay,az
            if len(parts) != 10:
                self.bad_lines += 1
                self.get_logger().warn(
                    f"⚠️ Invalid data length ({len(parts)}): {line} | bad={self.bad_lines}"
                )
                return

            try:
                qx, qy, qz, qw, gx, gy, gz, ax, ay, az = map(float, parts)
            except ValueError:
                self.bad_lines += 1
                self.get_logger().warn(f"⚠️ Float conversion failed: {line}")
                return

            # Ignore startup lines
            if self.good_lines < self.startup_ignore:
                self.good_lines += 1
                self.get_logger().info(
                    f"⏳ Startup settling ({self.good_lines}/{self.startup_ignore})"
                )
                return

            # Validate quaternion
            if any(isnan(v) or isinf(v) for v in [qx, qy, qz, qw]):
                self.get_logger().warn("⚠️ Invalid quaternion, skipping")
                return

            # Fill ROS2 IMU message
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = "base_footprint"

            # Orientation from BNO055 (direct quaternion)
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            # Angular velocity (rad/s)
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz

            # Linear acceleration (m/s^2)
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az

            # Covariances (same as Jetson code)
            imu.orientation_covariance = [
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
            imu.angular_velocity_covariance = [
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
            imu.linear_acceleration_covariance = [
                0.0001, 0.0,    0.0,
                0.0,    0.0001, 0.0,
                0.0,    0.0,    0.0009
            ]

            self.pub.publish(imu)
            self.good_lines += 1

            self.get_logger().info(
                f"✅ IMU published | good={self.good_lines} bad={self.bad_lines}"
            )

        except Exception as e:
            self.get_logger().error(f"🔥 Runtime error: {e}")
            if "serial" in str(e).lower():
                self.reconnect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = SerialIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
