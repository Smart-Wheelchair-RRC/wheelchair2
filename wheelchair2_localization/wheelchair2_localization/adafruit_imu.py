#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial

class BNO055SerialBridge(Node):
    def __init__(self):
        super().__init__('bno055_imu_publisher')
        
        # Using RELIABLE QoS to match standard EKF requirements
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        self.publisher_ = self.create_publisher(Imu, 'ada_imu/data', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Port updated to USB1 based on your successful hardware tests
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info("✅ BNO055 ESP32 Bridge Active on /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")

        # Timer set to 100Hz (0.01s). 
        # Note: If your CPU struggles, you can change this to 1.0/50.0
        self.create_timer(1.0 / 100.0, self.publish_imu)

    def publish_imu(self):
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                p = line.split(',')
                
                # Expecting the 11-value packet: ax,ay,az,qw,qx,qy,qz,gx,gy,gz,calib
                if len(p) == 11:
                    ax, ay, az = float(p[0]), float(p[1]), float(p[2])
                    qw, qx, qy, qz = float(p[3]), float(p[4]), float(p[5]), float(p[6])
                    gx, gy, gz = float(p[7]), float(p[8]), float(p[9])
                    calib = int(p[10])

                    if calib < 3:
                        self.get_logger().warn(f"IMU Uncalibrated: {calib}/3", throttle_duration_sec=10.0)

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu_link" 

                    # -------------------- QUATERNION MAPPING --------------------
                    # Matches your working I2C indices [quat[2], quat[1], quat[3], quat[0]]
                    msg.orientation.x = qy
                    msg.orientation.y = qx
                    msg.orientation.z = qz
                    msg.orientation.w = qw

                    # Angular velocity (Gyro)
                    msg.angular_velocity.x = gx
                    msg.angular_velocity.y = gy
                    msg.angular_velocity.z = gz
                  
                    # Linear acceleration 
                    # Forced Y/Z to 0.0 as per your working template
                    msg.linear_acceleration.x = ax
                    msg.linear_acceleration.y = 0.0 
                    msg.linear_acceleration.z = 0.0

                    # -------------------- CRITICAL COVARIANCE FIX --------------------
                    # Non-zero covariance prevents the EKF from "jumping" in RViz
                    msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                    msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                    msg.linear_acceleration_covariance = [
                        0.0001, 0.0,    0.0,
                        0.0,    0.0001, 0.0,
                        0.0,    0.0,    0.0009
                    ]

                    self.publisher_.publish(msg)

                    # -------------------- DYNAMIC TF --------------------
                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = "base_footprint"
                    t.child_frame_id = "imu_link"
                    t.transform.translation.z = 0.2 
                    t.transform.rotation = msg.orientation
                    
                    self.tf_broadcaster.sendTransform(t)

            except Exception as e:
                self.get_logger().debug(f"Parsing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()