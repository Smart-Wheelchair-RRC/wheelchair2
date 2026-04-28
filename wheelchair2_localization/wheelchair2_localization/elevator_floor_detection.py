import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import subprocess
import time
import math

class ElevatorFloorDetection(Node):
    def __init__(self):
        super().__init__('elevator_floor_detection')
        
        # --- Config ---
        self.TARGET_FLOOR = 4
        self.LIFT_INTERIOR_LIMIT = 1.6 
        self.EXIT_DIST = 1.2
        self.IMU_GRAVITY = 1.0 
        self.Z_THRESHOLD = 0.05 
        self.JOLT_DURATION_REQ = 0.3 # Seconds the acceleration must be sustained
        
        # --- Filtering ---
        self.imu_buffer = []
        self.buffer_size = 15 

        # --- State ---
        self.state = "IDLE" 
        self.move_candidate_time = None
        self.move_start_time = None
        self.start_odom = None
        self.target_found = False
        self.camera_process = None
        self.bridge = CvBridge()

        # --- Comms ---
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.floor_id_pub = self.create_publisher(Int32, '/detected_floor_id', 10)

        self.get_logger().info("Calibrated for Gravity=1.0. Improved Jolt Filtering Active.")

    def imu_cb(self, msg):
        self.imu_buffer.append(msg.linear_acceleration.z)
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.pop(0)
        
        avg_z = sum(self.imu_buffer) / len(self.imu_buffer)
        accel_diff = avg_z - self.IMU_GRAVITY

        # --- IMPROVED JOLT FILTERING ---
        if abs(accel_diff) > self.Z_THRESHOLD:
            if self.move_candidate_time is None:
                self.move_candidate_time = time.time()
            
            # Check if jolt is sustained (Lift) or temporary (Mat/Bump)
            elif (time.time() - self.move_candidate_time) > self.JOLT_DURATION_REQ:
                if self.state == "IDLE":
                    self.get_logger().info(f"Sustained motion: {'UP' if accel_diff > 0 else 'DOWN'}. Heating camera.")
                    self.move_start_time = time.time()
                    self.state = "MOVING"
                    self.start_camera()
                elif self.state == "SEARCHING":
                    self.get_logger().info("Lift moving again. Resetting search.")
                    self.state = "MOVING"
                    self.move_start_time = time.time()
                    self.target_found = False
        else:
            self.move_candidate_time = None # Reset candidate if acceleration drops

    def scan_cb(self, msg):
        if self.state not in ["MOVING", "SEARCHING"]: return
        if self.move_start_time and (time.time() - self.move_start_time) < 1.0: return

        num_readings = len(msg.ranges)
        
        # --- DUAL SECTOR DETECTION ---
        # Front 60 (indices ~330-30), Back 60 (indices ~150-210)
        front_indices = list(range(0, 30)) + list(range(num_readings-30, num_readings))
        back_indices = list(range(150, 210))
        
        f_pts = [msg.ranges[i] for i in front_indices if 0.1 < msg.ranges[i] < 10.0]
        b_pts = [msg.ranges[i] for i in back_indices if 0.1 < msg.ranges[i] < 10.0]

        f_dist = sorted(f_pts)[len(f_pts)//2] if f_pts else 0.0
        b_dist = sorted(b_pts)[len(b_pts)//2] if b_pts else 0.0

        front_open = f_dist > self.LIFT_INTERIOR_LIMIT
        back_open = b_dist > self.LIFT_INTERIOR_LIMIT

        # Logic for state transition
        if self.state == "MOVING" and (front_open or back_open):
            side = "FRONT" if front_open else "BACK"
            self.get_logger().info(f"{side} door opened. Median Dist: {max(f_dist, b_dist):.2f}m")
            self.state = "SEARCHING"
            self.start_odom = None

        # Door Close Logic: Only revert if BOTH doors are closed
        elif self.state == "SEARCHING" and not front_open and not back_open:
            self.get_logger().info("Both doors closed. Still inside.")
            self.state = "MOVING"
            self.target_found = False

    def start_camera(self):
        if self.camera_process is None:
            self.camera_process = subprocess.Popen(["ros2", "launch", "realsense2_camera", "rs_launch.py"])
            self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)

    def image_cb(self, msg):
        if self.state != "SEARCHING": return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        dict_aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        corners, ids, _ = cv2.aruco.detectMarkers(frame, dict_aruco)
        if ids is not None:
            detected_id = int(ids[0][0])
            if detected_id == self.TARGET_FLOOR:
                self.target_found = True

    def odom_cb(self, msg):
        if self.state == "SEARCHING":
            pos = msg.pose.pose.position
            if self.start_odom is None:
                self.start_odom = pos
                return
            dist = math.sqrt((pos.x - self.start_odom.x)**2 + (pos.y - self.start_odom.y)**2)
            if dist > self.EXIT_DIST:
                self.finalize_localization()

    def finalize_localization(self):
        if self.target_found:
            fid_msg = Int32()
            fid_msg.data = self.TARGET_FLOOR
            self.floor_id_pub.publish(fid_msg)
            self.publish_pose(self.TARGET_FLOOR)
            self.reset("DONE")
        else:
            self.get_logger().warn("Exited but no marker match.")
            self.reset("IDLE")

    def publish_pose(self, floor_id):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 15.5 
        msg.pose.pose.position.y = -2.2
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.01] * 36
        self.pose_pub.publish(msg)

    def reset(self, next_state):
        if self.camera_process:
            self.camera_process.terminate()
            self.camera_process = None
        if hasattr(self, 'img_sub'): self.destroy_subscription(self.img_sub)
        self.state = next_state
        self.start_odom = None
        self.target_found = False

def main():
    rclpy.init()
    node = ElevatorFloorDetection()
    rclpy.spin(node)
    rclpy.shutdown()