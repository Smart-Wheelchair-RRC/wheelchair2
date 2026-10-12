import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import time
import math
import threading
from collections import deque

class ElevatorArucoFloorDetector(Node):
    def __init__(self):
        super().__init__('elevator_aruco_floor_detector')

        # --- PARAMETERS & THRESHOLDS ---
        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.CAB_MAX_RADIUS = 2.0         # Robot enclosed inside elevator (m)
        self.DOOR_OPEN_DIST = 1.35        # Ray confirming door open (m)
        self.EXIT_DIST_THRESHOLD = 0.45   # Distance (m) driven to confirm exit
        self.Z_THRESHOLD = 0.020          # Vertical deviation (g's)
        self.JOLT_DURATION_REQ = 0.35     # Sustained acceleration duration (s)
        self.MAX_STATIONARY_VEL = 0.05   # Wheelchair max speed when riding (m/s)

        # --- SENSING BUFFERS ---
        self.imu_buffer = deque(maxlen=60) # ~300ms window at 200Hz
        self.calib_buffer = []
        self.calibrated_gravity = None

        # --- LIVE METRICS ---
        self.is_enclosed_in_cab = False
        self.current_horizontal_speed = 0.0

        # --- STATE MACHINE ---
        self.state = "IDLE"
        self.jolt_candidate_time = None
        self.start_odom = None
        self.last_published_floor = None
        self.is_exiting = False
        self.is_actively_scanning = False

        # --- VISION SETUP (Dedicated DICT_4X4_50 with Error Tuning) ---
        self.bridge = CvBridge()
        self.detected_aruco_floor = None
        self.frame_consensus_history = deque(maxlen=2)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_legacy_aruco = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.use_legacy_aruco = False

        # Stricter error correction to prevent rotated bit alias like 37
        self.aruco_params.errorCorrectionRate = 0.6
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10

        if not self.use_legacy_aruco:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # --- ROS 2 INTERFACES ---
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        
        self.floor_id_pub = self.create_publisher(Int32, '/detected_floor_id', 10)

        self.get_logger().info("🛗 Tuned DICT_4X4_50 ArUco + IMU Detector Online.")

    def imu_cb(self, msg):
        accel_z = msg.linear_acceleration.z

        # 1. Zero calibration
        if self.calibrated_gravity is None:
            self.calib_buffer.append(accel_z)
            if len(self.calib_buffer) >= 100:
                self.calibrated_gravity = sum(self.calib_buffer) / len(self.calib_buffer)
                self.get_logger().info(f"✅ Baseline Calibrated: {self.calibrated_gravity:.4f} g.")
            return

        self.imu_buffer.append(accel_z)
        if len(self.imu_buffer) < self.imu_buffer.maxlen:
            return

        # 2. Enclosure & Speed Guard
        if self.state in ["IDLE", "DOORS_OPEN"]:
            if not self.is_enclosed_in_cab:
                self.jolt_candidate_time = None
                return

            if self.current_horizontal_speed > self.MAX_STATIONARY_VEL:
                self.jolt_candidate_time = None
                return

        # 3. Detect vertical acceleration
        avg_z = sum(self.imu_buffer) / len(self.imu_buffer)
        accel_diff = avg_z - self.calibrated_gravity

        if abs(accel_diff) > self.Z_THRESHOLD:
            if self.jolt_candidate_time is None:
                self.jolt_candidate_time = time.time()
            elif (time.time() - self.jolt_candidate_time) > self.JOLT_DURATION_REQ:
                if self.state in ["IDLE", "DOORS_OPEN"]:
                    direction = "UP" if accel_diff > 0 else "DOWN"
                    self.get_logger().info(f"🚀 Sustained Motion: {direction} (Δ={accel_diff:+.3f}g). IN_TRANSIT.")
                    self.state = "IN_TRANSIT"
                    self.start_odom = None
                    self.detected_aruco_floor = None
                    self.is_exiting = False
                    self.is_actively_scanning = True
                    self.frame_consensus_history.clear()
        else:
            self.jolt_candidate_time = None

    def scan_cb(self, msg):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        total_rays = len(msg.ranges)

        if angle_inc == 0 or total_rays == 0:
            return

        valid_ranges = []
        front_pts = []
        back_pts = []
        cone_rad = math.radians(30)

        for i, r in enumerate(msg.ranges):
            if r < 0.1 or r > 15.0 or math.isinf(r) or math.isnan(r):
                continue

            valid_ranges.append(r)
            angle = angle_min + (i * angle_inc)
            norm_angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(norm_angle) < cone_rad:
                front_pts.append(r)
            elif abs(norm_angle) > (math.pi - cone_rad):
                back_pts.append(r)

        if valid_ranges:
            sorted_ranges = sorted(valid_ranges)
            p90_range = sorted_ranges[int(len(sorted_ranges) * 0.90)]
            self.is_enclosed_in_cab = (p90_range < self.CAB_MAX_RADIUS)
        else:
            self.is_enclosed_in_cab = False

        front_med = sorted(front_pts)[len(front_pts)//2] if front_pts else 0.0
        back_med = sorted(back_pts)[len(back_pts)//2] if back_pts else 0.0
        doors_are_open = (front_med > self.DOOR_OPEN_DIST or back_med > self.DOOR_OPEN_DIST)

        # Transition: In Transit -> Doors Open
        if self.state == "IN_TRANSIT" and doors_are_open:
            self.state = "DOORS_OPEN"
            self.is_actively_scanning = True
            self.start_odom = None
            self.get_logger().info(f"🚪 Doors Opened (Front: {front_med:.2f}m, Back: {back_med:.2f}m). Awaiting drive-out...")

    def image_cb(self, msg):
        if not self.is_actively_scanning and self.state not in ["DOORS_OPEN", "IN_TRANSIT"]:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.use_legacy_aruco:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
            else:
                corners, ids, _ = self.detector.detectMarkers(frame)

            if ids is not None and len(ids) > 0:
                raw_id = int(ids[0][0])
                
                # Reject IDs outside standard building floor range (0-9)
                if 0 <= raw_id <= 9:
                    self.frame_consensus_history.append(raw_id)
                    
                    # Require 2 consecutive frames agreeing on ID
                    if len(self.frame_consensus_history) == 2 and (self.frame_consensus_history[0] == self.frame_consensus_history[1]):
                        confirmed_floor = self.frame_consensus_history[0]
                        
                        if self.detected_aruco_floor != confirmed_floor:
                            self.detected_aruco_floor = confirmed_floor
                            self.get_logger().info(f"👁️ ArUco Tag Locked (DICT_4X4_50): Floor {self.detected_aruco_floor}")
                            
                            # Late detection after exit prompt appeared
                            if self.is_exiting and self.last_published_floor != confirmed_floor:
                                self.publish_floor(confirmed_floor)
                                self.last_published_floor = confirmed_floor
                                print(f"\n🎯 Late ArUco Lock: AUTO-PUBLISHED FLOOR {confirmed_floor}!")
                                self.is_actively_scanning = False
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_horizontal_speed = math.hypot(vx, vy)

        if self.state != "DOORS_OPEN" or self.is_exiting:
            return

        pos = msg.pose.pose.position

        # Anchor starting position at door opening
        if self.start_odom is None:
            self.start_odom = (pos.x, pos.y)
            return

        dist = math.hypot(pos.x - self.start_odom[0], pos.y - self.start_odom[1])
        if dist > self.EXIT_DIST_THRESHOLD:
            self.is_exiting = True
            self.is_actively_scanning = True
            self.get_logger().info(f"📍 Wheelchair drove {dist:.2f}m out of elevator cab.")
            threading.Thread(target=self.process_exit_sequence, daemon=True).start()

    def process_exit_sequence(self):
        # 0.8s settling window
        start_wait = time.time()
        while (time.time() - start_wait < 0.8) and (self.detected_aruco_floor is None):
            time.sleep(0.1)

        auto_floor = self.detected_aruco_floor

        if auto_floor is not None:
            self.publish_floor(auto_floor)
            self.last_published_floor = auto_floor
            self.is_actively_scanning = False
            self.get_logger().info(f"⚡ Successfully auto-published Floor {auto_floor} to navigation stack.")
        else:
            self.get_logger().warn("⚠️ No ArUco marker visible yet. Camera scanning remains ACTIVE while awaiting input...")

        self.reset_detector_state()
        threading.Thread(target=self.terminal_override_thread, args=(auto_floor,), daemon=True).start()

    def terminal_override_thread(self, published_floor):
        print("\n" + "="*60)
        print("  🛗 ROBOT EXITED ELEVATOR CAB")
        print("="*60)

        if published_floor is not None:
            print(f"🎯 Auto-published: FLOOR {published_floor}")
            print(">>> If WRONG, enter correct floor [0-9] to override, or ignore:")
        else:
            print("⚠️ No ArUco marker detected yet (Camera scanning continuously...)")
            print(">>> Enter Current Floor Number [0-9] (or let camera find marker): ")

        try:
            user_input = input().strip()
            if user_input.isdigit():
                entered_floor = int(user_input)
                self.is_actively_scanning = False
                if published_floor is None:
                    self.get_logger().info(f"📢 Manual floor confirmed: Publishing Floor {entered_floor}...")
                    self.publish_floor(entered_floor)
                    self.last_published_floor = entered_floor
                elif entered_floor != published_floor:
                    self.get_logger().info(f"🔁 Override: Republishing Floor {entered_floor} to /detected_floor_id...")
                    self.publish_floor(entered_floor)
                    self.last_published_floor = entered_floor
                else:
                    self.get_logger().info(f"Confirmed Floor {entered_floor}.")
        except Exception:
            pass

    def publish_floor(self, floor_num):
        fid_msg = Int32()
        fid_msg.data = int(floor_num)
        self.floor_id_pub.publish(fid_msg)
        self.get_logger().info(f"📢 Published Floor ID '{floor_num}' to /detected_floor_id.")

    def reset_detector_state(self):
        self.state = "IDLE"
        self.start_odom = None
        self.jolt_candidate_time = None
        self.is_exiting = False
        self.imu_buffer.clear()
        self.get_logger().info("Detector state returned to IDLE. (Background camera search active if unresolved).")

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorArucoFloorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()