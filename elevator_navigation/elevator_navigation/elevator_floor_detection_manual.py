import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import time
import math
import threading
from collections import deque

class ElevatorFloorDetectionTerminal(Node):
    def __init__(self):
        super().__init__('elevator_floor_detection_terminal')

        # --- PARAMETERS & THRESHOLDS ---
        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.CAB_MAX_RADIUS = 2.0         # Robot enclosed inside elevator (m)
        self.DOOR_OPEN_DIST = 1.4         # Single-side ray confirming door open (m)
        self.HALLWAY_CLEARANCE_DIST = 2.2 # Reading confirming robot has driven into open hallway (m)
        self.EXIT_DIST_THRESHOLD = 0.45   # Distance driven (m) after door opens to trigger prompt
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
        # IDLE -> IN_TRANSIT -> DOORS_OPEN -> PROMPTING
        self.state = "IDLE"
        self.jolt_candidate_time = None
        self.door_opened_time = None
        self.start_odom = None
        self.prompt_active = False

        # --- ROS 2 INTERFACES ---
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.floor_id_pub = self.create_publisher(Int32, '/detected_floor_id', 10)

        self.get_logger().info(f"🛗 Elevator Floor Detector online. Odom: '{self.odom_topic}'")

    def imu_cb(self, msg):
        accel_z = msg.linear_acceleration.z

        # 1. Zero-calibration
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
        if self.state in ["IDLE", "IN_CAB_READY", "DOORS_OPEN"]:
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
                if self.state in ["IDLE", "IN_CAB_READY", "DOORS_OPEN"]:
                    direction = "UP" if accel_diff > 0 else "DOWN"
                    self.get_logger().info(f"🚀 Sustained Motion: {direction} (Δ={accel_diff:+.3f}g). IN_TRANSIT.")
                    self.state = "IN_TRANSIT"
                    self.start_odom = None
                    self.door_opened_time = None
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

        if self.prompt_active:
            return

        front_med = sorted(front_pts)[len(front_pts)//2] if front_pts else 0.0
        back_med = sorted(back_pts)[len(back_pts)//2] if back_pts else 0.0
        doors_are_open = (front_med > self.DOOR_OPEN_DIST or back_med > self.DOOR_OPEN_DIST)

        # Transition 1: In Transit -> Doors Open
        if self.state == "IN_TRANSIT":
            if doors_are_open:
                self.state = "DOORS_OPEN"
                self.door_opened_time = time.time()
                self.get_logger().info(f"🚪 Doors Opened (Front: {front_med:.2f}m, Back: {back_med:.2f}m). Drive out or stay inside.")

        # Transition 2: When doors are open, check if user exited or stayed inside
        elif self.state == "DOORS_OPEN":
            # Fallback A: Either side shows substantial hallway clearance
            if (front_med > self.HALLWAY_CLEARANCE_DIST and back_med > self.HALLWAY_CLEARANCE_DIST) or \
               (front_med > 3.0 or back_med > 3.0):
                self.get_logger().info(f"📍 Clear hallway detected (Front: {front_med:.2f}m, Back: {back_med:.2f}m). Triggering prompt.")
                self.trigger_prompt()
                return

            # Fallback B: If doors closed AND we waited at least 5 seconds without moving out
            if not doors_are_open and self.is_enclosed_in_cab:
                if self.door_opened_time and (time.time() - self.door_opened_time > 5.0):
                    self.state = "IN_CAB_READY"
                    self.start_odom = None
                    self.get_logger().info("🔒 Doors Closed. Still inside cab. Waiting for next movement...")

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_horizontal_speed = math.hypot(vx, vy)

        if self.state != "DOORS_OPEN" or self.prompt_active:
            return

        pos = msg.pose.pose.position

        # Anchor odom starting position at door opening
        if self.start_odom is None:
            self.start_odom = (pos.x, pos.y)
            return

        dist = math.hypot(pos.x - self.start_odom[0], pos.y - self.start_odom[1])
        if dist > self.EXIT_DIST_THRESHOLD:
            self.get_logger().info(f"📍 Wheelchair drove {dist:.2f}m out of elevator cab.")
            self.trigger_prompt()

    def trigger_prompt(self):
        if self.prompt_active:
            return
        self.prompt_active = True
        self.state = "PROMPTING"
        threading.Thread(target=self.terminal_prompt_thread, daemon=True).start()

    def terminal_prompt_thread(self):
        print("\n" + "="*50)
        print("  🛗 ROBOT EXITED ELEVATOR CAB")
        print("="*50)

        while self.prompt_active:
            try:
                user_input = input(">>> Enter Current Floor Number [0-9 or 'q']: ").strip()
                if user_input.lower() == 'q':
                    self.get_logger().warn("Cancelled by user.")
                    self.finalize(None)
                    break

                entered_floor = int(user_input)
                self.get_logger().info(f"Confirmed arrival on Floor {entered_floor}.")
                self.finalize(entered_floor)
                break
            except ValueError:
                print("Invalid integer. Please enter e.g. 0, 4 or 'q'.")
            except EOFError:
                break

    def finalize(self, floor_num):
        if floor_num is not None:
            fid_msg = Int32()
            fid_msg.data = floor_num
            self.floor_id_pub.publish(fid_msg)
            self.get_logger().info(f"📢 Published Floor ID '{floor_num}' to /detected_floor_id.")

        # Reset state machine
        self.state = "IDLE"
        self.start_odom = None
        self.door_opened_time = None
        self.prompt_active = False
        self.jolt_candidate_time = None
        self.imu_buffer.clear()
        self.get_logger().info("Detector returned to IDLE. Ready for next elevator cycle.")

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFloorDetectionTerminal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()