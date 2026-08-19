import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import time
import math
import threading

class ElevatorFloorDetectionTerminal(Node):
    def __init__(self):
        super().__init__('elevator_floor_detection_terminal')
        
        # --- Config ---
        self.LIFT_INTERIOR_LIMIT = 1.6
        self.EXIT_DIST = 1.2
        
        # NOTE: Ensure your Livox IMU outputs in 'g's (1.0). 
        # Standard ROS 2 REP-145 dictates m/s^2 (~9.81). Adjust this if your sensor uses 9.81!
        self.IMU_GRAVITY = 1.0
        
        self.Z_THRESHOLD = 0.05
        self.JOLT_DURATION_REQ = 0.3

        self.imu_buffer = []
        self.buffer_size = 15

        # --- State ---
        self.state = "IDLE" 
        self.move_candidate_time = None
        self.move_start_time = None
        self.start_odom = None
        self.target_found = False
        self.prompt_thread_active = False

        # --- Comms ---
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.floor_id_pub = self.create_publisher(Int32, '/detected_floor_id', 10)

        self.get_logger().info("Elevator Terminal Sensor active. Waiting for elevator movement...")

    def imu_cb(self, msg):
        self.imu_buffer.append(msg.linear_acceleration.z)
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.pop(0)
        
        avg_z = sum(self.imu_buffer) / len(self.imu_buffer)
        accel_diff = avg_z - self.IMU_GRAVITY

        if abs(accel_diff) > self.Z_THRESHOLD:
            if self.move_candidate_time is None:
                self.move_candidate_time = time.time()
            elif (time.time() - self.move_candidate_time) > self.JOLT_DURATION_REQ:
                if self.state == "IDLE":
                    self.get_logger().info(f"Sustained motion: {'UP' if accel_diff > 0 else 'DOWN'}. Elevator in transit!")
                    self.move_start_time = time.time()
                    self.state = "MOVING"
                elif self.state == "SEARCHING":
                    self.get_logger().info("Lift moving again. Resetting search.")
                    self.state = "MOVING"
                    self.move_start_time = time.time()
                    self.target_found = False
        else:
            self.move_candidate_time = None 

    def scan_cb(self, msg):
        if self.state not in ["MOVING", "SEARCHING"]: return
        if self.move_start_time and (time.time() - self.move_start_time) < 1.0: return

        num_readings = len(msg.ranges)
        front_indices = list(range(0, 30)) + list(range(num_readings-30, num_readings))
        back_indices = list(range(150, 210))
        
        f_pts = [msg.ranges[i] for i in front_indices if 0.1 < msg.ranges[i] < 10.0]
        b_pts = [msg.ranges[i] for i in back_indices if 0.1 < msg.ranges[i] < 10.0]

        f_dist = sorted(f_pts)[len(f_pts)//2] if f_pts else 0.0
        b_dist = sorted(b_pts)[len(b_pts)//2] if b_pts else 0.0

        front_open = f_dist > self.LIFT_INTERIOR_LIMIT
        back_open = b_dist > self.LIFT_INTERIOR_LIMIT

        if self.state == "MOVING" and (front_open or back_open):
            self.state = "SEARCHING"
            self.start_odom = None
            self.get_logger().info("Elevator doors opened! Ready to exit.")
        elif self.state == "SEARCHING" and not front_open and not back_open:
            self.state = "MOVING"
            self.target_found = False

    def odom_cb(self, msg):
        if self.state == "SEARCHING":
            pos = msg.pose.pose.position
            if self.start_odom is None:
                self.start_odom = pos
                return
            dist = math.sqrt((pos.x - self.start_odom.x)**2 + (pos.y - self.start_odom.y)**2)
            if dist > self.EXIT_DIST:
                if not self.prompt_thread_active:
                    self.prompt_thread_active = True
                    threading.Thread(target=self.terminal_prompt_thread, daemon=True).start()

    def terminal_prompt_thread(self):
        print("\n" + "="*50)
        print("  ROBOT EXITED ELEVATOR THRESHOLD")
        print("="*50)
        
        while self.prompt_thread_active and self.state == "SEARCHING":
            try:
                user_input = input(">>> Enter detected Floor Number [or 'q' to cancel]: ").strip()
                if user_input.lower() == 'q':
                    self.get_logger().warn("User canceled floor confirmation.")
                    self.target_found = False
                    self.finalize_detection(None)
                    break
                
                entered_floor = int(user_input)
                self.get_logger().info(f"User confirmed exit on Floor {entered_floor}.")
                self.finalize_detection(entered_floor)
                break
            except ValueError:
                print("Invalid input. Please enter a numerical integer (e.g., 0, 4) or 'q'.")
            except EOFError:
                break

    def finalize_detection(self, floor_num):
        self.prompt_thread_active = False
        if floor_num is not None:
            fid_msg = Int32()
            fid_msg.data = floor_num
            self.floor_id_pub.publish(fid_msg)
            self.get_logger().info(f"Published Floor ID '{floor_num}' to /detected_floor_id.")
        else:
            self.get_logger().warn("Exited without floor confirmation.")
            
        # FIX: Must reset to IDLE so the node can handle subsequent elevator rides!
        self.reset("IDLE")

    def reset(self, next_state):
        self.state = next_state
        self.start_odom = None
        self.target_found = False
        self.prompt_thread_active = False

def main():
    rclpy.init()
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