import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import LoadMap
import subprocess
import time
import math
import cv2
import os
import numpy as np
import csv
from sklearn.cluster import DBSCAN

class HumanInTheLoopHealer(Node):
    def __init__(self):
        super().__init__('human_in_the_loop_healer')
        
        # --- CONFIGURATION ---
        self.THRESHOLD = 0.03        
        self.SLAM_DURATION = 6       
        self.ws_path = "/home/container_user/wheelchair2"
        self.map_base = f"{self.ws_path}/src/wheelchair2_navigation/maps/updated_lab"
        self.map_yaml = f"{self.map_base}.yaml"
        self.map_png = f"{self.map_base}.png"

        # State Variables
        self.current_pose = None
        self.is_transitioning = False
        self.start_time = time.time()
        
        # CSV Logging
        self.csv_filename = 'human_trigger_log.csv'
        with open(self.csv_filename, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Avg_Cov', 'Status', 'X', 'Y'])

        # Service Clients
        self.amcl_client = self.create_client(ChangeState, '/amcl/change_state')
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.get_logger().info(f"--- HUMAN-IN-THE-LOOP ACTIVE ---")
        self.get_logger().info(f"Threshold: {self.THRESHOLD} | Waiting for uncertainty...")

    def get_yaw(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def pose_callback(self, msg):
        if self.is_transitioning:
            return

        self.current_pose = msg.pose.pose
        
        # Calculate Average Covariance
        avg_cov = (msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[35]) / 3.0
        timestamp = round(time.time() - self.start_time, 2)

        # Trigger if Threshold is met
        if avg_cov >= self.THRESHOLD:
            self.get_logger().warn(f"UNCERTAINTY DETECTED: {avg_cov:.4f}")
            
            # This 'input' will block the callback thread until you hit Enter
            print(f"\n[!] PROMPT: High uncertainty detected at {timestamp}s.")
            user_input = input(">>> Press ENTER to trigger 6-second SLAM healing, or 's' to skip: ")
            
            if user_input.lower() != 's':
                self.is_transitioning = True
                self.run_healing_workflow()
            else:
                self.get_logger().info("User skipped the healing process.")
        
        # Periodic CSV logging
        if int(timestamp * 10) % 10 == 0: 
            self.log_to_csv(timestamp, avg_cov, "NORMAL")

    def log_to_csv(self, ts, cov, status):
        if self.current_pose:
            with open(self.csv_filename, mode='a') as f:
                writer = csv.writer(f)
                writer.writerow([ts, cov, status, self.current_pose.position.x, self.current_pose.position.y])

    def change_state(self, transition_id):
        if self.amcl_client.wait_for_service(timeout_sec=5.0):
            req = ChangeState.Request()
            req.transition.id = transition_id
            self.amcl_client.call_async(req)
            time.sleep(2.0)

    def filter_map_noise(self, image_path):
        """DBSCAN Map Filtering"""
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None: return
        filtered = image.copy()
        
        black_coords = np.argwhere(image == 0)
        if len(black_coords) > 0:
            labels = DBSCAN(eps=3, min_samples=7).fit_predict(black_coords)
            for row, col in black_coords[labels == -1]:
                filtered[row, col] = 255
        
        cv2.imwrite(image_path, filtered)
        self.get_logger().info("DBSCAN: Filtering complete.")

    def run_healing_workflow(self):
        slam_proc = None
        try:
            x, y = self.current_pose.position.x, self.current_pose.position.y
            yaw = self.get_yaw(self.current_pose.orientation)
            pose_str = f"[{x}, {y}, {yaw}]"

            # 1. Deactivate AMCL
            self.change_state(Transition.TRANSITION_DEACTIVATE)

            # 2. Start SLAM
            self.get_logger().info(f"Launching SLAM at {pose_str}...")
            bash_cmd = f"source {self.ws_path}/install/setup.bash && " \
                       f"ros2 launch wheelchair2_navigation online_async_launch.py " \
                       f"use_sim_time:=True map_start_pose:='{pose_str}'"
            slam_proc = subprocess.Popen(['bash', '-c', bash_cmd])

            # 3. 6-Second SLAM window
            self.get_logger().info("SLAM active. Updating map...")
            time.sleep(self.SLAM_DURATION)

            # 4. Save and Filter
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
                '-f', self.map_base, '--fmt', 'png', '--ros-args', '-p', 'use_sim_time:=True'
            ])
            time.sleep(1.0)
            self.filter_map_noise(self.map_png)

        except Exception as e:
            self.get_logger().error(f"Workflow Error: {e}")
        finally:
            if slam_proc:
                slam_proc.terminate()
                slam_proc.wait()
            
            # 5. Resume AMCL
            self.change_state(Transition.TRANSITION_ACTIVATE)
            
            # 6. Refresh Map Server
            if self.map_load_client.wait_for_service(timeout_sec=5.0):
                req = LoadMap.Request()
                req.map_url = self.map_yaml
                self.map_load_client.call_async(req)
                time.sleep(1.0)
            
            self.is_transitioning = False
            self.get_logger().info("SUCCESS: Map updated by human command.")

def main():
    rclpy.init()
    node = HumanInTheLoopHealer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()