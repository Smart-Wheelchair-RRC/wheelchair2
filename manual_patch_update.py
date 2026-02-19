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
from sklearn.cluster import DBSCAN

class MapPatcher(Node):
    def __init__(self):
        super().__init__('map_patcher')
        
        # Client for Lifecycle and Map Loading
        self.amcl_client = self.create_client(ChangeState, '/amcl/change_state')
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # Paths
        self.map_base_path = "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/updated_lab"
        self.map_png = self.map_base_path + ".png"
        self.map_yaml = self.map_base_path + ".yaml"

        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.get_logger().info("Map Patcher Node Ready. Waiting for AMCL pose...")

    def filter_map_noise(self, image_path):
        """Integrating version of filter_occupancy_map_dbscan_optimized"""
        self.get_logger().info(f"DBSCAN: Starting filter on {image_path}...")
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().error(f"Could not read image for filtering: {image_path}")
            return
        
        filtered_image = image.copy()

            # === BLACK PIXELS (Obstacles) ===
        # User params: e=3, m=7
        black_mask = image == 0
        black_coords = np.argwhere(black_mask)

        if len(black_coords) > 0:
            dbscan_black = DBSCAN(eps=3, min_samples=7)
            labels_black = dbscan_black.fit_predict(black_coords)
            noise_indices_black = black_coords[labels_black == -1]
            
            for row, col in noise_indices_black:
                filtered_image[row, col] = 255
            self.get_logger().info(f"DBSCAN: Removed {len(noise_indices_black)} black noise pixels.")

        # === GREY PIXELS (Uncertainty) ===
        # User params: ge=4, gm=40, filter_grey=True
        grey_mask = (image > 0) & (image < 255)
        grey_coords = np.argwhere(grey_mask)

        if len(grey_coords) > 0:
            MIN_GREY_CLUSTER_SIZE = 10  
            dbscan_grey = DBSCAN(eps=4, min_samples=40)
            labels_grey = dbscan_grey.fit_predict(grey_coords)
            
            unique_labels = set(labels_grey)
            grey_removed_count = 0

            for lbl in unique_labels:
                mask = labels_grey == lbl
                cluster_points = grey_coords[mask]
                
                # Remove if noise (-1) or cluster is smaller than threshold
                if lbl == -1 or len(cluster_points) < MIN_GREY_CLUSTER_SIZE:
                    for row, col in cluster_points:
                        filtered_image[row, col] = 255
                    grey_removed_count += len(cluster_points)

            self.get_logger().info(f"DBSCAN: Removed {grey_removed_count} grey noise pixels.")

        cv2.imwrite(image_path, filtered_image)
        self.get_logger().info("DBSCAN: Processing complete. Output saved.")


    def pose_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().info("First AMCL pose received! System ready for switch.")
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def run_workflow(self):
        slam_proc = None
        try:
            # --- NEW: Auto-wait enabling for the first pose ---
            self.get_logger().info("Checking for AMCL pose...")
            timeout_counter = 0
            while self.current_pose is None and timeout_counter < 50:
                self.get_logger().info("Waiting for /amcl_pose data... (ensure rosbag is playing)")
                rclpy.spin_once(self, timeout_sec=0.1) # Force the node to check for messages
                timeout_counter += 1
        
            # --- NEW: Checking if we have a pose first ---
            if self.current_pose is None:
                self.get_logger().error("TIMEOUT: No AMCL pose received. Is the rosbag playing?")
                return

            # --- NEW: Extracting and formatting the pose ---
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            pose_str = f"[{x}, {y}, {yaw}]"

            # --- STEP 1: DEACTIVATE AMCL ---
            self.get_logger().warn(f"Transitioning: Deactivating AMCL... Capturing pose: {pose_str}")
            self.change_lifecycle_state(self.amcl_client, Transition.TRANSITION_DEACTIVATE)

            # --- STEP 2: START SLAM ---
            self.get_logger().info("Launching SLAM to capture changes...")
            
            # Specifying wheelchair2 packages
            setup_cmd = "source /home/container_user/wheelchair2/install/setup.bash && "
            launch_cmd = f"ros2 launch wheelchair2_navigation online_async_launch.py use_sim_time:=True " \
                         f"map_start_pose:='{pose_str}' " \
                         f"map_file_name:=/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/posegraph/slam_lifelong_run"
            
            slam_proc = subprocess.Popen(['bash', '-c', setup_cmd + launch_cmd])
            
            # slam_proc = subprocess.Popen([
                # 'ros2', 'launch', 'wheelchair2_navigation', 'online_async_launch.py', 'use_sim_time:=True',
                # f'map_start_pose:={pose_str}', # Pass the captured pose
                # 'map_file_name:=/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/posegraph/slam_lifelong_run' # Load posegraph
            # ])        

            # --- STEP 3: WAIT FOR CAPTURE ---
            print("\n>>> SLAM is running. Drive the robot around the changed area.")
            input(">>> Press Enter when you have finished capturing the changes to SAVE the map...")

            # --- STEP 4: SAVING THE MAP ---
            self.get_logger().info("Saving updated map...")
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_base_path, '--fmt', 'png', '--ros-args', '-p', 'save_map_timeout:=10.0', '-p', 'use_sim_time:=True'
                # 'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_path, '--ros-args', '-p', 'save_map_timeout:=10.0'
            ])
        
            # --- APPLYING THE NOISE FILTER ---
            time.sleep(1.0) 
            self.filter_map_noise(self.map_png)

        except Exception as e:
            self.get_logger().error(f"Workflow failed: {e}")

        finally:
            if slam_proc:
                self.get_logger().info("Cleaning up SLAM process...")
                slam_proc.terminate()
                try:
                    slam_proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    slam_proc.kill() # Force kill
    
        # --- STEP 6: LOAD NEW MAP & RESUME AMCL ---
        # Load and Resume
            try:
                self.get_logger().info("Pushing new map to Map Server...")
                # 1. First, tell the map server to load the new file from disk
                self.load_new_map(self.map_yaml)

                # 2. Crucial: Give the Map Server 2 seconds to broadcast the new occupancy grid
                time.sleep(2.0) 

                # 3. Now wake up AMCL so it 'sees' the new map immediately on startup
                self.get_logger().info("Activating AMCL...")
                self.change_lifecycle_state(self.amcl_client, Transition.TRANSITION_ACTIVATE)

                self.get_logger().info("SUCCESS: Map healed, filtered, and navigation resumed.")
    
            except Exception as e:
                self.get_logger().error(f"Reload failed: {e}")            
            
            # self.load_new_map(self.map_yaml)
            # self.change_lifecycle_state(self.amcl_client, Transition.TRANSITION_ACTIVATE)
            # self.get_logger().info("SUCCESS: Map healed, filtered, and navigation resumed.")

    def cleanup(self):
        """Kills the SLAM process and restores AMCL if needed."""
        if hasattr(self, 'slam_proc') and self.slam_proc:
            self.get_logger().info("Cleaning up SLAM process...")
            self.slam_proc.terminate()
            try:
                self.slam_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.slam_proc.kill()
        
        # Ensure AMCL is back on
        self.change_lifecycle_state(self.amcl_client, Transition.TRANSITION_ACTIVATE)

    def change_lifecycle_state(self, client, transition_id):
        if client.wait_for_service(timeout_sec=2.0):
            req = ChangeState.Request()
            req.transition.id = transition_id
            client.call_async(req)
            time.sleep(1) # Waiting for state change

    def load_new_map(self, yaml_file):
        if self.map_load_client.wait_for_service(timeout_sec=2.0):
            req = LoadMap.Request()
            req.map_url = yaml_file
            self.map_load_client.call_async(req)

def main():
    rclpy.init()
    node = MapPatcher()
    try:
        input("Press Enter to start the Map Update...")
        node.run_workflow()
        node.get_logger().info("SUCCESS: Map healed and filtered.") # Move this here
    except Exception as e:
        node.get_logger().error(f"Workflow failed: {e}")
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()