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


class ManualTransitionTester(Node):
    def __init__(self):
        super().__init__('manual_transition_tester')
        
        # Service Clients
        self.amcl_client = self.create_client(ChangeState, '/amcl/change_state')
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # Paths
        self.ws_path = "/home/container_user/wheelchair2"
        self.map_base = f"{self.ws_path}/src/wheelchair2_navigation/maps/updated_lab"
        self.map_yaml = f"{self.ws_path}/src/wheelchair2_navigation/maps/updated_lab.yaml"
        self.map_png = f"{self.ws_path}/src/wheelchair2_navigation/maps/updated_lab.png"

        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.get_logger().info("Manual Transition Tester Ready. Ensure rosbag is playing.")

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
            self.get_logger().info(">>> SUCCESS: First AMCL pose received!")
        self.current_pose = msg.pose.pose

    def get_yaw(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    # Call for transition
    def change_state(self, transition_id):
        if self.amcl_client.wait_for_service(timeout_sec=2.0):
            req = ChangeState.Request()
            req.transition.id = transition_id
            self.amcl_client.call_async(req)
            time.sleep(2.0)

    def run_test(self):
        # 1. Wait for AMCL to localize
        while rclpy.ok() and self.current_pose is None:
            self.get_logger().info("Waiting for /amcl_pose...")
            rclpy.spin_once(self, timeout_sec=1.0)

        input("\n[STEP 1] Pose Captured. Press ENTER to PAUSE AMCL and START SLAM...")

        slam_proc = None
        try:
            # 2. Handoff Pose
            x, y = self.current_pose.position.x, self.current_pose.position.y
            yaw = self.get_yaw(self.current_pose.orientation)
            pose_str = f"[{x}, {y}, {yaw}]"

            # 3. TRANSITION: Deactivate AMCL
            self.get_logger().warn("Deactivating AMCL...")
            self.change_state(Transition.TRANSITION_DEACTIVATE)

            # 4. START SLAM (Using the Bash Source Fix)
            self.get_logger().info(f"Starting SLAM at {pose_str}...")
            bash_cmd = f"source {self.ws_path}/install/setup.bash && " \
                       f"ros2 launch wheelchair2_navigation online_async_launch.py " \
                       f"use_sim_time:=True map_start_pose:='{pose_str}'"
            slam_proc = subprocess.Popen(['bash', '-c', bash_cmd])

            input("\n[STEP 2] SLAM is running. Press ENTER to SAVE Map and RESUME AMCL...")

            # 5. SAVE MAP (Using Sim Time Fix)
            self.get_logger().info("Saving map...")
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
                '-f', self.map_base, '--fmt', 'png', '--ros-args', '-p', 'use_sim_time:=True'
            ])

            # Noise filter before loading the map
            time.sleep(1.0) 
            self.filter_map_noise(self.map_png)            

            # 6. RELOAD MAP SERVICE
            self.get_logger().info("Refreshing Map Server...")
            if self.map_load_client.wait_for_service(timeout_sec=5.0):
                req = LoadMap.Request()
                req.map_url = self.map_yaml
                self.map_load_client.call_async(req)
                time.sleep(2.0)

        except Exception as e:
            self.get_logger().error(f"Test failed: {e}")
        finally:
            if slam_proc:
                self.get_logger().info("Terminating SLAM...")
                slam_proc.terminate()
                slam_proc.wait()
            
            # 7. TRANSITION: Activate AMCL
            self.get_logger().info("Re-activating AMCL...")
            self.change_state(Transition.TRANSITION_ACTIVATE)
            self.get_logger().info("Test Finished. Check RViz for the updated map.")

            # 3. FORCE REFRESH: Tell the Map Server to read the filtered PNG from disk
            self.get_logger().warn("Refreshing Map Server with the NEW filtered map...")
            if self.map_load_client.wait_for_service(timeout_sec=5.0):
                req = LoadMap.Request()
                req.map_url = self.map_yaml # Points to the newly filtered .yaml
                self.map_load_client.call_async(req)
                time.sleep(1.0) # Short pause to let the map propagate
            
            self.get_logger().info("Test Finished. RViz should now show the updated, filtered map.")

def main():
    rclpy.init()
    node = ManualTransitionTester()
    node.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()