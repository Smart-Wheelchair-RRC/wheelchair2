#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Int32
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal 
from tf2_ros import TransformListener, Buffer
import math
import json
import os
import subprocess
import time

class DuckyMissionController(Node):
    def __init__(self):
        super().__init__('ducky_mission_controller')

        # --- Dynamic Storage File Configuration ---
        self.json_path = "/home/container_user/wheelchair2/src/wheelchair2_gui/wheelchair2_gui/robot_targets.json"
        self.locations = {}
        self.load_locations_from_disk()

        # --- Map Paths Database ---
        self.maps_db = {
            "GROUND": {
                "yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/hardware_lab_2.yaml",
                "pcd": "/home/container_user/wheelchair2/src/hardware_lab_2.pcd",
                "init_x": 0.0,
                "init_y": 0.0,
                "init_z": 0.0,
                "init_w": 1.0
            },
            "FLOOR_4": {
                "yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.yaml",
                "pcd": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.pcd",
                "init_x": -20.24,
                "init_y": -2.96,
                "init_z": -0.01999,
                "init_w": 0.99979
            }
        }

        # --- State Tracking Flags for LiftMux Handshake ---
        self.waiting_for_score = False
        self.active_target_layer = None

        # --- TF Setup for Real-time Pose Capture ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.app_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.nav_feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        self.targets_update_pub = self.create_publisher(String, '/targets_updated', 10)
        
        # Native Handshake Allocation Endpoints (Fixes Frame Drop and Timing Latency)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.click_loc_pub = self.create_publisher(Bool, '/click_loc', 10)

        # Lift Interface Pipelines
        self.lift_feedback_pub = self.create_publisher(String, '/lift_map_switch_feedback', 10)
        self.create_subscription(String, '/lift_map_switch_trigger', self.lift_map_switch_callback, 10)
        
        # NATIVE SCORE SUBSCRIPTION (Bypasses Subprocess & Drops completely)
        self.create_subscription(Int32, '/score', self.score_topic_callback, 10)

        # Topic Ingestion Endpoints
        self.create_subscription(String, '/save_target_trigger', self.save_target_topic_callback, 10)
        self.create_subscription(String, '/delete_target_trigger', self.delete_target_topic_callback, 10)
        self.create_subscription(String, '/go_to_dynamic_target_trigger', self.navigate_topic_callback, 10)

        # Nav2 Pipeline Subscriptions
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self.nav2_feedback_callback, 10)

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # Control States
        self.last_goal = None
        self.nav_state = "IDLE" 
        self.is_arrived_notified = False

        # Core Navigation Controls
        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)
        self.create_service(Trigger, 'get_current_pose', self.get_current_pose_callback)
        self.create_service(Trigger, 'get_stored_targets', self.get_stored_targets_callback)

        # Heartbeat loop initialization
        self.publish_targets_heartbeat()
        self.create_timer(0.1, self.publish_targets_heartbeat)

        self.get_logger().info('⚡ DUCKY BACKEND INTEGRATION ENGINE ONLINE: Native Handshaking active.')

    def lift_map_switch_callback(self, msg):
        target_layer = msg.data.upper().strip()
        if target_layer not in ["GROUND", "FLOOR_4"]:
            self.get_logger().error(f"Invalid floor signature dropped: {target_layer}")
            return

        self.get_logger().info(f" [LIFT MUX ENGAGED] Starting lightning map migration sequence to: {target_layer}")
        
        # ========================================================
        # FIX: EXPLICIT JSON WRAPPING FOR INITIAL LOADING STATE
        # ========================================================
        self.lift_feedback_pub.publish(String(data='{"data": "MAP_LOADING"}'))
        
        layer_data = self.maps_db[target_layer]
        yaml_path = layer_data["yaml"]
        pcd_path = layer_data["pcd"]

        # 2. Invoke Nav2 Map Server loading client ASYNCHRONOUSLY to maximize hardware performance
        self.get_logger().info("Blasting Nav2 map parameters asynchronously...")
        try:
            cmd_map = f"ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{{map_url: '{yaml_path}'}}\" > /dev/null 2>&1 &"
            subprocess.Popen(cmd_map, shell=True)
            
            # Asynchronously flush and re-bound costmap tracking frames
            subprocess.Popen("ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
            subprocess.Popen("ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
        except Exception as e:
            self.get_logger().error(f"Nav2 map infrastructure background swap failure: {str(e)}")

        # 3. Recycle 3D localization stack processes running inside Tmux container instantly
        self.get_logger().info("Recycling 3D point cloud localization nodes inside tmux session window...")
        try:
            subprocess.run(["tmux", "send-keys", "-t", "wheelchair_navigation:1.0", "C-c"], stdout=subprocess.DEVNULL)
            time.sleep(0.5) # Lightning operational delay minimum
            launch_str = f"ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:={pcd_path} nature:=indoor"
            subprocess.run(["tmux", "send-keys", "-t", "wheelchair_navigation:1.0", launch_str, "C-m"], stdout=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Tmux navigation stack execution tracking fault: {str(e)}")

        # 4. ARM THE WATCHDOG INTERCEPTOR
        self.active_target_layer = target_layer
        self.waiting_for_score = True
        self.get_logger().info("⌛ Native /score pipeline armed. Awaiting geometric matching node signal...")

    def score_topic_callback(self, msg):
        # Only process data frames if a map layout switch has been actively commanded
        if not self.waiting_for_score or self.active_target_layer is None:
            return

        captured_score = msg.data
        if captured_score > 0:
            self.get_logger().info(f" [NATIVE SIGNAL RECEIVED] Score converged at: {captured_score}. Loading navigation variables...")
            
            # Disarm tracker flag instantly to drop multi-message processing loops
            self.waiting_for_score = False
            layer_data = self.maps_db[self.active_target_layer]

            # ========================================================
            # NATIVE HIGH-SPEED POSE INJECTION (FIXES TIMING SYNC ISSUES)
            # ========================================================
            try:
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg() # Fixed timing serialization
                
                pose_msg.pose.pose.position.x = float(layer_data["init_x"])
                pose_msg.pose.pose.position.y = float(layer_data["init_y"])
                pose_msg.pose.pose.position.z = 0.0
                
                pose_msg.pose.pose.orientation.z = float(layer_data["init_z"])
                pose_msg.pose.pose.orientation.w = float(layer_data["init_w"])
                
                # Instant distribution down the global pipeline channel
                self.initial_pose_pub.publish(pose_msg)
                
                # Trigger localization re-initialization hook
                click_msg = Bool()
                click_msg.data = True
                self.click_loc_pub.publish(click_msg)
                self.get_logger().info("Initial pose positioning vectors successfully synchronized.")
            except Exception as e:
                self.get_logger().error(f"Localization pipeline frame alignment error: {str(e)}")

            # ========================================================
            # FIX: EXPLICIT JSON STRUCUTRED STRINGS FOR FLUTTER RECEPTION
            # ========================================================
            feedback_signal = "SUCCESS_FLOOR_G" if self.active_target_layer == "GROUND" else "SUCCESS_FLOOR_4"
            json_payload = f'{{"data": "{feedback_signal}"}}'
            
            self.lift_feedback_pub.publish(String(data=json_payload))
            self.get_logger().info(f"🎉 [HANDOVER FINISHED] Target {self.active_target_layer} actively running navigation parameters.")
            
            # Flush thread environment parameters clean
            self.active_target_layer = None

    def load_locations_from_disk(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    self.locations = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Failed to read location schema database: {str(e)}")
        else:
            self.locations = {}
            self.write_locations_to_disk()

    def write_locations_to_disk(self):
        try:
            with open(self.json_path, 'w') as f:
                json.dump(self.locations, f, indent=2)
            self.publish_targets_heartbeat()
        except Exception as e:
            self.get_logger().error(f"Disk write failure: {str(e)}")

    def publish_targets_heartbeat(self):
        msg = String()
        msg.data = json.dumps(self.locations)
        self.targets_update_pub.publish(msg)

    def get_stored_targets_callback(self, request, response):
        try:
            self.load_locations_from_disk() 
            response.success = True
            response.message = json.dumps(self.locations)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def get_current_pose_callback(self, request, response):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.3))
            x = round(trans.transform.translation.x, 3)
            y = round(trans.transform.translation.y, 3)
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = round(math.atan2(siny_cosp, cosy_cosp), 3)

            response.success = True
            response.message = json.dumps({"x": x, "y": y, "yaw": yaw})
        except Exception as e:
            response.success = False
            response.message = f"TF Failure: {str(e)}"
        return response

    def save_target_topic_callback(self, msg):
        try:
            data = json.loads(msg.data)
            name = data['name'].upper().strip()
            coords = data['coords']
            
            self.locations[name] = coords
            self.write_locations_to_disk()
            self.get_logger().info(f"⚡ REGISTERED TARGET: [{name}]")
        except Exception as e:
            self.get_logger().error(f"Save parser error: {str(e)}")

    def delete_target_topic_callback(self, msg):
        try:
            target_name = msg.data.upper().strip()
            if target_name in self.locations:
                del self.locations[target_name]
                self.write_locations_to_disk()
                self.get_logger().info(f"🔥 DELETED TARGET NODE: [{target_name}] from disk footprint.")
        except Exception as e:
            self.get_logger().error(f"Delete parsing execution error: {str(e)}")

    def navigate_topic_callback(self, msg):
        target_name = msg.data.upper().strip()
        if target_name in self.locations:
            loc = self.locations[target_name]
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(loc['x'])
            goal.pose.position.y = float(loc['y'])
            goal.pose.orientation.z = math.sin(loc['yaw'] / 2.0)
            goal.pose.orientation.w = math.cos(loc['yaw'] / 2.0)
            
            self.last_goal = goal
            self.nav_state = "ACTIVE"
            self.is_arrived_notified = False
            
            self.goal_pub.publish(goal)
            self.app_status_pub.publish(String(data="ACTIVE"))

    def nav_status_callback(self, msg):
        if not msg.status_list: return
        last_status = msg.status_list[-1].status
        if last_status == 4 and not self.is_arrived_notified:
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="REACHED"))
            self.is_arrived_notified = True
        elif last_status == 6:
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="ABORTED"))

    def wait_action_status_callback(self, msg):
        if not msg.status_list: return
        if msg.status_list[-1].status == 2 and self.nav_state != "WAITING":
            self.nav_state = "WAITING"
            self.app_status_pub.publish(String(data="WAITING"))

    def nav2_feedback_callback(self, msg):
        try:
            distance = msg.feedback.distance_remaining
            feedback_dict = {"distance": round(float(distance), 2), "eta_sec": int(msg.feedback.estimated_time_remaining.sec)}
            self.nav_feedback_pub.publish(String(data=json.dumps(feedback_dict)))
        except Exception: pass

    def trigger_nav2_cancel(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())

    def pause_callback(self, request, response):
        self.trigger_nav2_cancel()
        self.nav_state = "PAUSED"
        self.app_status_pub.publish(String(data="PAUSED"))
        response.success = True
        return response

    def resume_callback(self, request, response):
        if self.last_goal:
            self.last_goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.last_goal)
            self.nav_state = "ACTIVE"
            self.app_status_pub.publish(String(data="ACTIVE"))
            response.success = True
        else:
            response.success = False
        return response

    def stop_callback(self, request, response):
        self.trigger_nav2_cancel()
        self.last_goal = None
        self.nav_state = "IDLE"
        self.app_status_pub.publish(String(data="IDLE"))
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DuckyMissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()