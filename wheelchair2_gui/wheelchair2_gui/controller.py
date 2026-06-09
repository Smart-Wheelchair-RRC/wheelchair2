#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Int32
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal 
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
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
                "yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/ground/lidarnewmap2d.yaml",
                "pcd": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/ground/lidarnewmap1.pcd",
                "init_x": -1.245,
                "init_y": -0.83,
                "init_z": 0.0025,
                "init_w": 0.99999
            },
            "FLOOR_4": {
                "yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/4th/lidarnewmap2d.yaml",
                "pcd": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/4th/lidarnewmap.pcd",
                "init_x": -2.475,
                "init_y":  -0.733,
                "init_z": 0.14152,
                "init_w": 0.98993
            }
        }

        self.waiting_for_score = False
        self.active_target_layer = None
        self.deferred_final_target = None
        self.nav_state = "IDLE" 
        
        self.current_floor = self.determine_current_floor_dynamically()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.app_status_pub = self.create_publisher(String, '/navigation_status', status_qos)
        self.nav_feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        self.targets_update_pub = self.create_publisher(String, '/targets_updated', 10)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.click_loc_pub = self.create_publisher(Bool, '/click_loc', 10)

        self.lift_feedback_pub = self.create_publisher(String, '/lift_map_switch_feedback', 10)
        self.create_subscription(String, '/lift_map_switch_trigger', self.lift_map_switch_callback, 10)
        
        self.create_subscription(Int32, '/score', self.score_topic_callback, 10)

        self.create_subscription(String, '/save_target_trigger', self.save_target_topic_callback, 10)
        self.create_subscription(String, '/delete_target_trigger', self.delete_target_topic_callback, 10)
        self.create_subscription(String, '/go_to_dynamic_target_trigger', self.navigate_topic_callback, 10)

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self.nav2_feedback_callback, 10)

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.last_goal = None
        self.is_arrived_notified = False

        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)
        self.create_service(Trigger, 'get_current_pose', self.get_current_pose_callback)
        self.create_service(Trigger, 'get_stored_targets', self.get_stored_targets_callback)

        self.publish_targets_heartbeat()
        self.create_timer(1.0, self.publish_targets_heartbeat)

        self.app_status_pub.publish(String(data=self.nav_state))
        self.get_logger().info(f' DUCKY BACKEND INTEGRATION ENGINE ONLINE: Initialized dynamically on => [{self.current_floor}]')

    def determine_current_floor_dynamically(self):
        tmux_floor = self.determine_floor_from_tmux()
        if tmux_floor:
            return tmux_floor

        try:
            cmd = "ps aux | grep 3d_stack.launch.py"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                out = result.stdout
                if "4th" in out or "4th/lidarnewmap.pcd" in out:
                    self.get_logger().info("Process monitor trace matched: [FLOOR_4]")
                    return "FLOOR_4"
                elif "ground" in out or "ground/lidarnewmap1.pcd" in out:
                    self.get_logger().info("Process monitor trace matched: [GROUND]")
                    return "GROUND"
        except Exception:
            pass

        self.get_logger().error(" ALL POSITION DETECTION CRITERIA EXHAUSTED. Baseline default assigned: [GROUND]")
        return "GROUND"

    def determine_floor_from_tmux(self):
        """
        Extracts the explicit console history from the wheelchair_navigation session pane to verify 
        the active pcd layout or launch target without dealing with lifecycle parameter service locks.
        """
        try:
            cmd = ["tmux", "capture-pane", "-pt", "wheelchair_navigation:1.0"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            
            if result.returncode == 0:
                pane_content = result.stdout
                
                if "4th" in pane_content or "4th/lidarnewmap" in pane_content or "4th/lidarnewmap.pcd" in pane_content:
                    self.get_logger().info("🔍 [BOOT CHECK] Tmux history stream verified: [FLOOR_4]")
                    return "FLOOR_4"
                elif "ground" in pane_content or "ground/lidarnewmap" in pane_content or "ground/lidarnewmap1.pcd" in pane_content:
                    self.get_logger().info("Tmux history stream verified: [GROUND]")
                    return "GROUND"
                    
        except Exception as e:
            self.get_logger().error(f" Tmux history extraction pipeline failed: {str(e)}")
        return None

    def lift_map_switch_callback(self, msg):
        target_layer = msg.data.upper().strip()
        if target_layer not in ["GROUND", "FLOOR_4"]:
            return

        self.get_logger().info(f" [LIFT MUX ENGAGED] Starting map migration sequence to floor layer: [{target_layer}]")
        self.lift_feedback_pub.publish(String(data='{"data": "MAP_LOADING"}'))
        
        layer_data = self.maps_db[target_layer]
        yaml_path = layer_data["yaml"]
        pcd_path = layer_data["pcd"]

        self.get_logger().info(f" Loading map yaml configuration to map_server: {yaml_path}")
        try:
            cmd_map = f"ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{{map_url: '{yaml_path}'}}\" > /dev/null 2>&1 &"
            subprocess.Popen(cmd_map, shell=True)
            
            self.get_logger().info(" Clearing global and local costmap layers...")
            subprocess.Popen("ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
            subprocess.Popen("ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
        except Exception as e:
            self.get_logger().error(f" Nav2 map infrastructure background swap failure: {str(e)}")

        self.get_logger().info(f" Killing old localization stack and launching 3D point cloud stack via Tmux with map: {pcd_path}")
        try:
            subprocess.run(["tmux", "send-keys", "-t", "wheelchair_navigation:1.0", "C-c"], stdout=subprocess.DEVNULL)
            time.sleep(0.5) 
            launch_str = f"ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:={pcd_path} nature:=indoor"
            subprocess.run(["tmux", "send-keys", "-t", "wheelchair_navigation:1.0", launch_str, "C-m"], stdout=subprocess.DEVNULL)
    
        except Exception as e:
            self.get_logger().error(f" Tmux navigation stack execution tracking fault: {str(e)}")

        self.active_target_layer = target_layer
        self.waiting_for_score = True
        self.get_logger().info("⏳ Map load dispatched. Awaiting geometric score convergence matching node signal down topic /score...")

    def score_topic_callback(self, msg):
        if not self.waiting_for_score or self.active_target_layer is None:
            return

        captured_score = msg.data
    
        
        if captured_score > 0:
            self.get_logger().info(f" [NATIVE SIGNAL CONVERGED] Point cloud matched with score: {captured_score}. Resetting map transforms...")
            self.waiting_for_score = False
            layer_data = self.maps_db[self.active_target_layer]

            try:
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                
                pose_msg.pose.pose.position.x = float(layer_data["init_x"])
                pose_msg.pose.pose.position.y = float(layer_data["init_y"])
                pose_msg.pose.pose.position.z = 0.0
                
                pose_msg.pose.pose.orientation.z = float(layer_data["init_z"])
                pose_msg.pose.pose.orientation.w = float(layer_data["init_w"])
               
                self.initial_pose_pub.publish(pose_msg)
                
                click_msg = Bool()
                click_msg.data = True
                self.click_loc_pub.publish(click_msg)
            except Exception as e:
                self.get_logger().error(f" Localization pipeline frame alignment error: {str(e)}")

            feedback_signal = "SUCCESS_FLOOR_G" if self.active_target_layer == "GROUND" else "SUCCESS_FLOOR_4"
            json_payload = f'{{"data": "{feedback_signal}"}}'
            
            self.lift_feedback_pub.publish(String(data=json_payload))
            self.get_logger().info(f" [HANDOVER FINISHED] Layout [{self.active_target_layer}] actively targeted and ready.")
            
            self.current_floor = self.active_target_layer
            self.active_target_layer = None

            # Broadcast updated tracking information to frontend link
            floor_sync_msg = f"CURRENT_FLOOR:{self.current_floor}"
            self.app_status_pub.publish(String(data=floor_sync_msg))


            if self.deferred_final_target:
                self.target_to_fire = self.deferred_final_target
                self.deferred_final_target = None
                
                self.get_logger().info(f"❄️ [COOL DOWN ENGAGED] Localization stabilized. Scheduling post-swap refresh steps...")
                self.cooldown_step1_timer = self.create_timer(3.0, self._cooldown_clear_costmaps_callback)

    def _cooldown_clear_costmaps_callback(self):
        self.cooldown_step1_timer.destroy()
        self.get_logger().info("Performing a post-convergence costmap purge to clear phantom obstacles...")
        try:
            subprocess.Popen("ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
            subprocess.Popen("ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" > /dev/null 2>&1 &", shell=True)
        except Exception:
            pass
        self.cooldown_step2_timer = self.create_timer(2.0, self._cooldown_dispatch_target_callback)

    def _cooldown_dispatch_target_callback(self):
        self.cooldown_step2_timer.destroy()
        if hasattr(self, 'target_to_fire') and self.target_to_fire:
            self.get_logger().info(f" [CROSS-FLOOR ROUTING ACTIVE] Sending final destination goal: [{self.target_to_fire}] now!")
            self.execute_navigation_by_name(self.target_to_fire)
            self.target_to_fire = None

    def load_locations_from_disk(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    self.locations = json.load(f)
                self.get_logger().info(f"Loaded {len(self.locations)} targets successfully from disk storage database.")
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
    
            self.current_floor = self.determine_current_floor_dynamically()
            self.load_locations_from_disk() 
            
            response.success = True
            response.message = json.dumps(self.locations)
            
            floor_sync_msg = f"CURRENT_FLOOR:{self.current_floor}"
            self.app_status_pub.publish(String(data=floor_sync_msg))
            
            time.sleep(0.05)
            self.app_status_pub.publish(String(data=self.nav_state))
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
            floor_assignment = data.get('floor', 'GROUND').upper().strip()
            
            self.locations[name] = {
                "x": coords["x"],
                "y": coords["y"],
                "yaw": coords["yaw"],
                "floor": floor_assignment
            }
            self.write_locations_to_disk()
            self.get_logger().info(f"⚡ REGISTERED CONTEXTUAL TARGET: [{name}] mapped on layer => [{floor_assignment}]")
        except Exception as e:
            self.get_logger().error(f" Save multi-floor parameter parser error: {str(e)}")

    def delete_target_topic_callback(self, msg):
        try:
            target_name = msg.data.upper().strip()
            if target_name in self.locations:
                del self.locations[target_name]
                self.write_locations_to_disk()
                self.get_logger().info(f" DELETED TARGET NODE: [{target_name}] from disk memory context footprint.")
        except Exception as e:
            self.get_logger().error(f" Delete parsing execution error: {str(e)}")

    def navigate_topic_callback(self, msg):
        incoming_data = msg.data.strip()
        target_name = incoming_data.upper()
        
        self.get_logger().info(f"[NAV INPUT RECEIVED] Raw data packet ingested: '{incoming_data}'")
        
        if incoming_data.startswith("{") and incoming_data.endswith("}"):
            try:
                parsed_packet = json.loads(incoming_data)
                target_name = parsed_packet.get("target", "").upper().strip()
                deferred_target = parsed_packet.get("deferred_target", "").upper().strip()
                
                if deferred_target:
                    self.deferred_final_target = deferred_target
                    self.get_logger().info(f" [CROSS-FLOOR ROUTING INITIALIZED] Immediate Stop: [{target_name}] | Final Target Cached: [{self.deferred_final_target}]")
            except Exception as json_err:
                self.get_logger().error(f" Cross-floor trigger JSON ingestion mapping parsing mismatch anomaly: {str(json_err)}")
        else:
            if "(" in target_name:
                target_name = target_name.split("(")[0].strip()
                self.get_logger().info(f" Cleaned raw string target payload context to: [{target_name}]")

        self.execute_navigation_by_name(target_name)

    def execute_navigation_by_name(self, target_name):
        if target_name in self.locations:
            loc = self.locations[target_name]
            target_floor = loc.get('floor', 'GROUND').upper().strip()
            
            self.get_logger().info(f" NAV QUERY STATUS: Target=[{target_name}] | Current Floor State=[{self.current_floor}] | Expected Layer=[{target_floor}]")

            # --- SMART CROSS-FLOOR INTERCEPTION MIDDLEWARE ---
            if target_floor != self.current_floor:
                self.get_logger().warn(f"MAP MISMATCH: Target requires map layout [{target_floor}] but robot is on [{self.current_floor}]!")
                
                lift_waypoint = "LIFT_G" if self.current_floor == "GROUND" else "LIFT_4"
                
                if target_name == lift_waypoint:
                    pass
                else:
                    self.get_logger().info(f" INTERCEPTED: Setting [{target_name}] aside. Re-routing stack destination to: [{lift_waypoint}]")
                    self.deferred_final_target = target_name
                    
                    loc = self.locations.get(lift_waypoint)
                    if not loc:
                        self.get_logger().error(f"INTERCEPT CRITICAL DETOUR FAILURE: Node '{lift_waypoint}' missing in target storage schema configuration!")
                        return
                    target_name = lift_waypoint

            # --- SYSTEM COORD PUBLISH SEQUENCER ---
            self.get_logger().info(f" Preparing navigation goal coordinates for [{target_name}] (Floor Layer context: {self.current_floor})")
            
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
            
            self.get_logger().info(f" Publishing Goal Pose to Nav2 stack for node target: [{target_name}]")
            self.goal_pub.publish(goal)
            self.app_status_pub.publish(String(data=self.nav_state))
        else:
            self.get_logger().error(f" NAVIGATION ABORTED: Target identifier node unknown or unmapped on disk database: [{target_name}]")

    def nav_status_callback(self, msg):
        if not msg.status_list: return
        last_status = msg.status_list[-1].status
        if last_status == 4 and not self.is_arrived_notified:
            self.get_logger().info("🏁 Nav2 Goal Status indicates: GOAL REACHED")
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="REACHED"))
            self.is_arrived_notified = True
        elif last_status == 6:
            self.get_logger().warn(" Nav2 Goal Status indicates: GOAL ABORTED/FAILED")
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="ABORTED"))

    def wait_action_status_callback(self, msg):
        if not msg.status_list: return
        if msg.status_list[-1].status == 2 and self.nav_state != "WAITING":
            self.get_logger().warn(" Nav2 Local Planner blocked. State changed to: WAITING")
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
            self.get_logger().info(" Cancelling active Nav2 goal request pipeline...")
            self.cancel_client.call_async(CancelGoal.Request())

    def pause_callback(self, request, response):
        self.get_logger().info(" Service called: pause_robot")
        self.trigger_nav2_cancel()
        self.nav_state = "PAUSED"
        self.app_status_pub.publish(String(data="PAUSED"))
        response.success = True
        return response

    def resume_callback(self, request, response):
        self.get_logger().info(" Service called: resume_robot")
        if self.last_goal:
            self.get_logger().info(" Republicating previous goal coordinates to goal_pose topic...")
            self.last_goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.last_goal)
            self.nav_state = "ACTIVE"
            self.app_status_pub.publish(String(data="ACTIVE"))
            response.success = True
        else:
            self.get_logger().error("Cannot resume navigation: no last known goal is cached in stack memory.")
            response.success = False
        return response

    def stop_callback(self, request, response):
        self.get_logger().info("Service called: stop_robot")
        self.trigger_nav2_cancel()
        self.last_goal = None
        self.nav_state = "IDLE"
        if not self.waiting_for_score:
            self.deferred_final_target = None
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