#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger, SetBool, Empty
from std_msgs.msg import String, Bool, Int32
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal 
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformListener, Buffer
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from nav2_msgs.srv import LoadMap
import math
import json
import os
import subprocess
import time
import threading

class DuckyMissionController(Node):
    def __init__(self):
        super().__init__('ducky_mission_controller')

        # --- Dynamic Storage File Configuration ---
        self.json_path = "/home/container_user/wheelchair2/src/wheelchair2_gui/wheelchair2_gui/wheelchair_config.json"
        self.config = {}
        self.locations = {}
        self.floor_configs = {}
        self.load_locations_from_disk()

        self.waiting_for_score = False
        self.active_target_layer = None
        self.deferred_final_target = None
        self.nav_state = "IDLE" 
        
        self.current_floor = self.config.get("state", {}).get("current_floor", "1")

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

        # App interactions for elevator
        self.elevator_q_pub = self.create_publisher(String, '/app/question', 10)
        self.create_subscription(Bool, '/elevator/cmd_confirm', self.elevator_confirm_callback, 10)
        self.elevator_confirm_event = threading.Event()

        # Re-using original topics to not break app
        self.lift_feedback_pub = self.create_publisher(String, '/lift_map_switch_feedback', 10)
        
        self.create_subscription(String, '/save_target_trigger', self.save_target_topic_callback, 10)
        self.create_subscription(String, '/delete_target_trigger', self.delete_target_topic_callback, 10)
        self.create_subscription(String, '/go_to_dynamic_target_trigger', self.navigate_topic_callback, 10)

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self.nav2_feedback_callback, 10)

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._nav2_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Service Clients for Map Hotswap ---
        icp_node = "/icp_localization"
        bbs_node = "/gpu_ros2_test_rviz2"
        self.icp_pause = self.create_client(SetBool, "/pause_localization")
        self.icp_reset = self.create_client(Empty, "/reset_localization")
        self.icp_load = self.create_client(Trigger, "/load_map")
        self.icp_set_param = self.create_client(SetParameters, f"{icp_node}/set_parameters")

        self.bbs_pause = self.create_client(SetBool, "/bbs/pause")
        self.bbs_reload = self.create_client(Trigger, "/bbs/reload_map")
        self.bbs_set_param = self.create_client(SetParameters, f"{bbs_node}/set_parameters")

        self.dlio_reset = self.create_client(Trigger, "/dlio/odom/reset")

        self.map_load = self.create_client(LoadMap, "/map_server/load_map")
        self.global_clear = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        self.local_clear = self.create_client(Empty, "/local_costmap/clear_entirely_local_costmap")

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

    def elevator_confirm_callback(self, msg: Bool):
        if msg.data:
            self.elevator_confirm_event.set()

    def _wait_for_gui_confirm(self, question: str):
        self.elevator_confirm_event.clear()
        self.get_logger().info(f"Asking GUI: {question}")
        self.elevator_q_pub.publish(String(data=question))
        # Wait until the callback sets the event
        self.elevator_confirm_event.wait()
        self.get_logger().info("GUI Confirmed.")

    def _run_elevator_sequence(self, target_floor: str):
        self.get_logger().info(f"--- STARTING ELEVATOR SEQUENCE TO {target_floor} ---")
        
        current_cfg = self.floor_configs.get(self.current_floor)
        to_cfg = self.floor_configs.get(target_floor)

        if not current_cfg or not to_cfg:
            self.get_logger().error("Floor configuration missing! Aborting.")
            return

        # 1. Drive to wait pose
        self.nav_state = "ACTIVE"
        self.app_status_pub.publish(String(data=self.nav_state))
        self.get_logger().info("Driving to elevator wait pose...")
        self.send_nav2_goal_sync(current_cfg["wait_pose"])

        # 2. Wait for door open
        self._wait_for_gui_confirm("Are the elevator doors open?")

        # 3. Drive inside
        self.get_logger().info("Driving inside elevator...")
        self.send_nav2_goal_sync(current_cfg["inside_pose"])

        # 4. Map swap (Pausing ICP/BBS)
        self.get_logger().info("Swapping Maps (Pausing localizers)...")
        self.lift_feedback_pub.publish(String(data='{"data": "MAP_LOADING"}'))
        self._set_bool_service(self.icp_pause, True)
        self._set_bool_service(self.bbs_pause, True)

        self._trigger_service(self.map_load, LoadMap.Request(map_url=to_cfg["nav2_yaml"]))
        self._trigger_service(self.global_clear, Empty.Request())
        self._trigger_service(self.local_clear, Empty.Request())

        self._set_string_param(self.icp_set_param, "map_path", to_cfg["icp_pcd"])
        self._trigger_service(self.icp_load, Trigger.Request())
        self._trigger_service(self.icp_reset, Empty.Request())

        self._set_string_param(self.bbs_set_param, "map_path", to_cfg["bbs_dir"])
        self._trigger_service(self.bbs_reload, Trigger.Request())
        # Note: BBS remains paused

        self.get_logger().info("Map swap commands sent. Waiting for arrival.")

        # 5. Wait for arrival
        self._wait_for_gui_confirm(f"Have you arrived at Floor {target_floor}?")

        # 6. Wait for doors open after arrival
        self._wait_for_gui_confirm("Are the elevator doors open?")

        # 7. Localization handoff
        self.get_logger().info("Handoff: Resetting DLIO drift and unpausing ICP...")
        self._trigger_service(self.dlio_reset, Trigger.Request())
        time.sleep(0.5)

        self._set_bool_service(self.icp_pause, False)
        time.sleep(0.3)

        anchor = to_cfg["anchor_pose"]
        self.get_logger().info("Publishing anchor pose...")
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = float(anchor["x"])
        pose_msg.pose.pose.position.y = float(anchor["y"])
        pose_msg.pose.pose.position.z = float(anchor.get("z", 0.0))
        yaw = float(anchor["yaw"])
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance
        pose_msg.pose.covariance[0]  = 1.0   
        pose_msg.pose.covariance[7]  = 1.0   
        pose_msg.pose.covariance[14] = 0.25  
        pose_msg.pose.covariance[21] = 0.25  
        pose_msg.pose.covariance[28] = 0.25  
        pose_msg.pose.covariance[35] = 0.5 
        
        self.initial_pose_pub.publish(pose_msg)
        click_msg = Bool()
        click_msg.data = True
        self.click_loc_pub.publish(click_msg)

        feedback_signal = "SUCCESS_FLOOR_G" if target_floor == "1" else f"SUCCESS_FLOOR_{target_floor}"
        self.lift_feedback_pub.publish(String(data=f'{{"data": "{feedback_signal}"}}'))

        # 8. Drive to exit wait pose
        self.get_logger().info("Exiting elevator to wait pose...")
        self.send_nav2_goal_sync(to_cfg["wait_pose"])

        # 9. Finalize
        self.current_floor = target_floor
        self.config["state"]["current_floor"] = self.current_floor
        self.write_locations_to_disk()
        
        self.get_logger().info(f"Elevator sequence complete. Dispatched to final target: {self.deferred_final_target}")
        self.execute_navigation_by_name(self.deferred_final_target)
        self.deferred_final_target = None


    # --- Service Helper Functions ---
    def _set_bool_service(self, client, value):
        if client.wait_for_service(3.0):
            req = SetBool.Request()
            req.data = value
            client.call_async(req)

    def _trigger_service(self, client, req):
        if client.wait_for_service(3.0):
            client.call_async(req)

    def _set_string_param(self, client, name, value):
        if client.wait_for_service(3.0):
            req = SetParameters.Request()
            p = Parameter(name=name, value=ParameterType(type=ParameterType.PARAMETER_STRING, string_value=value))
            req.parameters = [p]
            client.call_async(req)

    def send_nav2_goal_sync(self, loc):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        yaw = float(loc.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav2_action.wait_for_server(5.0):
            return

        future = self._nav2_action.send_goal_async(goal)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
            
        res_future = goal_handle.get_result_async()
        res_event = threading.Event()
        res_future.add_done_callback(lambda f: res_event.set())
        res_event.wait()

    def load_locations_from_disk(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, 'r') as f:
                    self.config = json.load(f)
                self.locations = self.config.get("targets", {})
                self.floor_configs = self.config.get("floor_configs", {})
                self.get_logger().info(f"Loaded {len(self.locations)} targets successfully from disk storage database.")
            except Exception as e:
                self.get_logger().error(f"Failed to read location schema database: {str(e)}")
        else:
            self.get_logger().warn("JSON config not found at path. Using empty dictionary.")
            self.config = {"state": {"current_floor": "1"}, "floor_configs": {}, "targets": {}}
            self.locations = {}
            self.floor_configs = {}

    def write_locations_to_disk(self):
        try:
            self.config["targets"] = self.locations
            with open(self.json_path, 'w') as f:
                json.dump(self.config, f, indent=2)
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
            floor_assignment = data.get('floor', '1').upper().strip()
            
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
            except Exception as json_err:
                self.get_logger().error(f" Cross-floor trigger JSON ingestion mapping parsing mismatch anomaly: {str(json_err)}")
        else:
            if "(" in target_name:
                target_name = target_name.split("(")[0].strip()

        self.execute_navigation_by_name(target_name)

    def execute_navigation_by_name(self, target_name):
        if target_name in self.locations:
            loc = self.locations[target_name]
            target_floor = loc.get('floor', '1').upper().strip()
            
            self.get_logger().info(f" NAV QUERY STATUS: Target=[{target_name}] | Current Floor State=[{self.current_floor}] | Expected Layer=[{target_floor}]")

            # --- SMART CROSS-FLOOR INTERCEPTION MIDDLEWARE ---
            if target_floor != self.current_floor:
                self.get_logger().warn(f"MAP MISMATCH: Target requires map layout [{target_floor}] but robot is on [{self.current_floor}]!")
                
                self.get_logger().info(f" INTERCEPTED: Setting [{target_name}] aside. Re-routing to elevator flow sequence.")
                self.deferred_final_target = target_name
                
                # Spawn background thread to handle elevator logic to not block ROS
                threading.Thread(target=self._run_elevator_sequence, args=(target_floor,), daemon=True).start()
                return

            # --- SYSTEM COORD PUBLISH SEQUENCER (Normal Route) ---
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
