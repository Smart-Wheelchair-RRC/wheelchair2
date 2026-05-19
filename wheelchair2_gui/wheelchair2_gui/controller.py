#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal 
from tf2_ros import TransformListener, Buffer
import math
import json
import os

class DuckyMissionController(Node):
    def __init__(self):
        super().__init__('ducky_mission_controller')

        self.json_path = "/home/container_user/wheelchair2/src/wheelchair2_gui/wheelchair2_gui/robot_targets.json"
        self.locations = {}
        self.load_locations_from_disk()

 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.app_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.nav_feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        
        self.targets_update_pub = self.create_publisher(String, '/targets_updated', 10)
        
        self.create_subscription(String, '/save_target_trigger', self.save_target_topic_callback, 10)
        self.create_subscription(String, '/delete_target_trigger', self.delete_target_topic_callback, 10)
        self.create_subscription(String, '/go_to_dynamic_target_trigger', self.navigate_topic_callback, 10)

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self.nav2_feedback_callback, 10)

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.last_goal = None
        self.nav_state = "IDLE" 
        self.is_arrived_notified = False

        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)
        self.create_service(Trigger, 'get_current_pose', self.get_current_pose_callback)
        
        self.create_service(Trigger, 'get_stored_targets', self.get_stored_targets_callback)

        self.publish_targets_heartbeat()

        self.create_timer(0.1, self.publish_targets_heartbeat)

        self.get_logger().info('⚡ DUCKY CORE ONLINE: Target refresh, sync, and clear services active.')

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
        """Returns the complete target JSON directly to frontend client on-demand."""
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