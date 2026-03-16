#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal 
import math
import json

class DuckyMissionController(Node):
    def __init__(self):
        super().__init__('ducky_mission_controller')

        # --- 1. Map Coordinates ---
        self.locations = {
            "ENTRANCE":   {"x": 19.2,  "y": 31.17,  "yaw": 3.11},
            "BODH 101":   {"x": 40.14,  "y": -9.46,  "yaw": -1.02},
            "iHUB ADMIN": {"x": 32.89,  "y": 5.59,  "yaw": 3.13},
            "H102":       {"x": 25.88,  "y": 8.67,  "yaw": -3.05},
            "H103":       {"x": 36.9,  "y": -20.93,  "yaw": -1.61},
            
            # {"x": 4.62,  "y": -7.19,  "yaw": -0.04}   point a\
            # {"x": 6.11,  "y": 5.49,  "yaw": 3.01} exit door
            # {"x": 0.63,  "y": -0.15,  "yaw": -3.05} point b
            # "ENTRANCE":   {"x": 6.11,  "y": 5.49,  "yaw": 3.01},
            # "BODH 101":   {"x": 40.14,  "y": -9.46,  "yaw": -1.02},
            # "iHUB ADMIN": {"x": 3.08, "y": -6.45, "yaw": -1.76},
            # "H102":       {"x": 4.62,  "y": -7.19,  "yaw": -0.04},
            # "H103":       {"x": 0.63,  "y": -0.15,  "yaw": -3.05},
        }

        # --- 2. Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.app_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.nav_feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        
        # Monitor the Main Navigation Action
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        
        # HUMBLE FIX: Monitor the Wait Action Status directly (Replaces the Log listener)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, 
            '/navigate_to_pose/_action/feedback', 
            self.nav2_feedback_callback, 
            10
        )

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # --- 3. State Management ---
        self.last_goal = None
        self.nav_state = "IDLE" 
        self.is_arrived_notified = False

        # --- 4. Service Endpoints ---
        self.create_service(Trigger, 'go_to_ihub', lambda r, p: self.nav_to("iHUB ADMIN", r, p))
        self.create_service(Trigger, 'go_to_bodh101', lambda r, p: self.nav_to("BODH 101", r, p))
        self.create_service(Trigger, 'go_to_h102', lambda r, p: self.nav_to("H102", r, p))
        self.create_service(Trigger, 'go_to_h103', lambda r, p: self.nav_to("H103", r, p))
        self.create_service(Trigger, 'go_to_entrance', lambda r, p: self.nav_to("ENTRANCE", r, p))
        
        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)

        self.get_logger().info('DUCKY MISSION CONTROL: BT-AWARE BACKEND ONLINE (Wait Action Monitor)')

    def wait_action_status_callback(self, msg):
        """Detects if the Behavior Tree is currently in the Recovery Wait state."""
        if not msg.status_list:
            return
            
        # Status 2 = ACTIVE (Robot is currently waiting for obstacle to clear)
        current_status = msg.status_list[-1].status
        if current_status == 2:
            if self.nav_state != "WAITING":
                self.nav_state = "WAITING"
                self.get_logger().warn("OBSTACLE DETECTED: Behavior Tree is waiting...")
                self.app_status_pub.publish(String(data="WAITING"))
        else:
            # If wait is no longer active, but we were waiting, set back to ACTIVE
            if self.nav_state == "WAITING":
                self.nav_state = "ACTIVE"
                self.app_status_pub.publish(String(data="ACTIVE"))

    def nav2_feedback_callback(self, msg):
        """Processes distance and ETA feedback."""
        try:
            distance = msg.feedback.distance_remaining
            eta_sec = msg.feedback.estimated_time_remaining.sec
            feedback_dict = {"distance": round(float(distance), 2), "eta_sec": int(eta_sec)}
            self.nav_feedback_pub.publish(String(data=json.dumps(feedback_dict)))
        except Exception: pass

    def nav_status_callback(self, msg):
        """Monitors main goal status for Reach or Abort."""
        if not msg.status_list: return
        last_status = msg.status_list[-1].status
        
        if last_status == 4 and not self.is_arrived_notified:
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="REACHED"))
            self.is_arrived_notified = True
        elif last_status == 6: # ABORTED
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="ABORTED"))
        elif last_status in [1, 2]:
            self.is_arrived_notified = False

    def nav_to(self, room_id, request, response):
        if room_id in self.locations:
            loc = self.locations[room_id]
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
            
            response.success = True
            response.message = f"Navigating to {room_id}"
            return response
        response.success = False
        return response

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