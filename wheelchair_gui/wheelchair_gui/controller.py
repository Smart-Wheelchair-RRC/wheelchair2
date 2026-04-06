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

        # --- . Map Coordinates ---
        self.locations = {
            "ENTRANCE":   {"x": -2.41,  "y": 13.31,  "yaw": -3.03},
            "BODH 101":   {"x": 40.14,  "y": -9.46,  "yaw": -1.02},
            "iHUB ADMIN": {"x": 32.89,  "y": 5.59,  "yaw": 3.13},
            "H102":       {"x": 12.38,  "y": 15.87,  "yaw": 0.16},
            "H103":       {"x": 6.03,  "y": 14.98,  "yaw": 3.14},
        }

        # --- Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.app_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.nav_feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.create_subscription(GoalStatusArray, '/wait/_action/status', self.wait_action_status_callback, 10)
        

        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, 
            '/navigate_to_pose/_action/feedback', 
            self.nav2_feedback_callback, 
            10
        )

        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # --- State ---
        self.last_goal = None
        self.nav_state = "IDLE" 
        self.is_arrived_notified = False

        # --- Services ---
        self.create_service(Trigger, 'go_to_ihub', lambda r, p: self.nav_to("iHUB ADMIN", r, p))
        self.create_service(Trigger, 'go_to_bodh101', lambda r, p: self.nav_to("BODH 101", r, p))
        self.create_service(Trigger, 'go_to_h102', lambda r, p: self.nav_to("H102", r, p))
        self.create_service(Trigger, 'go_to_h103', lambda r, p: self.nav_to("H103", r, p))
        self.create_service(Trigger, 'go_to_entrance', lambda r, p: self.nav_to("ENTRANCE", r, p))
        
        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)

        self.get_logger().info('DUCKY MISSION CONTROL: ONLINE. Ready for goals.')

    def nav_to(self, room_id, request, response):
        """Dispatches a goal and logs the exact coordinates."""
        if room_id in self.locations:
            loc = self.locations[room_id]
            
            # --- LOGGING THE ATTEMPT ---
            self.get_logger().info(f"--- NEW GOAL REQUESTED ---")
            self.get_logger().info(f"Target: {room_id}")
            self.get_logger().info(f"Coordinates: X={loc['x']}, Y={loc['y']}, Yaw={loc['yaw']}")

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(loc['x'])
            goal.pose.position.y = float(loc['y'])
            
            # Quaternions
            goal.pose.orientation.z = math.sin(loc['yaw'] / 2.0)
            goal.pose.orientation.w = math.cos(loc['yaw'] / 2.0)
            
            self.last_goal = goal
            self.nav_state = "ACTIVE"
            self.is_arrived_notified = False
            
            # Publish to Nav2
            self.goal_pub.publish(goal)
            self.app_status_pub.publish(String(data="ACTIVE"))
            
            response.success = True
            response.message = f"Successfully dispatched goal to {room_id}"
            return response
        
        self.get_logger().error(f"GOAL FAILED: {room_id} not found in map dictionary.")
        response.success = False
        return response

    def nav_status_callback(self, msg):
        """Detailed logging for Nav2 status changes."""
        if not msg.status_list: 
            return
            
        last_status = msg.status_list[-1].status
        
        status_map = {1: "ACCEPTED", 2: "EXECUTING", 4: "SUCCEEDED", 6: "ABORTED", 5: "CANCELED"}
        status_name = status_map.get(last_status, f"UNKNOWN({last_status})")

        if last_status == 4 and not self.is_arrived_notified:
            self.get_logger().info("GOAL REACHED: Robot arrived at destination.")
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="REACHED"))
            self.is_arrived_notified = True
        elif last_status == 6:
            self.get_logger().error("GOAL ABORTED: Nav2 failed to reach the goal.")
            self.nav_state = "IDLE"
            self.app_status_pub.publish(String(data="ABORTED"))
        elif last_status == 2 and self.nav_state != "ACTIVE":
            self.get_logger().info(f"NAV STATUS UPDATE: {status_name}")

    def wait_action_status_callback(self, msg):
        if not msg.status_list: return
        current_status = msg.status_list[-1].status
        if current_status == 2:
            if self.nav_state != "WAITING":
                self.nav_state = "WAITING"
                self.get_logger().warn("OBSTACLE DETECTED: Behavior Tree Recovery 'WAIT' is active.")
                self.app_status_pub.publish(String(data="WAITING"))

    def nav2_feedback_callback(self, msg):
        try:
            distance = msg.feedback.distance_remaining
            # if int(distance) % 5 == 0: self.get_logger().info(f"Dist left: {distance:.2f}m")
            feedback_dict = {"distance": round(float(distance), 2), "eta_sec": int(msg.feedback.estimated_time_remaining.sec)}
            self.nav_feedback_pub.publish(String(data=json.dumps(feedback_dict)))
        except Exception: pass

    def trigger_nav2_cancel(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())

    def pause_callback(self, request, response):
        self.get_logger().info("ACTION: Pausing navigation.")
        self.trigger_nav2_cancel()
        self.nav_state = "PAUSED"
        self.app_status_pub.publish(String(data="PAUSED"))
        response.success = True
        return response

    def resume_callback(self, request, response):
        if self.last_goal:
            self.get_logger().info("ACTION: Resuming last goal coordinates.")
            self.last_goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.last_goal)
            self.nav_state = "ACTIVE"
            self.app_status_pub.publish(String(data="ACTIVE"))
            response.success = True
        else:
            self.get_logger().warn("RESUME FAILED: No previous goal exists.")
            response.success = False
        return response

    def stop_callback(self, request, response):
        self.get_logger().info("ACTION: Full Stop/Clear Goal.")
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