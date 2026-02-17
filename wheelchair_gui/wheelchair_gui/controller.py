#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal

class WheelchairMissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- 1. Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # Subscriber for the Manual Joystick from the App
        self.create_subscription(Twist, 'cmd_vel', self.teleop_callback, 10)

        # --- 2. Action Clients ---
        self.cancel_client = self.create_client(
            CancelGoal, 
            '/navigate_to_pose/_action/cancel_goal'
        )

        # --- 3. State & Smoothing Variables ---
        self.last_goal = None
        self.target_twist = Twist()   # What the app wants
        self.current_twist = Twist()  # What we actually send (smoothed)
        
        # SMOOTHING FACTOR: 0.18 gives a premium, comfortable glide.
        self.smoothing_factor = 0.18 
        
        # 10Hz Heartbeat: keeps the motors alive and applies smoothing
        self.publish_timer = self.create_timer(0.1, self.publish_twist_callback)

        # --- 4. Service Endpoints ---
        self.create_service(Trigger, 'go_to_products_lab', self.products_lab_callback)
        self.create_service(Trigger, 'go_to_cie_lab', self.cie_lab_callback)
        self.create_service(Trigger, 'go_to_entrance', self.entrance_callback)
        self.create_service(Trigger, 'go_to_RRC', self.rrc_callback)
        
        self.create_service(Trigger, 'pause_robot', self.pause_callback)
        self.create_service(Trigger, 'resume_robot', self.resume_callback)
        self.create_service(Trigger, 'stop_robot', self.stop_callback)

        print("\n" + "█"*60)
        self.get_logger().info('SMOOTH MISSION CONTROL: ONLINE')
        self.get_logger().info('Zero-Snap Logic Active (Instant Stop on Release)')
        print("█"*60 + "\n")

    def teleop_callback(self, msg):
        """Receives raw joystick input from the Pixel 6."""
        self.target_twist = msg

    def publish_twist_callback(self):
        """Applies Ramping, but snaps to zero immediately when joystick is let go."""
        
        # --- Linear X (Forward/Backward) ---
        # If target is 0, we stop immediately (No smoothing)
        if abs(self.target_twist.linear.x) < 0.01:
            self.current_twist.linear.x = 0.0
        else:
            # Apply smoothing for active movement
            self.current_twist.linear.x += (self.target_twist.linear.x - self.current_twist.linear.x) * self.smoothing_factor
        
        # --- Angular Z (Turning) ---
        if abs(self.target_twist.angular.z) < 0.01:
            self.current_twist.angular.z = 0.0
        else:
            self.current_twist.angular.z += (self.target_twist.angular.z - self.current_twist.angular.z) * self.smoothing_factor

        self.cmd_vel_pub.publish(self.current_twist)

    def trigger_nav2_cancel(self):
        """Official Nav2 'Cancel Goal' request."""
        self.get_logger().warn('ACTION: Calling Nav2 CancelGoal Service...')
        
        # Reset speeds immediately to stop
        self.target_twist = Twist()
        self.current_twist = Twist()
        
        if self.cancel_client.service_is_ready():
            req = CancelGoal.Request()
            self.cancel_client.call_async(req)
            self.get_logger().info('SUCCESS: Nav2 Mission Aborted.')
        else:
            self.get_logger().error('Nav2 Action Server not responding!')

    def send_goal(self, x, y, label):
        self.get_logger().info(f'🔘 DESTINATION SET: {label}')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        
        self.last_goal = goal
        self.goal_pub.publish(goal)

    # --- Mission Callbacks ---
    def products_lab_callback(self, request, response):
        self.send_goal(2.0, 1.5, "PRODUCTS LAB")
        response.success = True
        return response

    def cie_lab_callback(self, request, response):
        self.send_goal(5.0, -2.0, "CIE LAB")
        response.success = True
        return response

    def entrance_callback(self, request, response):
        self.send_goal(0.0, 0.0, "ENTRANCE")
        response.success = True
        return response

    def rrc_callback(self, request, response):
        self.send_goal(-3.0, 0.5, "RRC")
        response.success = True
        return response

    # --- Safety & Resume Callbacks ---
    def pause_callback(self, request, response):
        self.get_logger().warn('⚠️ PAUSE Triggered')
        self.trigger_nav2_cancel()
        response.success = True
        return response

    def resume_callback(self, request, response):
        if self.last_goal is not None:
            self.get_logger().info('▶️ RESUME: Continuing mission...')
            self.last_goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.last_goal)
            response.success = True
        else:
            self.get_logger().error('Resume failed: No history.')
            response.success = False
        return response

    def stop_callback(self, request, response):
        self.get_logger().error('🚨 EMERGENCY STOP')
        self.last_goal = None
        self.trigger_nav2_cancel()
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WheelchairMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()