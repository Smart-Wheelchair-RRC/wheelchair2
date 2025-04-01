#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

WHEELCHAIR_MAX_LIN_VEL = 2.0  # Max velocity for Wheelchair_V (m/s)
WHEELCHAIR_MAX_ANG_VEL = 2.0

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.25

msg = """
Control Your Wheelchair!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

def getKey():
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(value, min_val, max_val):
    return max(min(value, max_val), min_val)

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_keyboard', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        print(msg)

    def publish_twist(self):
        key = getKey()
        if key:
            if key == 'w':
                self.target_linear_vel = constrain(self.target_linear_vel + LIN_VEL_STEP_SIZE, -WHEELCHAIR_MAX_LIN_VEL, WHEELCHAIR_MAX_LIN_VEL)
            elif key == 'x':
                self.target_linear_vel = constrain(self.target_linear_vel - LIN_VEL_STEP_SIZE, -WHEELCHAIR_MAX_LIN_VEL, WHEELCHAIR_MAX_LIN_VEL)
            elif key == 'a':
                self.target_angular_vel = constrain(self.target_angular_vel + ANG_VEL_STEP_SIZE, -WHEELCHAIR_MAX_ANG_VEL, WHEELCHAIR_MAX_ANG_VEL)
            elif key == 'd':
                self.target_angular_vel = constrain(self.target_angular_vel - ANG_VEL_STEP_SIZE, -WHEELCHAIR_MAX_ANG_VEL, WHEELCHAIR_MAX_ANG_VEL)
            elif key in (' ', 's'):
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
            elif key == '\x03':
                self.destroy_node()
                rclpy.shutdown()
                return
            print(f"Linear Vel: {self.target_linear_vel:.2f}, Angular Vel: {self.target_angular_vel:.2f}")
        
        twist = Twist()
        twist.linear.x = self.target_linear_vel
        twist.angular.z = self.target_angular_vel
        self.publisher.publish(twist)

if __name__ == '__main__':
    rclpy.init()
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    teleop_node = KeyboardTeleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
