#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import math
import sys

class GetMapCoords(Node):
    def __init__(self):
        super().__init__('get_map_coords')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.get_pos)
        self.get_logger().info('Searching for robot position... (Make sure your map/odom is active)')

    def get_pos(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                now, 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Convert Quaternion to Euler Yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            print(f'\n"PLACE_NAME":     {{"x": {round(x, 2)},  "y": {round(y, 2)},  "yaw": {round(yaw, 2)}}},')
            
            raise KeyboardInterrupt 
            
        except Exception as e:

            pass

def main(args=None):
    rclpy.init(args=args)
    node = GetMapCoords()
    
    try:

        rclpy.spin(node)
    except KeyboardInterrupt:

        print("\nCoordinate captured. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()