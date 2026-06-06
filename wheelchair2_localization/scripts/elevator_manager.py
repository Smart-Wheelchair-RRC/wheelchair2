import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess

class ElevatorManager(Node):
    def __init__(self):
        super().__init__('elevator_manager')
        
        # 1. State Variables
        self.current_floor = 1
        self.is_inside_elevator = False
        
        # Hardcoded 3D map coordinates of the elevator door on each floor
        self.elevator_poses = {
            1: {"x": 1.5,  "y": 1.5,  "z": 0.0, "qw": 1.0, "qz": 0.0},
            2: {"x": 1.5,  "y": 1.5,  "z": 3.5, "qw": 1.0, "qz": 0.0},
            3: {"x": 12.0, "y": 2.5,  "z": 7.0, "qw": 0.7, "qz": -0.7},
            4: {"x": -9.0, "y": -19.5, "z": 10.5, "qw": 0.72, "qz": 0.69}
        }

        # 2. Subscribers
        self.floor_sub = self.create_subscription(Int32, '/elevator/floor', self.floor_callback, 10)
        self.inside_sub = self.create_subscription(Bool, '/elevator/is_inside', self.inside_callback, 10)

        # 3. Publishers
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.get_logger().info('Elevator Manager Active and Monitoring...')

    def floor_callback(self, msg):
        if msg.data == self.current_floor:
            return
        
        self.get_logger().info(\u000f"Floor change detected: Floor {self.current_floor} -> Floor {msg.data}")
        self.current_floor = msg.data
        
        # If inside the elevator, change the maps dynamically in the background!
        if self.is_inside_elevator:
            self.reload_floor_map(self.current_floor)

    def inside_callback(self, msg):
        prev_state = self.is_inside_elevator
        self.is_inside_elevator = msg.data

        # Transition 1: Wheelchair just entered the elevator cabin
        if not prev_state and self.is_inside_elevator:
            self.get_logger().warn('Wheelchair entered elevator. Freezing 3D-BBS to save computing cycles.')
            # Optional: You can send a lifecycle command or custom service to pause 3D-BBS matching

        # Transition 2: Wheelchair is exiting the elevator onto the destination floor
        elif prev_state and not self.is_inside_elevator:
            self.get_logger().info(\u000f"Exiting elevator on Floor {self.current_floor}. Injecting Anchor Seed Pose!")
            self.inject_anchor_pose(self.current_floor)

    def reload_floor_map(self, floor_num):
        """
        Dynamically tells ICP to load the pre-downsampled map of the new floor
        """
        map_path = f"/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/floor_{floor_num}/map.pcd"
        self.get_logger().info(f"Commanding ICP to swap map layer to: {map_path}")
        
        # Call the ROS 2 runtime parameter override directly
        cmd = ["ros2", "param", "set", "/icp_localization", "pcd_file_path", map_path]
        subprocess.Popen(cmd)

    def inject_anchor_pose(self, floor_num):
        """
        Publishes the exact coordinates of the lift door to jumpstart localization instantly
        """
        if floor_num not in self.elevator_poses:
            self.get_logger().error(f"No anchor pose mapped for floor {floor_num}!")
            return

        pose_data = self.elevator_poses[floor_num]
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = pose_data["x"]
        msg.pose.pose.position.y = pose_data["y"]
        msg.pose.pose.position.z = pose_data["z"]
        
        # Orientation
        msg.pose.pose.orientation.w = pose_data["qw"]
        msg.pose.pose.orientation.z = pose_data["qz"]
        
        # Tight covariance because we know exactly where the elevator door is!
        msg.pose.covariance[0] = 0.05  # x var
        msg.pose.covariance[7] = 0.05  # y var
        msg.pose.covariance[35] = 0.02 # yaw var
        
        self.init_pose_pub.publish(msg)
        self.get_logger().info("Anchor Seed Injected successfully. ICP should latch immediately.")

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()