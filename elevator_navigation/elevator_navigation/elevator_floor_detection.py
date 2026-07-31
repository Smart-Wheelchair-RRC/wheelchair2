import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool, Empty, Trigger # ◄ UPDATED: Brought in Trigger types
from nav2_msgs.srv import ClearEntireCostmap
from cv_bridge import CvBridge
import cv2
import subprocess
import time
import math
import os

class ElevatorFloorDetection(Node):
    def __init__(self):
        super().__init__('elevator_floor_detection')
        
        # --- Config ---
        self.TARGET_FLOOR = 4
        self.LIFT_INTERIOR_LIMIT = 1.6
        self.EXIT_DIST = 1.2
        self.IMU_GRAVITY = 1.0
        self.Z_THRESHOLD = 0.05
        self.JOLT_DURATION_REQ = 0.3
        
        # Absolute path base where floor assets are kept
        self.MAPS_BASE_DIR = "/home/container_user/wheelchair2/src/config"  # Update path matching system deployment layout

        self.imu_buffer = []
        self.buffer_size = 15

        # --- State ---
        self.state = "IDLE" 
        self.move_candidate_time = None
        self.move_start_time = None
        self.start_odom = None
        self.target_found = False
        self.camera_process = None
        self.bridge = CvBridge()

        # --- Base Service Client Initializations ---
        self.pause_odom_cli = self.create_client(SetBool, 'pause_odom')
        self.reset_map_cli = self.create_client(Empty, 'reset_map')
        
        self.pause_loc_cli = self.create_client(SetBool, 'pause_localization')
        self.reset_loc_cli = self.create_client(Empty, 'reset_localization')
        
        # ◄ UPDATED: Swapped out SetString configurations for standard Trigger instances
        self.load_map_cli = self.create_client(Trigger, 'load_map')
        
        self.pause_bbs_cli = self.create_client(SetBool, 'pause_3dbbs')
        self.reset_bbs_cli = self.create_client(Empty, 'reset_3dbbs')
        self.load_bbs_cli = self.create_client(Trigger, 'load_3dbbs_config')

        # ◄ ADDED: Direct Node Param Client hooks to update configuration values over the parameter space
        self.icp_param_cli = self.create_client(SetParameters, '/icp_localization/set_parameters')
        self.bbs_param_cli = self.create_client(SetParameters, '/gpu_ros2_test_rviz2/set_parameters')

        # Nav2 lifecycle costmap clear clients
        self.clear_global_costmap_cli = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # --- Comms ---
        self.imu_sub = self.create_subscription(Imu, '/livox/imu', self.imu_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.floor_id_pub = self.create_publisher(Int32, '/detected_floor_id', 10)

        self.get_logger().info("System initialized. Multi-floor Pause/Reset architecture active.")

    def call_service_async(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return None
        return client.call_async(request)

    def trigger_elevator_entry_freeze(self):
        self.get_logger().info("Entering lift: Freezing localization pipelines...")
        
        # 1. Pause DLIO tracking loops
        req_bool = SetBool.Request()
        req_bool.data = True
        self.call_service_async(self.pause_odom_cli, req_bool)
        
        # 2. Pause ICP Scan-matching loops
        self.call_service_async(self.pause_loc_cli, req_bool)
        
        # 3. Pause 3D-BBS loops
        self.call_service_async(self.pause_bbs_cli, req_bool)

    def trigger_elevator_interior_reset(self, target_floor):
        self.get_logger().info(f"Elevator in transit. Wiping memory arrays for floor {target_floor}...")
        req_empty = Empty.Request()

        # 1. Wipe visual cloud accumulation inside MapNode
        self.call_service_async(self.reset_map_cli, req_empty)

        # 2. Wipe ICP tracking history, tracking transforms, and raw accumulators
        self.call_service_async(self.reset_loc_cli, req_empty)

        # 3. Wipe 3D-BBS input scan point buffers 
        self.call_service_async(self.reset_bbs_cli, req_empty)

        # 4. Update file parameters and invoke our updated Trigger interfaces
        floor_dir = os.path.join(self.MAPS_BASE_DIR, f"floor_{target_floor}")
        pcd_path = os.path.join(floor_dir, "map.pcd")
        yaml_path = os.path.join(floor_dir, "ros2_test.yaml")

        # Set ICP Node config parameter path state wrapper
        if os.path.exists(pcd_path):
            param_req = SetParameters.Request()
            param = Parameter()
            param.name = "config"
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = pcd_path
            param_req.parameters.append(param)
            self.call_service_async(self.icp_param_cli, param_req)
        else:
            self.get_logger().error(f"PCD file missing on disk: {pcd_path}")

        # Set 3D-BBS Node config parameter path state wrapper
        if os.path.exists(yaml_path):
            param_req = SetParameters.Request()
            param = Parameter()
            param.name = "config"
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = yaml_path
            param_req.parameters.append(param)
            self.call_service_async(self.bbs_param_cli, param_req)
        else:
            self.get_logger().error(f"YAML configuration path missing on disk: {yaml_path}")

        # Trigger clean pipeline update sequences using clean Trigger requests
        req_trigger = Trigger.Request()
        self.create_timer(0.2, lambda: self.call_service_async(self.load_map_cli, req_trigger))
        self.create_timer(0.2, lambda: self.call_service_async(self.load_bbs_cli, req_trigger))

    def trigger_elevator_exit_resume(self):
        self.get_logger().info("Exiting lift: Activating tracking frames...")
        req_bool = SetBool.Request()
        req_bool.data = False

        # Unpause tracking sequences down the pipe
        self.call_service_async(self.pause_odom_cli, req_bool)
        self.call_service_async(self.pause_loc_cli, req_bool)
        self.call_service_async(self.pause_bbs_cli, req_bool)

    def clear_navigation_costmaps(self):
        self.get_logger().info("Flushing costmap layer history...")
        req_clear = ClearEntireCostmap.Request()
        self.call_service_async(self.clear_global_costmap_cli, req_clear)
        self.call_service_async(self.clear_local_costmap_cli, req_clear)

    def imu_cb(self, msg):
        self.imu_buffer.append(msg.linear_acceleration.z)
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.pop(0)
        
        avg_z = sum(self.imu_buffer) / len(self.imu_buffer)
        accel_diff = avg_z - self.IMU_GRAVITY

        if abs(accel_diff) > self.Z_THRESHOLD:
            if self.move_candidate_time is None:
                self.move_candidate_time = time.time()
            elif (time.time() - self.move_candidate_time) > self.JOLT_DURATION_REQ:
                if self.state == "IDLE":
                    self.get_logger().info(f"Sustained motion: {'UP' if accel_diff > 0 else 'DOWN'}.")
                    self.trigger_elevator_entry_freeze()
                    
                    # Wipe memory immediately when lift takes off
                    self.trigger_elevator_interior_reset(self.TARGET_FLOOR)
                    
                    self.move_start_time = time.time()
                    self.state = "MOVING"
                    self.start_camera()
                elif self.state == "SEARCHING":
                    self.get_logger().info("Lift moving again. Resetting search.")
                    self.state = "MOVING"
                    self.move_start_time = time.time()
                    self.target_found = False
        else:
            self.move_candidate_time = None 

    def scan_cb(self, msg):
        if self.state not in ["MOVING", "SEARCHING"]: return
        if self.move_start_time and (time.time() - self.move_start_time) < 1.0: return

        num_readings = len(msg.ranges)
        front_indices = list(range(0, 30)) + list(range(num_readings-30, num_readings))
        back_indices = list(range(150, 210))
        
        f_pts = [msg.ranges[i] for i in front_indices if 0.1 < msg.ranges[i] < 10.0]
        b_pts = [msg.ranges[i] for i in back_indices if 0.1 < msg.ranges[i] < 10.0]

        f_dist = sorted(f_pts)[len(f_pts)//2] if f_pts else 0.0
        b_dist = sorted(b_pts)[len(b_pts)//2] if b_pts else 0.0

        front_open = f_dist > self.LIFT_INTERIOR_LIMIT
        back_open = b_dist > self.LIFT_INTERIOR_LIMIT

        if self.state == "MOVING" and (front_open or back_open):
            self.state = "SEARCHING"
            self.start_odom = None
        elif self.state == "SEARCHING" and not front_open and not back_open:
            self.state = "MOVING"
            self.target_found = False

    def start_camera(self):
        if self.camera_process is None:
            self.camera_process = subprocess.Popen(["ros2", "launch", "realsense2_camera", "rs_launch.py"])
            self.img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)

    def image_cb(self, msg):
        if self.state != "SEARCHING": 
            cv2.destroyAllWindows()
            return
            
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            detected_id = int(ids[0][0])
            self.get_logger().info(f"Detected ArUco ID: {detected_id}")
            
            if detected_id == self.TARGET_FLOOR:
                self.target_found = True
                cv2.putText(frame, "TARGET FLOOR MATCH!", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Elevator Vision Debug", frame)
        cv2.waitKey(1)

    def odom_cb(self, msg):
        if self.state == "SEARCHING":
            pos = msg.pose.pose.position
            if self.start_odom is None:
                self.start_odom = pos
                return
            dist = math.sqrt((pos.x - self.start_odom.x)**2 + (pos.y - self.start_odom.y)**2)
            if dist > self.EXIT_DIST:
                self.finalize_localization()

    def finalize_localization(self):
        cv2.destroyAllWindows()
        if self.target_found:
            fid_msg = Int32()
            fid_msg.data = self.TARGET_FLOOR
            self.floor_id_pub.publish(fid_msg)
            
            # 1. Update global coordinates positioning
            self.publish_pose(self.TARGET_FLOOR)
            
            # 2. Flush residual costmaps to prevent ghost wall artifacts
            self.clear_navigation_costmaps()
            
            # 3. Unfreeze alignment trackers to engage local localization frames
            self.trigger_elevator_exit_resume()
            
            self.reset("DONE")
        else:
            self.get_logger().warn("Exited but no marker match.")
            self.trigger_elevator_exit_resume() # Safety resume even if marker missed
            self.reset("IDLE")

    def publish_pose(self, floor_id):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # Coordinates matching the new floor elevator landing threshold layout
        msg.pose.pose.position.x = 15.5
        msg.pose.pose.position.y = -2.2
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.05   # x var
        msg.pose.covariance[7] = 0.05   # y var
        msg.pose.covariance[35] = 0.02  # yaw var
        self.pose_pub.publish(msg)

    def reset(self, next_state):
        if self.camera_process:
            self.camera_process.terminate()
            self.camera_process = None
        if hasattr(self, 'img_sub'): self.destroy_subscription(self.img_sub)
        self.state = next_state
        self.start_odom = None
        self.target_found = False

def main():
    rclpy.init()
    node = ElevatorFloorDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()