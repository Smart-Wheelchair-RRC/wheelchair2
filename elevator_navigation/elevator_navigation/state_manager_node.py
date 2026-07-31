import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Empty, Trigger
from nav2_msgs.srv import ClearEntireCostmap
import yaml
import os
import time
import math

from elevator_navigation.topological_router import TopologicalGraphRouter

class NavigationStateManager(Node):
    def __init__(self):
        super().__init__('nav_state_manager')
        
        self.declare_parameter('yaml_config_path', '')
        self.declare_parameter('initial_building', 'D-Block')
        self.declare_parameter('initial_floor', 0)  # Integer floor ID default (0 = Ground Floor)

        yaml_path = self.get_parameter('yaml_config_path').get_parameter_value().string_value
        
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().error(f"Invalid YAML config path: '{yaml_path}'")
            return

        with open(yaml_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.router = TopologicalGraphRouter(self.config)
        self.base_dir = self.config['system']['base_dir']

        # --- PERSISTENT STATE ---
        self.current_building = self.get_parameter('initial_building').get_parameter_value().string_value
        self.current_floor = int(self.get_parameter('initial_floor').get_parameter_value().integer_value)
        self.current_elevator = None
        self.robot_status = "IDLE"           # IDLE, NAVIGATING, IN_ELEVATOR, PAUSED
        self.current_pose = [0.0, 0.0, 0.0]
        self.active_goal_request = None
        self.detected_floor_from_terminal = None

        # --- ROS 2 SERVICE CLIENTS FOR MAP SWAPPING & LOCALIZATION ---
        self.pause_odom_cli = self.create_client(SetBool, 'pause_odom')
        self.reset_map_cli = self.create_client(Empty, 'reset_map')
        self.pause_loc_cli = self.create_client(SetBool, 'pause_localization')
        self.reset_loc_cli = self.create_client(Empty, 'reset_localization')
        self.load_map_cli = self.create_client(Trigger, 'load_map')
        self.pause_bbs_cli = self.create_client(SetBool, 'pause_3dbbs')
        self.reset_bbs_cli = self.create_client(Empty, 'reset_3dbbs')
        self.load_bbs_cli = self.create_client(Trigger, 'load_3dbbs_config')

        self.icp_param_cli = self.create_client(SetParameters, '/icp_localization/set_parameters')
        self.bbs_param_cli = self.create_client(SetParameters, '/gpu_ros2_test_rviz2/set_parameters')

        self.clear_global_costmap_cli = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # --- ROS 2 INTERFACES ---
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to terminal node confirmation
        self.floor_sub = self.create_subscription(Int32, '/detected_floor_id', self.floor_detection_cb, 10)

        self.get_logger().info(f"State Manager Online. Initialized on {self.current_building} -> Floor {self.current_floor}")

    def floor_detection_cb(self, msg):
        """Catches the confirmed integer floor ID from the Terminal Prompt node."""
        self.detected_floor_from_terminal = int(msg.data)
        self.get_logger().info(f"Received confirmed floor ID from terminal: Floor {self.detected_floor_from_terminal}")

    def call_service_async(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return None
        return client.call_async(request)

    def trigger_pipeline_freeze(self):
        """Pauses localization pipelines before swapping."""
        req_bool = SetBool.Request()
        req_bool.data = True
        self.call_service_async(self.pause_odom_cli, req_bool)
        self.call_service_async(self.pause_loc_cli, req_bool)
        self.call_service_async(self.pause_bbs_cli, req_bool)

    def trigger_pipeline_resume(self):
        """Resumes localization pipelines after swapping."""
        req_bool = SetBool.Request()
        req_bool.data = False
        self.call_service_async(self.pause_odom_cli, req_bool)
        self.call_service_async(self.pause_loc_cli, req_bool)
        self.call_service_async(self.pause_bbs_cli, req_bool)

    def clear_navigation_costmaps(self):
        req_clear = ClearEntireCostmap.Request()
        self.call_service_async(self.clear_global_costmap_cli, req_clear)
        self.call_service_async(self.clear_local_costmap_cli, req_clear)

    def send_user_goal(self, target_bldg, target_floor, target_pose):
        self.active_goal_request = {
            "bldg": target_bldg,
            "floor": int(target_floor),
            "pose": target_pose
        }
        self.replan_and_execute()

    def replan_and_execute(self):
        plan = self.router.get_route(
            self.current_building,
            self.current_floor,
            self.current_pose,
            self.active_goal_request["bldg"],
            self.active_goal_request["floor"],
            self.active_goal_request["pose"]
        )
        self.execute_plan(plan)

    def execute_plan(self, plan):
        for step in plan:
            if step['action'] == 'navigate':
                self.robot_status = "NAVIGATING"
                self.get_logger().info(f"Navigating: {step['description']}")
                self.ros_navigate_to_pose(step['pose'])
                
            elif step['action'] == 'wait_for_elevator':
                success = self.interact_with_elevator(step['shaft_name'], step['target_floor'])
                if not success:
                    return
                
            elif step['action'] == 'swap_map_and_initialize':
                self.get_logger().info(f"Map Swap: {step['description']}")
                self.swap_maps(step['new_occupancy'], step['new_pcd'])
                self.inject_initial_pose(step['initial_pose'])
                
                # --- STATE UPDATE ---
                self.current_floor = int(step['new_floor_id'])
                self.current_elevator = None
                self.robot_status = "IDLE"

        self.get_logger().info("Target Reached Successfully!")
        self.active_goal_request = None

    def interact_with_elevator(self, shaft_name, target_floor):
        self.robot_status = "IN_ELEVATOR"
        self.current_floor = None 
        self.current_elevator = shaft_name
        self.detected_floor_from_terminal = None
        self.get_logger().info(f"Riding {shaft_name} to Floor {target_floor}. Waiting for terminal confirmation...")

        # Wait for ElevatorFloorDetectionTerminal to publish the integer floor ID
        while self.robot_status == "IN_ELEVATOR" and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            
            if self.detected_floor_from_terminal is not None:
                actual_floor = int(self.detected_floor_from_terminal)
                if actual_floor == int(target_floor):
                    self.get_logger().info(f"Arrived at intended floor: Floor {actual_floor}")
                    return True 
                else:
                    self.get_logger().warn(f"Doors opened on UNEXPECTED floor: Floor {actual_floor}!")
                    return self.handle_wrong_floor_exit(actual_floor)
        return False

    def handle_wrong_floor_exit(self, actual_floor):
        """Loads the unexpected integer floor's map, localizes the robot, and prompts for recovery."""
        self.robot_status = "PAUSED"
        
        # Verify floor exists in YAML configuration
        floor_dict = self.config['buildings'][self.current_building]['floors']
        if actual_floor not in floor_dict:
            self.get_logger().error(f"Floor {actual_floor} does not exist in YAML config for {self.current_building}!")
            self.active_goal_request = None
            self.robot_status = "IDLE"
            return False

        floor_data = floor_dict[actual_floor]
        exit_pose = floor_data['elevators'][self.current_elevator]['exit_initial_pose']
        
        # 1. Immediately swap maps to the floor we actually landed on so localization doesn't break
        self.get_logger().info(f"Emergency localization: Loading map for Floor {actual_floor}...")
        self.swap_maps(floor_data['maps']['occupancy'], floor_data['maps']['pcd'])
        self.inject_initial_pose(exit_pose)
        
        # 2. Update persistent state to reality
        self.current_floor = actual_floor
        self.current_pose = exit_pose
        self.current_elevator = None
        
        # 3. Ask the user in the terminal what to do next
        user_choice = self.prompt_user_ui(
            f"\n[ALERT] Exited on Floor {actual_floor}, but goal is on Floor {self.active_goal_request['floor']}."
            f"\n>>> Do you still want to navigate to Floor {self.active_goal_request['floor']}? (y/n): "
        )
        
        if user_choice == "CONTINUE":
            self.get_logger().info("User chose CONTINUE. Generating recovery route from current floor...")
            # Automatically generates a new path: Current Floor Elevator -> Target Floor -> Final Room
            self.replan_and_execute()
            return False
        else:
            self.get_logger().info("Navigation cancelled by user after wrong floor exit.")
            self.active_goal_request = None
            self.robot_status = "IDLE"
            return False

    def ros_navigate_to_pose(self, pose):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]

        cy = math.cos(pose[2] * 0.5)
        sy = math.sin(pose[2] * 0.5)
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy

        self.get_logger().info("Sending goal to Nav2...")
        time.sleep(1.0) 

    def swap_maps(self, occ_rel_path, pcd_rel_path):
        """Swaps maps by resetting memory arrays, updating node parameters, and reloading triggers."""
        self.get_logger().info("Freezing localization to swap map assets...")
        self.trigger_pipeline_freeze()

        # 1. Clear accumulators
        req_empty = Empty.Request()
        self.call_service_async(self.reset_map_cli, req_empty)
        self.call_service_async(self.reset_loc_cli, req_empty)
        self.call_service_async(self.reset_bbs_cli, req_empty)

        # 2. Update parameter paths dynamically
        full_pcd = os.path.join(self.base_dir, pcd_rel_path)
        full_occ = os.path.join(self.base_dir, occ_rel_path)

        if os.path.exists(full_pcd):
            param_req = SetParameters.Request()
            param = Parameter()
            param.name = "config"
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = full_pcd
            param_req.parameters.append(param)
            self.call_service_async(self.icp_param_cli, param_req)
        else:
            self.get_logger().error(f"PCD file missing on disk: {full_pcd}")

        if os.path.exists(full_occ):
            param_req = SetParameters.Request()
            param = Parameter()
            param.name = "config"
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = full_occ
            param_req.parameters.append(param)
            self.call_service_async(self.bbs_param_cli, param_req)

        # 3. Trigger reload pipelines
        req_trigger = Trigger.Request()
        self.create_timer(0.2, lambda: self.call_service_async(self.load_map_cli, req_trigger))
        self.create_timer(0.2, lambda: self.call_service_async(self.load_bbs_cli, req_trigger))

        # 4. Clear old Nav2 costmaps
        self.clear_navigation_costmaps()
        self.get_logger().info(f"Loaded new Occupancy: {full_occ} | PCD: {full_pcd}")

    def inject_initial_pose(self, pose_xy_yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_xy_yaw[0]
        msg.pose.pose.position.y = pose_xy_yaw[1]

        cy = math.cos(pose_xy_yaw[2] * 0.5)
        sy = math.sin(pose_xy_yaw[2] * 0.5)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy

        self.initial_pose_pub.publish(msg)
        self.current_pose = pose_xy_yaw
        
        # Resume localization tracking after pose injection
        self.trigger_pipeline_resume()

    def prompt_user_ui(self, question):
        """Terminal prompt for recovery after an unexpected floor exit."""
        while True:
            try:
                ans = input(question).strip().lower()
                if ans in ['y', 'yes']:
                    return "CONTINUE"
                elif ans in ['n', 'no', 'q']:
                    return "CANCEL"
                else:
                    print("Please enter 'y' to continue navigation or 'n' to cancel.")
            except EOFError:
                return "CANCEL"

def main(args=None):
    rclpy.init(args=args)
    node = NavigationStateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()