import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from std_msgs.msg import Int32, Bool
from std_srvs.srv import SetBool, Empty, Trigger
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
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
        self.declare_parameter('initial_floor', 0)

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
        self.robot_status = "IDLE"
        self.current_pose = [0.0, 0.0, 0.0]
        self.active_goal_request = None
        self.detected_floor_from_terminal = None

        # --- ROS 2 SERVICE CLIENTS ---
        self.pause_odom_cli = self.create_client(SetBool, 'pause_odom')
        self.reset_map_cli = self.create_client(Empty, 'reset_map')
        self.pause_loc_cli = self.create_client(SetBool, 'pause_localization')
        self.reset_loc_cli = self.create_client(Empty, 'reset_localization')
        self.load_map_cli = self.create_client(Trigger, 'load_map')
        self.pause_bbs_cli = self.create_client(SetBool, 'pause_3dbbs')
        self.reset_bbs_cli = self.create_client(Empty, 'reset_3dbbs')
        self.load_bbs_cli = self.create_client(Trigger, 'load_3dbbs_config')
        
        self.map_server_load_cli = self.create_client(LoadMap, '/map_server/load_map')

        self.icp_param_cli = self.create_client(SetParameters, '/icp_localization/set_parameters')
        self.bbs_param_cli = self.create_client(SetParameters, '/gpu_ros2_test_rviz2/set_parameters')

        self.clear_local_costmap_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # --- ROS 2 INTERFACES ---
        self.click_loc_pub = self.create_publisher(Bool, '/click_loc', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.floor_sub = self.create_subscription(Int32, '/detected_floor_id', self.floor_detection_cb, 10)

        self.get_logger().info(f"State Manager Online. Initialized on {self.current_building} -> Floor {self.current_floor}")

    # def floor_detection_cb(self, msg):
    #     self.detected_floor_from_terminal = int(msg.data)
    #     self.get_logger().info(f"Received confirmed floor ID from terminal: Floor {self.detected_floor_from_terminal}")
    
    def floor_detection_cb(self, msg):
        self.detected_floor_from_terminal = int(msg.data)
        self.get_logger().info(f"Received confirmed floor ID from terminal: Floor {self.detected_floor_from_terminal}")

        # --- NEW: Manual Teleoperation Trap ---
        # If the system isn't running an autonomous route, assume manual teleoperation
        if self.robot_status == "IDLE" and self.active_goal_request is None:
            new_floor = self.detected_floor_from_terminal
            if new_floor == self.current_floor:
                self.get_logger().info("Floor hasn't changed. Map swap skipped.")
                return

            self.get_logger().info(f"🕹️ Manual teleop floor transition detected! Preparing to load Floor {new_floor}...")
            self.handle_manual_floor_transition(new_floor)
            
    def handle_manual_floor_transition(self, new_floor):
        self.robot_status = "PAUSED"
        
        # Verify the entered floor exists in the current building's YAML
        floor_dict = self.config['buildings'][self.current_building]['floors']
        if new_floor not in floor_dict:
            self.get_logger().error(f"Floor {new_floor} does not exist in {self.current_building} config! Aborting swap.")
            self.robot_status = "IDLE"
            return
            
        floor_data = floor_dict[new_floor]
        
        # 1. Safely pause and wipe the odometry/map buffers from the previous floor
        self.pause_and_wipe()
        
        # 2. Reload the new floor parameters and trigger the 3D-BBS auto-localization sequence
        self.reload_and_resume(
            new_floor,
            floor_data['maps']['pcd'],
            floor_data['maps']['occupancy'],
            self.config.get('base_bbs_yaml', '')
        )
        
        # 3. Update the state machine and return control
        self.current_floor = new_floor
        self.current_elevator = None
        self.robot_status = "IDLE"
        self.get_logger().info("✅ Manual map swap and global localization complete! You may continue driving.")

    def call_service_async(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return None
        return client.call_async(request)

    def pause_and_wipe(self):
        """Mirrors the master_pause_and_wipe.sh script."""
        self.get_logger().info("⏸️  1. Pausing ICP, DLIO, and 3D-BBS...")
        req_bool = SetBool.Request()
        req_bool.data = True
        self.call_service_async(self.pause_loc_cli, req_bool)
        self.call_service_async(self.pause_odom_cli, req_bool)
        self.call_service_async(self.pause_bbs_cli, req_bool)

        self.get_logger().info("🗑️  2. Resetting maps, buffers, and accumulators...")
        req_empty = Empty.Request()
        self.call_service_async(self.reset_loc_cli, req_empty)
        self.call_service_async(self.reset_map_cli, req_empty)
        self.call_service_async(self.reset_bbs_cli, req_empty)

        self.get_logger().info("🧹 3. Clearing Nav2 Local Costmap...")
        req_clear = ClearEntireCostmap.Request()
        self.call_service_async(self.clear_local_costmap_cli, req_clear)
        self.get_logger().info("✅ Robot paused and memory wiped.")

    def reload_and_resume(self, new_floor_id, pcd_rel_path, occ_rel_path, base_bbs_yaml):
        """Mirrors the sequenced master_reload_and_resume.sh script."""
        full_pcd = os.path.join(self.base_dir, pcd_rel_path)
        full_occ = os.path.join(self.base_dir, occ_rel_path)
        base_bbs_yaml_full = os.path.join(self.base_dir, base_bbs_yaml)

        self.get_logger().info("⚙️  PREPARATION: Updating Node Parameters...")
        
        # 1. Update ICP Parameter (Corrected to pcd_file_path)
        param_req = SetParameters.Request()
        param = Parameter(name="pcd_file_path", value=ParameterType(type=ParameterType.PARAMETER_STRING, string_value=full_pcd))
        param_req.parameters.append(param)
        self.call_service_async(self.icp_param_cli, param_req)

        # 2. Dynamic YAML generation for 3D-BBS
        temp_yaml_path = "/tmp/current_floor_3dbbs.yaml"
        target_dir = os.path.dirname(full_pcd)
        try:
            with open(base_bbs_yaml_full, 'r') as f:
                config_data = yaml.safe_load(f)
            config_data['target_clouds'] = target_dir
            with open(temp_yaml_path, 'w') as f:
                yaml.safe_dump(config_data, f)
                
            param_req_bbs = SetParameters.Request()
            param_bbs = Parameter(name="config", value=ParameterType(type=ParameterType.PARAMETER_STRING, string_value=temp_yaml_path))
            param_req_bbs.parameters.append(param_bbs)
            self.call_service_async(self.bbs_param_cli, param_req_bbs)
        except Exception as e:
            self.get_logger().error(f"Failed to generate dynamic YAML: {e}")

        # --- THE SEQUENTIAL STARTUP ---
        req_trigger = Trigger.Request()
        req_unpause = SetBool.Request(data=False)

        # STEP 1: DLIO
        self.get_logger().info("▶️  STEP 1: Resuming DLIO Odometry")
        self.call_service_async(self.pause_odom_cli, req_unpause)
        time.sleep(3.0)

        # STEP 2: 3D-BBS Global Localization
        self.get_logger().info("🎯 STEP 2: Reloading & Resuming 3D-BBS")
        self.call_service_async(self.load_bbs_cli, req_trigger)
        self.call_service_async(self.pause_bbs_cli, req_unpause)
        
        self.get_logger().info("⏳ Waiting 3s for LiDAR/IMU buffers to fill...")
        time.sleep(3.0)
        
        self.get_logger().info("🔍 Triggering Global Pose Estimation (Simulating /click_loc)...")
        click_msg = Bool(data=True)
        self.click_loc_pub.publish(click_msg)
        
        self.get_logger().info("⏳ Waiting 3s for pose calculation to finish...")
        time.sleep(3.0)

        # STEP 3: ICP and Nav2 Map Server
        self.get_logger().info("🗺️  STEP 3: Reloading ICP and Nav2 Map Server")
        map_req = LoadMap.Request(map_url=full_occ)
        self.call_service_async(self.map_server_load_cli, map_req)
        
        self.call_service_async(self.load_map_cli, req_trigger)
        self.call_service_async(self.pause_loc_cli, req_unpause)

        self.get_logger().info(f"✅ Floor {new_floor_id} transition complete! System Online.")
        self.current_floor = int(new_floor_id)

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
                
            elif step['action'] == 'wait_and_enter_elevator':
                success = self.interact_with_elevator(step['shaft_name'], step['target_floor'])
                if not success:
                    return

            elif step['action'] == 'pause_and_wipe_navigation':
                self.pause_and_wipe()

            elif step['action'] == 'reload_and_resume_navigation':
                self.reload_and_resume(
                    step['new_floor_id'],
                    step['new_pcd'],
                    step['new_occupancy'],
                    step['base_bbs_yaml']
                )
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
        self.robot_status = "PAUSED"
        
        floor_dict = self.config['buildings'][self.current_building]['floors']
        if actual_floor not in floor_dict:
            self.get_logger().error(f"Floor {actual_floor} does not exist in YAML!")
            self.active_goal_request = None
            self.robot_status = "IDLE"
            return False

        floor_data = floor_dict[actual_floor]
        exit_pose = floor_data['elevators'][self.current_elevator]['exit_initial_pose']
        
        self.get_logger().info(f"Emergency localization: Preparing Floor {actual_floor}...")
        
        # Wipe the corrupted odometry tracking from the wrong trip
        self.pause_and_wipe()
        
        # Reload the floor we actually landed on
        self.reload_and_resume(
            actual_floor,
            floor_data['maps']['pcd'],
            floor_data['maps']['occupancy'],
            self.config.get('base_bbs_yaml', '')
        )
        
        self.current_pose = exit_pose
        self.current_elevator = None
        
        user_choice = self.prompt_user_ui(
            f"\n[ALERT] Exited on Floor {actual_floor}, but goal is on Floor {self.active_goal_request['floor']}."
            f"\n>>> Do you still want to navigate to Floor {self.active_goal_request['floor']}? (y/n): "
        )
        
        if user_choice == "CONTINUE":
            self.get_logger().info("Generating recovery route from current floor...")
            self.replan_and_execute()
            return False
        else:
            self.get_logger().info("Navigation cancelled.")
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

    def prompt_user_ui(self, question):
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