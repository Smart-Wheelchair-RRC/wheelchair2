import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int32, Bool, String
from std_srvs.srv import SetBool, Empty, Trigger
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
import yaml
import json
import os
import time
import math
import traceback

from elevator_navigation.topological_router import TopologicalGraphRouter

class NavigationStateManager(Node):
    def __init__(self):
        super().__init__('nav_state_manager')
        
        # Enable reentrant execution for concurrent actions/callbacks
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('yaml_config_path', '/home/container_user/wheelchair2/src/elevator_navigation/config/campus_config.yaml')
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
        self.pause_odom_cli = self.create_client(SetBool, 'pause_odom', callback_group=self.cb_group)
        self.reset_map_cli = self.create_client(Empty, 'reset_map', callback_group=self.cb_group)
        self.pause_loc_cli = self.create_client(SetBool, 'pause_localization', callback_group=self.cb_group)
        self.reset_loc_cli = self.create_client(Empty, 'reset_localization', callback_group=self.cb_group)
        self.load_map_cli = self.create_client(Trigger, 'load_map', callback_group=self.cb_group)
        self.pause_bbs_cli = self.create_client(SetBool, 'pause_3dbbs', callback_group=self.cb_group)
        self.reset_bbs_cli = self.create_client(Empty, 'reset_3dbbs', callback_group=self.cb_group)
        self.load_bbs_cli = self.create_client(Trigger, 'load_3dbbs_config', callback_group=self.cb_group)
        
        self.map_server_load_cli = self.create_client(LoadMap, '/map_server/load_map', callback_group=self.cb_group)

        self.icp_param_cli = self.create_client(SetParameters, '/icp_localization/set_parameters', callback_group=self.cb_group)
        self.bbs_param_cli = self.create_client(SetParameters, '/gpu_ros2_test_rviz2/set_parameters', callback_group=self.cb_group)

        self.clear_local_costmap_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', callback_group=self.cb_group)

        # --- ROS 2 INTERFACES ---
        self.click_loc_pub = self.create_publisher(Bool, '/click_loc', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        
        # Subscriptions
        self.floor_sub = self.create_subscription(Int32, '/detected_floor_id', self.floor_detection_cb, 10, callback_group=self.cb_group)
        self.goal_sub = self.create_subscription(String, '/send_navigation_goal', self.terminal_goal_cb, 10, callback_group=self.cb_group)

        self.get_logger().info(f"State Manager Online. Initialized on {self.current_building} -> Floor {self.current_floor}")

    def terminal_goal_cb(self, msg):
        try:
            data = json.loads(msg.data)
            target_bldg = data.get("bldg", self.current_building)
            target_floor = int(data["floor"])
            target_pose = [float(x) for x in data["pose"]]

            self.get_logger().info(f"🎯 Goal Received via Terminal: {target_bldg} -> Floor {target_floor} @ {target_pose}")
            self.send_user_goal(target_bldg, target_floor, target_pose)
        except Exception as e:
            self.get_logger().error(f"Error processing goal: {e}\n{traceback.format_exc()}")

    def floor_detection_cb(self, msg):
        self.detected_floor_from_terminal = int(msg.data)
        self.get_logger().info(f"Received confirmed floor ID from terminal: Floor {self.detected_floor_from_terminal}")

        # Manual Teleoperation Transition Trap
        if self.robot_status == "IDLE" and self.active_goal_request is None:
            new_floor = self.detected_floor_from_terminal
            if new_floor == self.current_floor:
                self.get_logger().info("Floor unchanged. Map reload skipped.")
                return

            self.get_logger().info(f"🕹️ Manual teleop floor transition detected! Loading Floor {new_floor}...")
            self.handle_manual_floor_transition(new_floor)

    def handle_manual_floor_transition(self, new_floor):
        self.robot_status = "PAUSED"
        
        floor_dict = self.config['buildings'][self.current_building]['floors']
        if new_floor not in floor_dict:
            self.get_logger().error(f"Floor {new_floor} does not exist in {self.current_building} config!")
            self.robot_status = "IDLE"
            return
            
        floor_data = floor_dict[new_floor]
        
        # Extract default exit pose for this floor
        exit_pose = None
        elevators = floor_data.get('elevators', {})
        if self.current_elevator and self.current_elevator in elevators:
            exit_pose = elevators[self.current_elevator].get('exit_initial_pose')
        elif elevators:
            first_elev = next(iter(elevators.values()))
            exit_pose = first_elev.get('exit_initial_pose')
        
        self.pause_and_wipe()
        self.reload_and_resume(
            new_floor,
            floor_data['maps']['pcd'],
            floor_data['maps']['occupancy'],
            self.config.get('system', {}).get('base_bbs_yaml', ''),
            exit_pose=exit_pose
        )
        
        self.current_floor = new_floor
        self.current_elevator = None
        self.robot_status = "IDLE"
        self.get_logger().info("✅ Manual map swap & global localization complete. Ready to drive.")

    def call_service_async(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return None
        return client.call_async(request)

    def pause_and_wipe(self):
        """Pauses pipelines, resets state arrays, and clears costmaps."""
        self.get_logger().info("⏸️  1. Pausing ICP, DLIO, and 3D-BBS...")
        req_bool = SetBool.Request(data=True)
        self.call_service_async(self.pause_loc_cli, req_bool)
        self.call_service_async(self.pause_odom_cli, req_bool)
        self.call_service_async(self.pause_bbs_cli, req_bool)
        time.sleep(1.0)

        self.get_logger().info("🗑️  2. Resetting maps, buffers, and accumulators...")
        req_empty = Empty.Request()
        self.call_service_async(self.reset_loc_cli, req_empty)
        self.call_service_async(self.reset_map_cli, req_empty)
        self.call_service_async(self.reset_bbs_cli, req_empty)
        time.sleep(1.0)

        self.get_logger().info("🧹 3. Clearing Nav2 Local Costmap...")
        req_clear = ClearEntireCostmap.Request()
        self.call_service_async(self.clear_local_costmap_cli, req_clear)
        time.sleep(0.5)

        self.get_logger().info("✅ Navigation paused and memory wiped.")

    def inject_initial_pose(self, pose_xy_yaw):
        """Publishes the exact elevator exit pose to /initialpose to anchor ICP localization."""
        if pose_xy_yaw is None:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose_xy_yaw[0])
        msg.pose.pose.position.y = float(pose_xy_yaw[1])
        msg.pose.pose.position.z = 0.0

        yaw = float(pose_xy_yaw[2])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy

        # Standard initialpose covariance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[14] = 0.25
        msg.pose.covariance[21] = 0.06853891945200942
        msg.pose.covariance[28] = 0.06853891945200942
        msg.pose.covariance[35] = 0.06853891945200942

        self.get_logger().info(f"📌 Setting initial pose to /initialpose: [x={pose_xy_yaw[0]:.2f}, y={pose_xy_yaw[1]:.2f}, yaw={yaw:.2f}]")
        self.initial_pose_pub.publish(msg)
        self.current_pose = pose_xy_yaw

    def reload_and_resume(self, new_floor_id, pcd_rel_path, occ_rel_path, base_bbs_yaml, exit_pose=None):
        full_pcd = os.path.join(self.base_dir, pcd_rel_path)
        full_occ = os.path.join(self.base_dir, occ_rel_path)
        base_bbs_yaml_full = os.path.join(self.base_dir, base_bbs_yaml)

        self.get_logger().info("⚙️  PREPARATION: Updating Node Parameters...")
        
        # 1. Update ICP Parameter
        param_req = SetParameters.Request()
        param_val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=full_pcd)
        param_req.parameters.append(Parameter(name="pcd_path", value=param_val))
        param_req.parameters.append(Parameter(name="pcd_file_path", value=param_val))
        param_req.parameters.append(Parameter(name="config", value=param_val))
        self.call_service_async(self.icp_param_cli, param_req)

        # 2. Dynamic YAML generation for 3D-BBS
        temp_yaml_path = "/tmp/current_floor_3dbbs.yaml"
        target_dir = os.path.dirname(full_pcd)
        try:
            config_data = {}
            if base_bbs_yaml_full and os.path.isfile(base_bbs_yaml_full):
                with open(base_bbs_yaml_full, 'r') as f:
                    config_data = yaml.safe_load(f) or {}
            
            config_data['target_clouds'] = target_dir

            with open(temp_yaml_path, 'w') as f:
                yaml.safe_dump(config_data, f)
                
            param_req_bbs = SetParameters.Request()
            param_bbs_val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=temp_yaml_path)
            param_bbs = Parameter(name="config", value=param_bbs_val)
            param_req_bbs.parameters.append(param_bbs)
            self.call_service_async(self.bbs_param_cli, param_req_bbs)
            self.get_logger().info(f"✅ Generated 3D-BBS dynamic config at {temp_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate dynamic YAML: {e}\n{traceback.format_exc()}")

        time.sleep(1.0)
        req_trigger = Trigger.Request()
        req_unpause = SetBool.Request(data=False)

        # STEP 1: DLIO
        self.get_logger().info("▶️  STEP 1: Resuming DLIO Odometry")
        self.call_service_async(self.pause_odom_cli, req_unpause)
        time.sleep(3.0)

        # STEP 2: 3D-BBS Global Localization
        self.get_logger().info("🎯 STEP 2: Reloading & Resuming 3D-BBS")
        self.call_service_async(self.load_bbs_cli, req_trigger)
        time.sleep(1.5)
        self.call_service_async(self.pause_bbs_cli, req_unpause)
        
        self.get_logger().info("⏳ Waiting 3s for sensor buffers to fill...")
        time.sleep(3.0)
        
        self.get_logger().info("🔍 Triggering 3D-BBS Pose Estimation (/click_loc)...")
        click_msg = Bool(data=True)
        self.click_loc_pub.publish(click_msg)
        
        self.get_logger().info("⏳ Waiting 4s for global localization convergence...")
        time.sleep(4.0)

        # STEP 3: ICP and Nav2 Map Server
        self.get_logger().info("🗺️  STEP 3: Reloading ICP and Nav2 Map Server")
        map_req = LoadMap.Request(map_url=full_occ)
        self.call_service_async(self.map_server_load_cli, map_req)
        time.sleep(1.5)
        
        # Trigger ICP reload with updated parameter
        self.call_service_async(self.load_map_cli, req_trigger)
        time.sleep(1.5)
        self.call_service_async(self.pause_loc_cli, req_unpause)
        time.sleep(1.5)

        # STEP 4: Fallback Initial Pose Injection (Anchors ICP to Elevator Exit)
        if exit_pose is not None:
            self.get_logger().info("📍 Overriding initial pose with known exit coordinates from config...")
            self.inject_initial_pose(exit_pose)
            time.sleep(1.0)

        self.get_logger().info(f"✅ Floor {new_floor_id} fully loaded and initialized.")
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
                success = self.ros_navigate_to_pose(step['pose'])
                if not success:
                    self.get_logger().error("Navigation failed! Aborting remaining route execution.")
                    self.active_goal_request = None
                    self.robot_status = "IDLE"
                    return
                
            elif step['action'] == 'wait_and_enter_elevator':
                success = self.interact_with_elevator(step['shaft_name'], step['target_floor'])
                if not success:
                    return

            elif step['action'] == 'pause_and_wipe_navigation':
                self.pause_and_wipe()

            elif step['action'] == 'reload_and_resume_navigation':
                # Retrieve exit pose from step dict or lookup from config
                exit_pose = step.get('exit_pose', None)
                if exit_pose is None and self.current_elevator:
                    try:
                        exit_pose = self.config['buildings'][self.current_building]['floors'][step['new_floor_id']]['elevators'][self.current_elevator]['exit_initial_pose']
                    except Exception:
                        pass

                self.reload_and_resume(
                    step['new_floor_id'],
                    step['new_pcd'],
                    step['new_occupancy'],
                    step['base_bbs_yaml'],
                    exit_pose=exit_pose
                )
                self.current_elevator = None
                self.robot_status = "IDLE"

        self.get_logger().info("🎯 Target Reached Successfully!")
        self.active_goal_request = None

    def interact_with_elevator(self, shaft_name, target_floor):
        self.robot_status = "IN_ELEVATOR"
        self.current_floor = None 
        self.current_elevator = shaft_name
        self.detected_floor_from_terminal = None
        self.get_logger().info(f"Riding {shaft_name} to Floor {target_floor}. Awaiting floor detection...")

        while self.robot_status == "IN_ELEVATOR" and rclpy.ok():
            time.sleep(0.5)
            
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
            self.get_logger().error(f"Floor {actual_floor} does not exist in YAML config!")
            self.active_goal_request = None
            self.robot_status = "IDLE"
            return False

        floor_data = floor_dict[actual_floor]
        exit_pose = floor_data['elevators'][self.current_elevator]['exit_initial_pose']
        
        self.get_logger().info(f"Emergency recovery: Loading Floor {actual_floor}...")
        self.pause_and_wipe()
        self.reload_and_resume(
            actual_floor,
            floor_data['maps']['pcd'],
            floor_data['maps']['occupancy'],
            self.config.get('system', {}).get('base_bbs_yaml', ''),
            exit_pose=exit_pose
        )
        
        self.current_pose = exit_pose
        self.current_elevator = None
        
        user_choice = self.prompt_user_ui(
            f"\n[ALERT] Exited on Floor {actual_floor}, but goal is on Floor {self.active_goal_request['floor']}."
            f"\n>>> Do you still want to navigate to Floor {self.active_goal_request['floor']}? (y/n): "
        )
        
        if user_choice == "CONTINUE":
            self.get_logger().info("Re-routing from current floor...")
            self.replan_and_execute()
            return False
        else:
            self.get_logger().info("Navigation cancelled by user.")
            self.active_goal_request = None
            self.robot_status = "IDLE"
            return False

    def ros_navigate_to_pose(self, pose):
        """Dispatches an action goal to Nav2 and waits using multi-threaded execution."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server (/navigate_to_pose) unavailable!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pose[0])
        goal_msg.pose.pose.position.y = float(pose[1])
        goal_msg.pose.pose.position.z = 0.0

        yaw = float(pose[2])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy

        self.get_logger().info(f"Sending goal to Nav2: [x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={yaw:.2f}]")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        # Wait for acceptance (handled cleanly by MultiThreadedExecutor)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal request!")
            return False

        self.get_logger().info("Nav2 accepted goal. Driving to target...")
        result_future = goal_handle.get_result_async()

        # Wait until goal finishes
        while rclpy.ok() and not result_future.done():
            time.sleep(0.2)

        if not result_future.done():
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Successfully arrived at navigation waypoint.")
            return True
        else:
            self.get_logger().warn(f"Navigation ended with status code: {status}")
            return False

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
    
    # Use MultiThreadedExecutor so callbacks and action clients run on separate threads without deadlock
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()