import rclpy
import csv
import pathlib
from threading import Lock
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import numpy

ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel_keyboard'
CSV_FILE_PATH = f'processed_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
CMD_VEL_TIMEOUT_SEC = 0.1

LOGGER_TYPE = "ICP"

# POSE_TOPIC = '/amcl_pose'
# from geometry_msgs.msg import PoseWithCovarianceStamped as PoseType

POSE_TOPIC = '/range_sensor_pose'
from geometry_msgs.msg import PoseStamped as PoseType

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')
        self.csv_file_path = pathlib.Path(CSV_FILE_PATH)
        self.lock = Lock()
        self.last_cmd_time = None

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.latest_pose: PoseType = None
        self.latest_odom: Odometry = None
        
        self.first_pose_matrix = None
        self.initial_cmd_vel_rotation = None
        self.odom_matrix = numpy.eye(4)
        self.cmd_vel_matrix = numpy.eye(4)
        self.current_cmd_pose = numpy.array([0.0, 0.0, 0.0])

        self.create_subscription(PoseType, POSE_TOPIC, self.pose_callback, qos_profile)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, qos_profile)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_callback, qos_profile)

        self._initialize_csv()
        self.get_logger().info(f"Logging processed trajectories to '{self.csv_file_path.resolve()}'")

    def _initialize_csv(self):
        with open(self.csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_sec', 'timestamp_nanosec',
                'pose_px', 'pose_py', 'pose_pz', 'pose_qx', 'pose_qy', 'pose_qz', 'pose_qw',
                'odom_px', 'odom_py', 'odom_pz', 'odom_qx', 'odom_qy', 'odom_qz', 'odom_qw',
                'cmd_px', 'cmd_py', 'cmd_pz', 'cmd_qx', 'cmd_qy', 'cmd_qz', 'cmd_qw'
            ])

    def pose_callback(self, msg): self.latest_pose = msg
    def odom_callback(self, msg): self.latest_odom = msg

    def cmd_vel_callback(self, msg: Twist):
        log_row = None
        with self.lock:
            if not all([self.latest_pose, self.latest_odom]):
                return

            current_time = self.get_clock().now()
            if self.last_cmd_time is None:
                self.last_cmd_time = current_time
                return
            
            dt = (current_time - self.last_cmd_time).nanoseconds / 1e9
            
            if dt > CMD_VEL_TIMEOUT_SEC:
                self.get_logger().warn(f"No cmd_vel for {dt:.2f}s. Skipping integration step to prevent jump.")
                self.last_cmd_time = current_time
                return
                
            self.last_cmd_time = current_time
            
            if self.first_pose_matrix is None:
                self._initialize_transforms()

            ts_sec, ts_nsec = self.latest_pose.header.stamp.sec, self.latest_pose.header.stamp.nanosec

            pose_data = self._get_amcl_pose_data()
            odom_data = self._get_transformed_odom_data()
            cmd_data = self._get_transformed_cmd_vel_data(msg, dt)
            
            log_row = [ts_sec, ts_nsec] + pose_data + odom_data + cmd_data

        if log_row:
            with open(self.csv_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(log_row)
            self.get_logger().info(f"Logged processed data at {log_row[0]}.{log_row[1]}", throttle_duration_sec=1)

    def _initialize_transforms(self):
        if LOGGER_TYPE == "ICP":
            first_pose = self.latest_pose.pose
            rot = Rotation.from_quat([first_pose.orientation.x, first_pose.orientation.y, first_pose.orientation.z, first_pose.orientation.w])
            
            self.first_pose_matrix = numpy.eye(4)
            self.first_pose_matrix[:3, :3] = rot.as_matrix()
            self.first_pose_matrix[:3, 3] = numpy.array([first_pose.position.x, first_pose.position.y, first_pose.position.z])
            self.initial_cmd_vel_rotation = rot

        if LOGGER_TYPE == "AMCL":
            first_pose = self.latest_pose.pose.pose
            rot = Rotation.from_quat([first_pose.orientation.x, first_pose.orientation.y, first_pose.orientation.z, first_pose.orientation.w])
            
            self.first_pose_matrix = numpy.eye(4)
            self.first_pose_matrix[:3, :3] = rot.as_matrix()
            self.first_pose_matrix[:3, 3] = numpy.array([first_pose.position.x, first_pose.position.y, first_pose.position.z])
            self.initial_cmd_vel_rotation = rot

    def _get_amcl_pose_data(self):
        if LOGGER_TYPE == "ICP":
            p, q = self.latest_pose.pose.position, self.latest_pose.pose.orientation
        elif LOGGER_TYPE == "AMCL":
            p, q = self.latest_pose.pose.pose.position, self.latest_pose.pose.pose.orientation

        return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def _get_transformed_odom_data(self):
        p_o, q_o = self.latest_odom.pose.pose.position, self.latest_odom.pose.pose.orientation
        self.odom_matrix[:3, :3] = Rotation.from_quat([q_o.x, q_o.y, q_o.z, q_o.w]).as_matrix()
        self.odom_matrix[:3, 3] = numpy.array([p_o.x, p_o.y, p_o.z])
        
        odom_tf = self.first_pose_matrix @ self.odom_matrix
        odom_quat = Rotation.from_matrix(odom_tf[:3, :3]).as_quat()
        return [odom_tf[0, 3], odom_tf[1, 3], odom_tf[2, 3], odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]]

    def _get_transformed_cmd_vel_data(self, cmd_vel_msg: Twist, dt: float):
        vx, wz = cmd_vel_msg.linear.x, cmd_vel_msg.angular.z
        self.current_cmd_pose[0] += vx * numpy.cos(self.current_cmd_pose[2]) * dt
        self.current_cmd_pose[1] += vx * numpy.sin(self.current_cmd_pose[2]) * dt
        self.current_cmd_pose[2] += wz * dt
        
        self.cmd_vel_matrix[:3, :3] = self.initial_cmd_vel_rotation.as_matrix()
        self.cmd_vel_matrix[:3, 3] = numpy.array([self.current_cmd_pose[0], self.current_cmd_pose[1], 0])
        
        cmd_tf = self.first_pose_matrix @ self.cmd_vel_matrix
        cmd_quat = Rotation.from_matrix(cmd_tf[:3, :3]).as_quat()
        return [cmd_tf[0, 3], cmd_tf[1, 3], cmd_tf[2, 3], cmd_quat[0], cmd_quat[1], cmd_quat[2], cmd_quat[3]]

def main():
    rclpy.init()
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
