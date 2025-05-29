#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class FASTLIOLocalizationNode(Node):
    """
    FAST-LIO based localization node that uses ICP to align lidar scans with a pre-built PCD map.
    
    Design principles:
    - Maintains separation between parameter setup, subscriptions, and publishers
    - Uses Open3D for efficient point cloud processing and ICP alignment
    - Transforms between coordinate frames to provide map-relative localization
    """

    def __init__(self):
        super().__init__('localization')

        # Initialize class attributes
        self.map_pcd = None
        self.current_odometry = None
        self.initial_pose_estimate = np.identity(4)
        self.map_to_odom_transform = np.identity(4)
        self.pcd_map_path = '/path/to/your/map.pcd'  # Default path to the PCD map
        self.lidar_cloud_topic = '/cloud_registered'  # Default topic for lidar scans
        self.fast_lio_odom_topic = '/Odometry'  # Default topic for Fast LIO odometry
        self.initialpose_topic = '/initialpose'  # Default topic for initial pose estimates
        self.localization_pose_topic = '/localization_pose'  # Default topic for publishing localization pose
        
        # Setup component parts
        self._setup_parameters()
        self.load_pcd_map()
        self._setup_subscribers()
        self._setup_publishers()

        self.get_logger().info(f"Initialized.")
        self.get_logger().info(f"\tlidar_cloud topic: {self.lidar_cloud_topic}")
        self.get_logger().info(f"\tfast_lio_odom topic: {self.fast_lio_odom_topic}")
        self.get_logger().info(f"\tinitialpose topic: {self.initialpose_topic}")
        self.get_logger().info(f"\tlocalization_pose topic: {self.localization_pose_topic}")
    
    def _setup_parameters(self):
        """
        Sets up ROS parameters with defaults.
        
        The node uses separate topics to allow for flexible configuration in different robot setups.
        """
        # PCD Map Path
        self.declare_parameter('pcd_map_path', self.pcd_map_path)
        self.pcd_map_path = self.get_parameter('pcd_map_path').get_parameter_value().string_value
        
        # Topic parameters
        self.declare_parameter('lidar_cloud_topic', '/cloud_registered')
        self.lidar_cloud_topic = self.get_parameter('lidar_cloud_topic').get_parameter_value().string_value
        
        self.declare_parameter('fast_lio_odom_topic', '/Odometry')
        self.fast_lio_odom_topic = self.get_parameter('fast_lio_odom_topic').get_parameter_value().string_value
        
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.initialpose_topic = self.get_parameter('initialpose_topic').get_parameter_value().string_value
        
        self.declare_parameter('localization_pose_topic', '/localization_pose')
        self.localization_pose_topic = self.get_parameter('localization_pose_topic').get_parameter_value().string_value
    
    def _setup_subscribers(self):
        """Set up all subscribers"""
        # Define QoS Profile
        DEFAULT_QOS = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriptions directly without using a dictionary
        self.subscription_scan = self.create_subscription(
            msg_type=PointCloud2,
            topic=self.lidar_cloud_topic,
            callback=self.deal_with_current_scan,
            qos_profile=DEFAULT_QOS
        )
        
        self.subscription_odom = self.create_subscription(
            msg_type=Odometry,
            topic=self.fast_lio_odom_topic,
            callback=self.deal_with_current_odometry,
            qos_profile=DEFAULT_QOS
        )
        
        self.subscription_initial_pose = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic=self.initialpose_topic,
            callback=self.deal_with_initial_pose,
            qos_profile=DEFAULT_QOS
        )
    
    def _setup_publishers(self):
        """Set up all publishers"""
        self.pub_localization = self.create_publisher(
            msg_type=PoseStamped,
            topic=self.localization_pose_topic,
            qos_profile=QoSProfile(
                depth=10,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

    def load_pcd_map(self):
        """Loads the PCD map from the specified file path."""
        try:
            self.map_pcd = o3d.io.read_point_cloud(self.pcd_map_path)
            if self.map_pcd.is_empty():
                self.get_logger().error(f"PCD map at {self.pcd_map_path} is empty.")
                self.map_pcd = None
            else:
                self.get_logger().info(f"Successfully loaded PCD map from {self.pcd_map_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD map: {e}")

    def deal_with_initial_pose(self, msg: PoseWithCovarianceStamped):
        """Callback to handle initial pose estimates for ICP."""

        self.get_logger().info("Received initial pose estimate.")

        # extract the pose from the message
        pose = msg.pose.pose
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        rotation = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()

        # create the initial pose estimate matrix
        self.initial_pose_estimate = np.identity(4)
        self.initial_pose_estimate[:3, :3] = rotation
        self.initial_pose_estimate[:3, 3] = translation
        self.map_to_odom_transform = self.initial_pose_estimate

    def deal_with_current_odometry(self, msg: Odometry):
        """Stores the latest odometry from Fast LIO."""
        self.current_odometry = msg

    def deal_with_current_scan(self, msg: PointCloud2):
        """Processes the current Lidar scan to perform localization."""
        if self.map_pcd is None:
            self.get_logger().warn("PCD map not loaded, skipping scan processing.")
            return

        # unpack the lidar scan point cloud data
        pc_data_structured = point_cloud2.read_points(msg, field_names="xyz", skip_nans=True)
        pc_data = np.vstack([pc_data_structured['x'], pc_data_structured['y'], pc_data_structured['z']]).T
        source_pcd = o3d.geometry.PointCloud()
        source_pcd.points = o3d.utility.Vector3dVector(pc_data)

        # downsample the point clouds for faster processing
        source_pcd_down = source_pcd.voxel_down_sample(voxel_size=0.2)
        if not hasattr(self, 'map_pcd_down'):
            self.map_pcd_down = self.map_pcd.voxel_down_sample(voxel_size=0.2)

        # sign up ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_pcd_down, self.map_pcd_down, 0.5, self.initial_pose_estimate,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
        )

        self.map_to_odom_transform = reg_p2p.transformation

        # Update the initial guess for the next iteration
        self.initial_pose_estimate = reg_p2p.transformation

        self.publish_localization_pose()

    def publish_localization_pose(self):
        """Calculates and publishes the robot's pose in the map frame."""
        if self.current_odometry is None:
            return

        # Get the transform from the odometry message (odom -> lidar_link)
        odom_pose = self.current_odometry.pose.pose
        translation = np.array([odom_pose.position.x, odom_pose.position.y, odom_pose.position.z])
        rotation = R.from_quat([odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w]).as_matrix()
        odom_to_lidar_transform = np.identity(4)
        odom_to_lidar_transform[:3, :3] = rotation
        odom_to_lidar_transform[:3, 3] = translation

        # Combine transforms: map -> odom -> lidar_link
        final_transform = self.map_to_odom_transform @ odom_to_lidar_transform

        # Create and publish the PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "map"  # Assuming the PCD map is in the 'map' frame

        final_translation = final_transform[:3, 3]
        final_rotation = R.from_matrix(final_transform[:3, :3]).as_quat()

        pose_stamped_msg.pose.position.x = final_translation[0]
        pose_stamped_msg.pose.position.y = final_translation[1]
        pose_stamped_msg.pose.position.z = final_translation[2]
        pose_stamped_msg.pose.orientation.x = final_rotation[0]
        pose_stamped_msg.pose.orientation.y = final_rotation[1]
        pose_stamped_msg.pose.orientation.z = final_rotation[2]
        pose_stamped_msg.pose.orientation.w = final_rotation[3]

        self.pub_localization.publish(pose_stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FASTLIOLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()