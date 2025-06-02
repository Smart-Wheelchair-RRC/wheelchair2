#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


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
        self.publish_3d_map = False  # Flag to control map publishing
        self.initial_pose_estimate = np.identity(4)
        self.map_to_odom_transform = np.identity(4)
        self.pcd_map_path = '/path/to/your/map.pcd'  # Default path to the PCD map
        self.lidar_cloud_topic = '/cloud_registered'  # Default topic for lidar scans
        self.fast_lio_odom_topic = '/Odometry'  # Default topic for Fast LIO odometry
        self.initialpose_topic = '/initialpose'  # Default topic for initial pose estimates
        self.localization_pose_topic = '/localization_pose'  # Default topic for publishing localization pose
        
        # ICP parameters
        self.icp_threshold = 0.8  # Default ICP distance threshold
        self.icp_max_iterations = 200  # Default ICP max iterations
        
        # Setup component parts
        self._setup_parameters()
        self._setup_subscribers()
        self._setup_publishers()
        self.load_pcd_map()

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

        self.declare_parameter('publish_3d_map', False)
        self.publish_3d_map = self.get_parameter('publish_3d_map').get_parameter_value().bool_value
        
        # Topic parameters
        self.declare_parameter('lidar_cloud_topic', '/cloud_registered')
        self.lidar_cloud_topic = self.get_parameter('lidar_cloud_topic').get_parameter_value().string_value
        
        self.declare_parameter('fast_lio_odom_topic', '/Odometry')
        self.fast_lio_odom_topic = self.get_parameter('fast_lio_odom_topic').get_parameter_value().string_value
        
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.initialpose_topic = self.get_parameter('initialpose_topic').get_parameter_value().string_value
        
        self.declare_parameter('localization_pose_topic', '/localization_pose')
        self.localization_pose_topic = self.get_parameter('localization_pose_topic').get_parameter_value().string_value
        
        # ICP parameters
        self.declare_parameter('icp_threshold', 0.8)
        self.icp_threshold = self.get_parameter('icp_threshold').get_parameter_value().double_value
        
        self.declare_parameter('icp_max_iterations', 200)
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').get_parameter_value().integer_value
        
        # Map cropping parameters
        self.declare_parameter('use_map_cropping', True)
        self.use_map_cropping = self.get_parameter('use_map_cropping').get_parameter_value().bool_value
        
        self.declare_parameter('map_crop_distance', 20.0)  # Default 20 meters radius
        self.map_crop_distance = self.get_parameter('map_crop_distance').get_parameter_value().double_value
    
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

        if self.publish_3d_map:

            self.map_publisher = self.create_publisher(
                msg_type=PointCloud2,
                topic='/map_by_fast_lio',
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
            map_pcd = o3d.io.read_point_cloud(self.pcd_map_path)
            if map_pcd.is_empty():
                self.get_logger().error(f"PCD map at {self.pcd_map_path} is empty.")
                map_pcd = None
            else:
                self.get_logger().info(f"Successfully loaded PCD map from {self.pcd_map_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD map: {e}")

        if self.use_map_cropping and (self.initial_pose_estimate is not None):
            # If map cropping is enabled, crop the map to the initial pose
            robot_position = self.initial_pose_estimate[:3, 3]
            cropped_map_pcd = self.crop_map_to_fov(robot_position, self.map_crop_distance, map_pcd)

            if cropped_map_pcd is None:
                self.get_logger().warn("Map cropping failed, using full map instead")
                cropped_map_pcd = map_pcd
            else:
                self.get_logger().info(f"Cropped map to {len(cropped_map_pcd.points)} points within {self.map_crop_distance}m radius")

        # downsample the map for faster processing
        self.map_pcd = cropped_map_pcd.voxel_down_sample(voxel_size=1e-3) if cropped_map_pcd else None

        if self.map_pcd is not None:
            self.get_logger().info(f"Map loaded with {len(self.map_pcd.points)} points after downsampling.")
            # Publish the map point cloud for visualization


            if self.publish_3d_map:

                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "map"  # Assuming the PCD map is in the 'map' frame

                map_msg = point_cloud2.create_cloud_xyz32(
                    header=header,
                    points=np.asarray(self.map_pcd.points)
                )
                self.map_publisher.publish(map_msg)

        else:
            self.get_logger().error("Failed to load or process the PCD map. Localization will not work without a valid map.")

    def deal_with_initial_pose(self, msg: PoseWithCovarianceStamped):
        """Callback to handle initial pose estimates for ICP."""

        self.get_logger().info("Received initial pose estimate.")

        # extract the pose from the message
        pose = msg.pose.pose
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        rotation = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()

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

        # sign up ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_pcd_down, self.map_pcd, self.icp_threshold, self.initial_pose_estimate,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_max_iterations)
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
        rotation = Rotation.from_quat([odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w]).as_matrix()
        odom_to_lidar_transform = np.identity(4)
        odom_to_lidar_transform[:3, :3] = rotation
        odom_to_lidar_transform[:3, 3] = translation

        # Combine transforms: map -> odom -> lidar_link
        print(odom_to_lidar_transform)
        final_transform = self.map_to_odom_transform @ odom_to_lidar_transform

        # Create and publish the PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "map"  # Assuming the PCD map is in the 'map' frame

        final_translation = final_transform[:3, 3]
        final_rotation = Rotation.from_matrix(final_transform[:3, :3]).as_quat()

        pose_stamped_msg.pose.position.x = final_translation[0]
        pose_stamped_msg.pose.position.y = final_translation[1]
        pose_stamped_msg.pose.position.z = final_translation[2]
        pose_stamped_msg.pose.orientation.x = final_rotation[0]
        pose_stamped_msg.pose.orientation.y = final_rotation[1]
        pose_stamped_msg.pose.orientation.z = final_rotation[2]
        pose_stamped_msg.pose.orientation.w = final_rotation[3]

        self.pub_localization.publish(pose_stamped_msg)
    
    def crop_map_to_fov(self, robot_position, max_distance, map_pcd):
        """
        Crops the map point cloud to include only points within a specified distance from the robot.
        
        This function filters the global map to only include points within a 360-degree
        field of view around the robot and up to a maximum distance from the robot's
        current position. This optimization reduces the computational load during ICP
        registration by limiting the point cloud size while still maintaining accuracy
        in localization, especially in large environments.
        
        Args:
            robot_position (numpy.ndarray): The [x, y, z] position of the robot in the map frame.
            max_distance (float): The maximum distance (in meters) from the robot to include points.
            
        Returns:
            open3d.geometry.PointCloud: A cropped version of the map point cloud.
        """
        if map_pcd is None:
            self.get_logger().warn("Cannot crop map: PCD map not loaded.")
            return None
        
        # Create a copy of the map to avoid modifying the original
        cropped_map = o3d.geometry.PointCloud()
        
        # Get map points as numpy array
        points = np.asarray(map_pcd.points)
        
        # Calculate distances from each point to the robot position
        distances = np.sqrt(np.sum((points - robot_position) ** 2, axis=1))
        
        # Filter points within the specified distance
        mask = distances <= max_distance
        filtered_points = points[mask]
        
        # Create a new point cloud with the filtered points
        cropped_map.points = o3d.utility.Vector3dVector(filtered_points)
        
        # If the original map had colors or normals, copy those as well
        if map_pcd.has_colors():
            colors = np.asarray(map_pcd.colors)
            cropped_map.colors = o3d.utility.Vector3dVector(colors[mask])
            
        if map_pcd.has_normals():
            normals = np.asarray(map_pcd.normals)
            cropped_map.normals = o3d.utility.Vector3dVector(normals[mask])
        
        # Calculate reduction percentage for logging
        if len(points) > 0:
            reduction_percent = (1.0 - len(filtered_points) / len(points)) * 100
            self.get_logger().debug(f"Cropped map from {len(points)} to {len(filtered_points)} points "
                                   f"({reduction_percent:.2f}% reduction) within {max_distance}m radius")
        else:
            self.get_logger().warn("Map appears to be empty before cropping")
        
        return cropped_map


def main(args=None):
    rclpy.init(args=args)
    node = FASTLIOLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()