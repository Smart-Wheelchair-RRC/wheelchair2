import numpy
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import open3d
import tf_transformations


DEFAULT_QOS = QoSProfile(depth=10)


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("map_topic", "/map"),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize necessary class variables
        self.current_scan = None
        self.current_odometry = None
        self.initial_pose = numpy.eye(4)
        self.global_map = open3d.geometry.PointCloud()
        self.T_map_to_odom = numpy.eye(4)
        self.initialized = False
        self.map_received = False

        self.subscriptions_created = {
            "scan": self.create_subscription(
                msg_type=PointCloud2,
                topic="/cloud_registered",
                callback=self.deal_with_current_scan,
                qos_profile=DEFAULT_QOS
            ),
            "odometry": self.create_subscription(
                msg_type=Odometry,
                topic="/Odometry",
                callback=self.deal_with_current_odometry,
                qos_profile=DEFAULT_QOS
            ),
            "initialpose": self.create_subscription(
                msg_type=PoseWithCovarianceStamped,
                topic="/initialpose",
                callback=self.deal_with_initial_pose,
                qos_profile=DEFAULT_QOS
            ),
            "map": self.create_subscription(
                msg_type=OccupancyGrid,
                topic=self.get_parameter("map_topic").value,
                callback=self.deal_with_global_map,
                qos_profile=QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            ),
        }

        self.pub_localization = self.create_publisher(
            msg_type=PoseStamped,
            topic="/localization_pose",
            qos_profile=QoSProfile(
                depth=10,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

        self.get_logger().info(f"Waiting for map on topic {self.get_parameter('map_topic').value}...")

    def deal_with_global_map(self, message: OccupancyGrid):
        """Process the OccupancyGrid map and convert it to a point cloud"""
        self.get_logger().info(f"Received map with size {message.info.width}x{message.info.height}, resolution: {message.info.resolution}")
        
        # Extract occupied cells from the occupancy grid
        points = []
        for y in range(message.info.height):
            for x in range(message.info.width):
                # Check if cell is occupied (value > 0)
                index = y * message.info.width + x
                if 0 <= index < len(message.data) and message.data[index] > 0:
                    # Convert grid coordinates to world coordinates
                    world_x = message.info.origin.position.x + x * message.info.resolution
                    world_y = message.info.origin.position.y + y * message.info.resolution
                    world_z = message.info.origin.position.z  # Usually 0
                    
                    points.append([world_x, world_y, world_z])
        
        # Convert points to numpy array
        if not points:
            self.get_logger().warn("Received map has no occupied cells")
            return

        # Create Open3D point cloud from occupied cells
        self.global_map = open3d.geometry.PointCloud()
        self.global_map.points = open3d.utility.Vector3dVector(numpy.array(points))
        
        # Downsample point cloud to reduce computational load
        self.global_map = self.voxel_down_sample(
            self.global_map, 
            self.get_parameter("map_voxel_size").value
        )
        
        self.get_logger().info(f"Created point cloud map with {len(numpy.asarray(self.global_map.points))} points")
        self.map_received = True

    def deal_with_current_scan(self, message: PointCloud2):
        """Process the incoming point cloud scan"""
        # Convert the point cloud message to a numpy array
        pc_array = point_cloud2.read_points_numpy(message, field_names="xyz", reshape_organized_cloud=True)
        
        # Convert to open3d PointCloud
        self.current_scan = open3d.geometry.PointCloud()
        self.current_scan.points = open3d.utility.Vector3dVector(pc_array)

    def deal_with_current_odometry(self, message: Odometry):
        """Save the current odometry message"""
        self.current_odometry = message

    def deal_with_initial_pose(self, message: PoseWithCovarianceStamped):
        """Process the initial pose estimate"""
        self.get_logger().info("Received initial pose.")
        
        if not self.map_received:
            self.get_logger().warn("Initial pose received but map not yet available. Waiting for map...")
            return
            
        self.initial_pose = self.pose_to_mat(message.pose.pose)
        self.T_map_to_odom = self.initial_pose  # Start with this as our estimate
        self.initialized = True
        
        if self.current_scan is not None and len(numpy.asarray(self.current_scan.points)) > 0:
            self.global_localization()
        
        # Start the localization timer
        if hasattr(self, 'timer_localization'):
            # Reset timer if it already exists
            self.timer_localization.cancel()
            
        self.timer_localization = self.create_timer(
            timer_period_sec=1.0 / self.get_parameter("freq_localization").value,
            callback=self.localization_timer_callback
        )
        self.get_logger().info("Localization initialized and timer started")

    def voxel_down_sample(self, pcd, voxel_size):
        """Downsample a pointcloud using voxel grid filter"""
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except Exception as e:
            # Fallback for older open3d versions
            pcd_down = open3d.geometry.voxel_down_sample(pcd, voxel_size)
        return pcd_down

    def registration_at_scale(self, scan, map_pc, initial, scale):
        """ICP registration at a specific scale"""
        if len(numpy.asarray(scan.points)) == 0 or len(numpy.asarray(map_pc.points)) == 0:
            self.get_logger().warn("Empty pointcloud in registration")
            return numpy.eye(4), 0.0
            
        try:
            scan_down = self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale)
            map_down = self.voxel_down_sample(map_pc, self.get_parameter("map_voxel_size").value * scale)
            
            result_icp = open3d.pipelines.registration.registration_icp(
                scan_down,
                map_down,
                1.0 * scale,
                initial,
                open3d.pipelines.registration.TransformationEstimationPointToPoint(),
                open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
            )
            return result_icp.transformation, result_icp.fitness
        except Exception as e:
            self.get_logger().error(f"Error in registration: {e}")
            return numpy.eye(4), 0.0
    
    def pose_to_mat(self, pose):
        """Convert a geometry_msgs Pose to a 4x4 transformation matrix"""
        trans = numpy.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def inverse_se3(self, transformation_matrix):
        """Compute the inverse of an SE(3) transformation matrix"""
        trans_inverse = numpy.eye(4)
        # R
        trans_inverse[:3, :3] = transformation_matrix[:3, :3].T
        # t
        trans_inverse[:3, 3] = -numpy.matmul(transformation_matrix[:3, :3].T, transformation_matrix[:3, 3])
        return trans_inverse
    
    def crop_global_map_in_FOV(self):
        """
        Crop the global map to the field of view from the current robot position
        """
        if self.current_odometry is None or not hasattr(self.global_map, 'points'):
            self.get_logger().warn("No odometry data or global map available for FOV cropping")
            return None
            
        T_odom_to_base_link = self.pose_to_mat(self.current_odometry.pose.pose)
        T_map_to_base_link = numpy.matmul(self.T_map_to_odom, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # Get points from global map
        global_map_in_map = numpy.asarray(self.global_map.points)
        if len(global_map_in_map) == 0:
            self.get_logger().warn("Global map has no points")
            return None
            
        global_map_in_map = numpy.column_stack([global_map_in_map, numpy.ones(len(global_map_in_map))])
        global_map_in_base_link = numpy.matmul(T_base_link_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = numpy.where(
                (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (numpy.abs(numpy.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )[0]  # Get the first element of the tuple returned by where
        else:
            indices = numpy.where(
                (global_map_in_base_link[:, 0] > 0)
                & (global_map_in_base_link[:, 0] < self.get_parameter("fov_far").value)
                & (numpy.abs(numpy.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.get_parameter("fov").value / 2.0)
            )[0]  # Get the first element of the tuple returned by where
            
        global_map_in_FOV = open3d.geometry.PointCloud()
        global_map_in_FOV.points = open3d.utility.Vector3dVector(global_map_in_map[indices, :3])

        return global_map_in_FOV

    def global_localization(self):
        """
        Perform global localization using the current scan and global map
        """
        if not self.initialized:
            self.get_logger().info("Not initialized yet, waiting for initial pose")
            return
        
        if not self.map_received:
            self.get_logger().info("No map received yet, waiting for map")
            return
            
        if self.current_scan is None or len(numpy.asarray(self.current_scan.points)) == 0:
            self.get_logger().warn("Current scan is empty, skipping localization")
            return
            
        # Crop global map to FOV for more efficient processing
        global_map_in_FOV = self.crop_global_map_in_FOV()
        if global_map_in_FOV is None or len(numpy.asarray(global_map_in_FOV.points)) == 0:
            self.get_logger().warn("No points in FOV, skipping localization")
            return

        # First coarse registration
        transformation, _ = self.registration_at_scale(self.current_scan, global_map_in_FOV, initial=self.T_map_to_odom, scale=5)
        
        # Then fine registration
        transformation, fitness = self.registration_at_scale(self.current_scan, global_map_in_FOV, initial=transformation, scale=1)

        self.T_map_to_odom = transformation
        self.publish_pose(transformation)
        if fitness > self.get_parameter("localization_threshold").value:
            self.get_logger().debug(f"Successful localization with fitness {fitness}")
        else:
            self.get_logger().warn(f"Fitness score {fitness} less than localization threshold {self.get_parameter('localization_threshold').value}")

    def publish_pose(self, transform):
        """Publish the localization pose result"""
        pose_msg = PoseStamped()
        xyz = transform[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(transform)
        
        pose_msg.pose.position.x = xyz[0]
        pose_msg.pose.position.y = xyz[1]
        pose_msg.pose.position.z = xyz[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        self.pub_localization.publish(pose_msg)

    def localization_timer_callback(self):
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return
            
        if not self.map_received:
            self.get_logger().info("Waiting for map...")
            return

        if self.current_scan is not None and len(numpy.asarray(self.current_scan.points)) > 0:
            self.global_localization()
        else:
            self.get_logger().warn("Current scan is empty, skipping localization.")


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()