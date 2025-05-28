import kiss_icp.registration as ICP_registration
from kiss_icp.mapping import VoxelHashMap
import numpy
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from kiss_icp.voxelization import voxel_down_sample
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# import something to convert a 4x4 matrix to a Pose message. not from kiss
import tf_transformations


DEFAULT_QOS: QoSProfile = 10



class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
                topic="/map",
                callback=self.deal_with_global_map,
                qos_profile=DEFAULT_QOS
            ),
        }

        self.publications_created = {
            "localization": self.create_publisher(
                msg_type=Pose,
                topic="/localization_pose",
                qos_profile=QoSProfile(
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            ),
        }

        self.voxel_hashmap = VoxelHashMap(
            max_distance=2,
            voxel_size=self.get_parameter("map_voxel_size").value,
            max_points_per_voxel=1000
        )

    def deal_with_global_map(self, message: OccupancyGrid):
        # print(message)
        # self.get_logger().info("Received global map.")

        self.voxel_hashmap.clear()
        self.voxel_hashmap.update(
            points=numpy.array([
                (x, y, 0) for x in range(message.info.width)
                for y in range(message.info.height)
                if message.data[y * message.info.width + x] > 0
            ])  # Assuming the map is in the robot's frame
        )


    def deal_with_current_scan(self, message: PointCloud2):
        # self.get_logger().info("Received current scan.")
        self.current_scan = point_cloud2.read_points_numpy(message, field_names="xyz", reshape_organized_cloud=True)

    def deal_with_current_odometry(self, message: Odometry):
        # self.get_logger().info("Received current odometry.")
        self.current_odometry: numpy.ndarray = numpy.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w
        ])

    def deal_with_initial_pose(self, message: PoseWithCovarianceStamped):
        # self.get_logger().info("Received initial pose.")
        self.initial_pose: numpy.ndarray[numpy.float64[4, 4]] = numpy.eye(4)

        self.initial_pose[0, 3] = message.pose.pose.position.x
        self.initial_pose[1, 3] = message.pose.pose.position.y
        self.initial_pose[2, 3] = message.pose.pose.position.z
        self.initial_pose[0, 0] = message.pose.pose.orientation.x
        self.initial_pose[1, 1] = message.pose.pose.orientation.y
        self.initial_pose[2, 2] = message.pose.pose.orientation.z
        self.initial_pose[3, 3] = message.pose.pose.orientation.w
        self.registration()

        self.timer_localization = self.create_timer(
            timer_period_sec=1.0 / self.get_parameter("freq_localization").value,
            callback=self.localization_timer_callback
        )

    def registration(self):
        self.kiss_icp_registration = ICP_registration.Registration(
            convergence_criterion= 0.01,
            max_num_iterations= 100,
            max_num_threads= 4,
        )

    def global_localization(self):

        try:
            localised_pose = self.kiss_icp_registration.align_points_to_map(
                initial_guess=self.initial_pose,
                voxel_map= self.voxel_hashmap,
                points= voxel_down_sample(
                    self.current_scan, 0.1
                ),
                max_correspondance_distance= self.get_parameter("localization_threshold").value,
                kernel= 0.2,
            )

        except Exception as e:
            print(e.with_traceback())

        # publish the localised pose

        print("Localised pose:", localised_pose)
        
        # the localised pose is a 4x4 matrix, convert it to a Pose message
        pose_msg = Pose()
        pose_msg.position.x = localised_pose[0, 3]
        pose_msg.position.y = localised_pose[1, 3]
        pose_msg.position.z = localised_pose[2, 3]
        quat = tf_transformations.quaternion_from_matrix(localised_pose)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        self.publications_created["localization"].publish(pose_msg)

    def localization_timer_callback(self):
        if self.current_scan.size == 0:
            self.get_logger().warn("Current scan is empty, skipping localization.")
            return
        
        self.global_localization()


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()