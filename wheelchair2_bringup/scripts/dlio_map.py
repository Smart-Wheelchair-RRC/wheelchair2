import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class DenseMapSaver(Node):
    def __init__(self):
        super().__init__('dense_map_saver')
        
        self.filename = "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/himalaya/apr16_lab_to_classroom/dlio_map.pcd"
        # Open file in binary append/write mode
        self.file = open(self.filename, "wb")
        
        # PCD files require a specific header. We write a placeholder first.
        # By padding the 'WIDTH' and 'POINTS' variables with spaces (<20),
        # we lock the header to a fixed byte length. When the node shuts down, 
        # we can seek back to byte 0 and overwrite this header with the exact point count.
        self.header_template = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH {points:<20}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS {points:<20}\n"
            "DATA binary\n"
        )
        
        self.total_points = 0
        
        # Write the initial dummy header
        initial_header = self.header_template.format(points=0).encode('ascii')
        self.file.write(initial_header)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/dlio/odom_node/pointcloud/deskewed',
            self.cloud_callback,
            10
        )
        
        self.get_logger().info("Streaming points directly to disk... Press Ctrl+C to finalize.")

    def cloud_callback(self, msg):
        arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        try:
            # Fixes the TypeError: Extracts columns from the structured numpy array
            xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)
        except (IndexError, TypeError, ValueError):
            # Fallback just in case you run this on older ROS2 versions (e.g. Foxy)
            xyz = np.array(list(arr), dtype=np.float32)
            
        if xyz.size == 0:
            return
            
        # Write the raw float32 binary bytes directly to the file stream
        self.file.write(xyz.tobytes())
        self.total_points += xyz.shape[0]
        
        self.get_logger().info(
            f"Saved {self.total_points} points directly to disk so far...", 
            throttle_duration_sec=2.0
        )

    def finalize_pcd(self):
        if self.total_points == 0:
            print("\n[WARNING] No points were saved. Closing empty file.")
            self.file.close()
            return

        print(f"\n[INFO] Finalizing PCD file with {self.total_points} total points...")
        
        # Seek back to the absolute beginning of the file
        self.file.seek(0)
        
        # Create the true header with the final point count
        final_header = self.header_template.format(points=self.total_points).encode('ascii')
        
        # Overwrite the dummy header
        self.file.write(final_header)
        self.file.close()
        print(f"[INFO] Save complete! File located at ./{self.filename}")


def main(args=None):
    rclpy.init(args=args)
    node = DenseMapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # We use print() here because rclpy.spin just tore down the ROS logger context
        print("\n[INFO] Interrupted by user. Wrapping up the PCD file...")
    finally:
        node.finalize_pcd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()