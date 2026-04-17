#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class LidarCropNode(Node):
    def __init__(self):
        super().__init__('lidar_crop_node')
        
        # Subscriber to your non-ground points
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10  # Standard QoS. If it doesn't receive data, you may need to use rclpy.qos.qos_profile_sensor_data
        )
        
        # Publisher for the cropped cloud
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar_box_cropped', 10)
        
        # Map ROS PointField types to numpy datatypes
        self.type_mapping = {
            PointField.INT8: np.int8,
            PointField.UINT8: np.uint8,
            PointField.INT16: np.int16,
            PointField.UINT16: np.uint16,
            PointField.INT32: np.int32,
            PointField.UINT32: np.uint32,
            PointField.FLOAT32: np.float32,
            PointField.FLOAT64: np.float64,
        }
        
        self.get_logger().info("Lidar Crop Node Started. Cropping 90 degrees at the rear...")

    def listener_callback(self, msg):
        if not msg.data:
            return

        # 1. Create a dynamic NumPy dtype based on the exact fields in the incoming message.
        # This ensures we don't lose Livox-specific fields like 'intensity', 'tag', or 'line'.
        try:
            dtype_dict = {
                'names': [f.name for f in msg.fields],
                'formats': [self.type_mapping[f.datatype] for f in msg.fields],
                'offsets': [f.offset for f in msg.fields],
                'itemsize': msg.point_step
            }

            # 2. View the raw byte data as a structured 1D numpy array
            cloud_arr = np.ndarray(
                shape=(msg.width * msg.height,),
                dtype=dtype_dict,
                buffer=msg.data
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse PointCloud2: {e}")
            return

        # 3. Calculate angles in the XY plane. 
        # Standard ROS frame: +X is Forward, +Y is Left.
        # Back is -X, which corresponds to +/- 180 degrees (+/- pi).
        angles = np.arctan2(cloud_arr['y'], cloud_arr['x'])

        # 4. Filter the points. 
        # A 90-degree slice at the back means 45 degrees to the left and right of -X.
        # This translates to keeping points where the absolute angle is <= 135 degrees (3pi/4).
        mask = np.abs(angles) <= (3.0 * np.pi / 4.0)
        filtered_arr = cloud_arr[mask]

        # 5. Reconstruct the cropped PointCloud2 message
        cropped_msg = PointCloud2()
        cropped_msg.header = msg.header  # Keeps original frame_id and timestamp
        cropped_msg.height = 1
        cropped_msg.width = len(filtered_arr)
        cropped_msg.fields = msg.fields
        cropped_msg.is_bigendian = msg.is_bigendian
        cropped_msg.point_step = msg.point_step
        cropped_msg.row_step = msg.point_step * cropped_msg.width
        cropped_msg.is_dense = msg.is_dense
        cropped_msg.data = filtered_arr.tobytes()

        # 6. Publish the cleaned cloud
        self.publisher.publish(cropped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()