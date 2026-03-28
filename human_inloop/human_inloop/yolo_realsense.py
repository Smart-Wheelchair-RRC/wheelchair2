import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloRealSenseNode(Node):
    def __init__(self):
        super().__init__('yolo_realsense_node')

        # Initialize the YOLOv8 model (using the nano model for speed)
        self.model = YOLO('yolov8n.pt')
        
        # Target classes: 0 = Person, 56 = Chair (from your previous setup)
        # Remove this list if you want to detect everything
        self.target_classes = [0, 56] 

        # Initialize CvBridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the RealSense RGB topic
        # NOTE: Check your realsense launch file. This topic might be '/camera/camera/color/image_raw' depending on your config.
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw', 
            self.image_callback,
            10
        )

        # Create a publisher to output the annotated video stream
        self.publisher_ = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info("YOLO RealSense Node has been started.")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV Image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. Run YOLO inference
            results = self.model.predict(
                source=cv_image, 
                classes=self.target_classes, 
                conf=0.5, 
                verbose=False
            )

            # 3. Model the Boxes: Extracting raw coordinate data
            # This loop shows how to grab the actual math for the bounding boxes
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Bounding box coordinates [x1, y1, x2, y2]
                    b = box.xyxy[0].tolist() 
                    class_id = int(box.cls[0])
                    confidence = round(float(box.conf[0]), 2)
                    
                    class_name = "Person" if class_id == 0 else "Chair"
                    self.get_logger().info(f"Detected {class_name} at [x1:{int(b[0])}, y1:{int(b[1])}, x2:{int(b[2])}, y2:{int(b[3])}] with {confidence} conf.")

            # 4. Generate the image with drawn bounding boxes
            annotated_frame = results[0].plot()

            # 5. Convert back to ROS Image message and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloRealSenseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()