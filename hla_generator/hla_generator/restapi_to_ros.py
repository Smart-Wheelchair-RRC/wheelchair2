import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class HLAGenerator(Node):
    def __init__(self):
        super().__init__('hla_generator_node')
        self.create_subscription(String, '/llm_command', self.llama_cb, 10)
        self.get_logger().info(f"HLA Generator is up and listening to /llama_command ...")

    def llama_cb(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get("action", "none")
            destination = data.get("destination", "none")
            self.get_logger().info(f"Received Action: {action},  Destination: {destination}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Faillled to decode JSON: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = HLAGenerator()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()