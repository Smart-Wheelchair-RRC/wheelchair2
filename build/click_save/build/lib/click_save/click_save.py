import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk


class ClickSavePublisher(Node):
    def __init__(self):
        super().__init__('click_save')

        # Publishers
        self.pub_save_map = self.create_publisher(Bool, '/save_map', 10)
        self.pub_save_pose = self.create_publisher(Bool, '/save_pose', 10)

        # Create GUI window
        self.root = tk.Tk()
        self.root.title("Click Save")
        self.root.geometry("300x200")

        # Save Map button
        btn_map = tk.Button(
            self.root,
            text="Save Map",
            width=20,
            height=5,
            command=self.publish_save_map
        )
        btn_map.pack(pady=10)

        # Save Pose button
        btn_pose = tk.Button(
            self.root,
            text="Save Pose",
            width=20,
            height=5,
            command=self.publish_save_pose
        )
        btn_pose.pack(pady=10)

        # Proper shutdown when window closes
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def publish_save_map(self):
        msg = Bool()
        msg.data = True
        self.pub_save_map.publish(msg)
        self.get_logger().info("Save Map requested")

    def publish_save_pose(self):
        msg = Bool()
        msg.data = True
        self.pub_save_pose.publish(msg)
        self.get_logger().info("Save Pose requested")

    def on_close(self):
        self.get_logger().info("Shutting down click_save GUI")
        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ClickSavePublisher()
    node.run()


if __name__ == '__main__':
    main()
