#!/usr/bin/env python3
"""
simulate_elevator.py

Simulates elevator events from the terminal for testing the supervisor
without needing physical hardware or a sensor trigger node.

Usage:
  python3 simulate_elevator.py

Requires: rclpy, std_msgs
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import time
import sys


class ElevatorSimulator(Node):
    def __init__(self):
        super().__init__("elevator_simulator")
        self.is_inside_pub = self.create_publisher(Bool, "/elevator/is_inside", 1)
        self.floor_pub     = self.create_publisher(Int32, "/elevator/target_floor", 1)
        self.get_logger().info("Elevator simulator ready.")

    def publish_is_inside(self, value: bool):
        msg = Bool()
        msg.data = value
        self.is_inside_pub.publish(msg)
        self.get_logger().info(f"Published /elevator/is_inside = {value}")

    def publish_target_floor(self, floor: int):
        msg = Int32()
        msg.data = floor
        self.floor_pub.publish(msg)
        self.get_logger().info(f"Published /elevator/target_floor = {floor}")


def main():
    rclpy.init()
    node = ElevatorSimulator()

    print("\n=== Elevator Supervisor Test Simulator ===")
    print("Commands:")
    print("  e <floor>   Enter elevator heading to <floor>  (e.g.  e 4)")
    print("  x           Exit elevator (doors open on destination floor)")
    print("  q           Quit\n")

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd.startswith("e "):
                try:
                    floor = int(cmd.split()[1])
                    node.publish_target_floor(floor)
                    time.sleep(0.1)  # let target_floor arrive before is_inside
                    node.publish_is_inside(True)
                except (IndexError, ValueError):
                    print("Usage: e <floor_number>")
            elif cmd == "x":
                node.publish_is_inside(False)
            elif cmd == "q":
                break
            else:
                print("Unknown command. Use 'e <floor>', 'x', or 'q'.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()