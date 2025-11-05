#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray
import math
# import serial

#pip install pyserial

class MyDIYNode(Node):
    def __init__(self):
        super().__init__("node_name")

        self.get_logger().info("Waiting for joint_from_pose")

        # try:
        #     self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        #     self.get_logger().info("Serial /dev/ttyACM0 opened")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to open serial: {e}")
        #     self.ser = None

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_from_pose',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        angles = list(msg.data)
        self.get_logger().info(f"receive the joint angle: {angles}\n")

        encoder_vals = []
        for a in angles:
            norm = (a + math.pi) / (2 * math.pi)
            enc = int(norm * 4095)
            encoder_vals.append(enc)

        self.get_logger().info("motor encoder values:")
        for i, v in enumerate(encoder_vals):
            self.get_logger().info(f"{i}: {v}")

        serial_str = "".join(f"{v:04d}" for v in encoder_vals)
        self.get_logger().info(f"Serial output:{serial_str}\n\n")

        # if self.ser is not None:
        #     try:
        #         self.ser.write((serial_str + "\n").encode())
        #     except Exception as e:
        #         self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MyDIYNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
