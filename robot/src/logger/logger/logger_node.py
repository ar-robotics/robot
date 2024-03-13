from interfaces.msg import Message, RobotData, VRData, VRHand, VRMode

import rclpy
from rclpy.node import Node

from .logger import Logger


class LoggerNode(Node):
    """Logs messages from nodes."""

    def __init__(self):
        super().__init__("Logger")

        print(self.__class__.__name__, "is running!")

        self.logger = Logger()

        # subscribers
        self.sub_vr = self.create_subscription(
            VRData, "_vr_data", self.logger.handle_vr_data, 1
        )
        self.sub_vr_hand = self.create_subscription(
            VRHand, "_vr_hand", self.logger.handle_vr_hand, 1
        )
        self.sub_vr_mode = self.create_subscription(
            VRMode, "_vr_mode", self.logger.handle_vr_mode, 1
        )
        self.sub_robot_data = self.create_subscription(
            RobotData, "robot_data", self.logger.handle_robot_data, 1
        )
        self.sub_message = self.create_subscription(
            Message, "message", self.logger.handle_message, 1
        )


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()
