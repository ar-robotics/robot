from interfaces.msg import VRData

import rclpy
from rclpy.node import Node

from .logger import Logger

# from std_msgs.msg import String


class LoggerNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Logger")

        print(self.__class__.__name__, "is running!")

        self.logger = Logger()

        # NOTE: subscribers
        self.sub_vr = self.create_subscription(
            VRData, "_vr_data", self.logger.handle_vr_data, 1
        )


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()
