from interfaces.msg import VRData

import rclpy
from rclpy.node import Node

from .config import PRODUCTION
from .controller import Controller

# from std_msgs.msg import String


class ControllerNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Controller")

        print(self.__class__.__name__, "is running!")

        self.controller = Controller(PRODUCTION)

        # NOTE: subscribers
        self.sub_vr = self.create_subscription(
            VRData, "vr_data", self.controller.handle_vr_data, 1
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
