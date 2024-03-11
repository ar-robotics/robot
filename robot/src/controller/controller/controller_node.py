from interfaces.msg import VRData, VRHand, VRMode

import rclpy
from rclpy.node import Node

from .controller import Controller
from .utils import get_production


class ControllerNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Controller")

        print(self.__class__.__name__, "is running!")

        is_production = get_production()
        self.controller = Controller(is_production)

        # NOTE: subscribers
        self.sub_vr = self.create_subscription(
            VRData, "vr_data", self.controller.handle_vr_data, 1
        )
        self.sub_vr_hand = self.create_subscription(
            VRHand, "vr_hand", self.controller.handle_vr_hand, 1
        )
        self.sub_vr_mode = self.create_subscription(
            VRMode, "vr_mode", self.controller.handle_vr_mode, 1
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
