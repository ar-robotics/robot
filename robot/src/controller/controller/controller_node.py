from interfaces.msg import VRData

import rclpy
from rclpy.node import Node

from .controller import Controller

# from std_msgs.msg import String


class ControllerNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Controller")

        print(self.__class__.__name__, "is running!")

        self.controller = Controller(True)

        # subscribers
        self.sub_vr = self.create_subscription(
            VRData, "vr_data", self.controller.handle_vr_data, 1
        )
        # self.sub_pos = self.create_subscription(Testing, "testing", self.sub, 1)

        # publishers
        self.create_timer(1, self.print_loop)

    def sub(self, msg) -> None:
        print("got message", msg)

    def print_loop(self) -> None:
        print("we are running...")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
