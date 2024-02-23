from interfaces.msg import VRData

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String


class Master(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Master")

        print(self.__class__.__name__, "is running!")

        self.in_emergency_mode = False

        # NOTE: subscribers
        self.sub_vr = self.create_subscription(
            VRData, "_vr_data", self.handle_unsafe_vr_data, 1
        )

        # NOTE: publishers
        self.pub_vr = self.create_publisher(VRData, "vr_data", 1)

    def handle_unsafe_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        if self.in_emergency_mode:
            print("In emergency mode, aborting...")
            return

        self.pub_vr.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Master()
    rclpy.spin(node)
    rclpy.shutdown()
