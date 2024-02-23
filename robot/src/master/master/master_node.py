from interfaces.msg import VRData, VRHand

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String


class MasterNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Master")

        print(self.__class__.__name__, "is running!")

        self.in_emergency_mode = False

        # NOTE: subscribers
        self.sub_vr = self.create_subscription(
            VRData, "_vr_data", self.handle_unsafe_vr_data, 1
        )
        self.sub_vr_hand = self.create_subscription(
            VRHand, "_vr_hand", self.handle_unsafe_vr_hand, 1
        )

        # NOTE: publishers
        self.pub_vr = self.create_publisher(VRData, "vr_data", 1)
        self.pub_vr_hand = self.create_publisher(VRHand, "vr_hand", 1)

    def check_emergency_mode(self) -> None:
        if self.in_emergency_mode:
            raise Exception("In emergency mode, aborting...")

        return

    def handle_unsafe_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        self.check_emergency_mode()
        self.pub_vr.publish(msg)

    def handle_unsafe_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        self.check_emergency_mode()
        self.pub_vr_hand.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
