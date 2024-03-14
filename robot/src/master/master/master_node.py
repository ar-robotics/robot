from interfaces.msg import Message, RobotData, VRData, VRHand, VRMode

import rclpy
from rclpy.node import Node

from .modes import Mode

# from std_msgs.msg import String


class MasterNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Master")

        print(self.__class__.__name__, "is running!")

        self.mode = Mode.IDLE

        # subscribers
        self.sub_vr = self.create_subscription(
            VRData, "_vr_data", self.handle_unsafe_vr_data, 1
        )
        self.sub_vr_hand = self.create_subscription(
            VRHand, "_vr_hand", self.handle_unsafe_vr_hand, 1
        )
        self.sub_vr_mode = self.create_subscription(
            VRMode, "_vr_mode", self.handle_unsafe_vr_mode, 1
        )
        self.sub_robot_data = self.create_subscription(
            RobotData, "_robot_data", self.handle_robot_data, 1
        )

        # publishers
        self.pub_vr = self.create_publisher(VRData, "vr_data", 1)
        self.pub_vr_hand = self.create_publisher(VRHand, "vr_hand", 1)
        self.pub_vr_mode = self.create_publisher(VRMode, "vr_mode", 1)
        self.pub_robot_data = self.create_publisher(RobotData, "robot_data", 1)
        self.pub_message = self.create_publisher(Message, "message", 1)

    def set_mode(self, mode: Mode) -> None:
        """Sets the mode.

        Args:
            mode: mode
        """
        self.mode = mode
        print(f"mode set to {self.mode}")

        msg = Message()
        msg.message = f"mode set to {self.mode}"
        msg.level = 20
        self.pub_message.publish(msg)

    def get_mode(self) -> Mode:
        """Gets the mode.

        Returns:
            mode
        """
        return self.mode

    # NOTE: make this a decorator
    def not_emergency_mode(self) -> None:
        """Checks if the node is in emergency mode and raises an exception if it is."""
        if self.mode != Mode.EMERGENCY:
            return

        # raise Exception("In emergency mode, aborting...")
        print("In emergency mode, aborting...")

    def has_mode(self, mode: Mode) -> bool:
        """Checks if the node has the mode.

        Args:
            mode: mode

        Returns:
            True if the node has the mode
        """
        return self.mode == mode

    def int_to_mode(self, mode: int) -> Mode:
        """Converts an integer to a Mode.

        Args:
            mode: mode

        Returns:
            Mode
        """
        return Mode(mode)

    def handle_robot_data(self, msg) -> None:
        """Handles RobotData messages.

        Args:
            msg: RobotData message
        """
        msg.mode = self.mode.name
        self.pub_robot_data.publish(msg)

    def handle_unsafe_vr_mode(self, msg) -> None:
        """Handles VRMode messages.

        Args:
            msg: VRMode message
        """
        self.not_emergency_mode()
        mode = self.int_to_mode(msg.mode)

        if self.has_mode(mode):
            return

        self.set_mode(mode)
        self.pub_vr_mode.publish(msg)

    def handle_unsafe_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        self.not_emergency_mode()

        if not self.has_mode(Mode.DRIVE):
            return

        self.pub_vr.publish(msg)

    def handle_unsafe_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        self.not_emergency_mode()

        if not self.has_mode(Mode.ARM):
            return

        self.pub_vr_hand.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
