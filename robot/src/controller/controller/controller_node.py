from interfaces.msg import Message, RobotData, VRData, VRHand, VRMode

import rclpy
from rclpy.node import Node

from .controller import Controller
from .utils import fill_vector_msg, get_data_hertz, get_production, get_sleep_mode_hertz


class ControllerNode(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Controller")

        print(self.__class__.__name__, "is running!")

        is_production = get_production()
        data_hertz = get_data_hertz()
        sleep_mode_hertz = get_sleep_mode_hertz()

        self.controller = Controller(is_production)

        # publishers
        self.pub_robot_data = self.create_publisher(RobotData, "_robot_data", 1)
        self.pub_message = self.create_publisher(Message, "message", 1)

        # subscribers
        self.sub_vr = self.create_subscription(
            VRData, "vr_data", self.controller.handle_vr_data, 1
        )
        self.sub_vr_hand = self.create_subscription(
            VRHand, "vr_hand", self.controller.handle_vr_hand, 1
        )
        self.sub_vr_mode = self.create_subscription(
            VRMode, "vr_mode", self.controller.handle_vr_mode, 1
        )

        # timers
        self.create_timer(1 / data_hertz, self.get_robot_data)
        self.create_timer(
            1 / sleep_mode_hertz, self.controller.check_last_message_received
        )

        msg = Message()
        msg.message = (
            f"Using settings {is_production=}, {data_hertz=}, {sleep_mode_hertz=}"
        )
        msg.level = 20
        self.pub_message.publish(msg)

    def get_robot_data(self) -> None:
        """Gets robot data such as voltage, speed, gyroscope, etc. and publishes it."""
        data = self.controller.robot.get_data()

        msg = RobotData()
        msg = fill_vector_msg(msg, "accelerometer", data["accelerometer"])
        msg = fill_vector_msg(msg, "gyroscope", data["gyroscope"])
        msg = fill_vector_msg(msg, "magnetometer", data["magnetometer"])
        msg = fill_vector_msg(msg, "motion", data["motion"])
        msg.voltage = data["voltage"]
        msg.mode = ""  # master fills this field

        self.pub_robot_data.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
