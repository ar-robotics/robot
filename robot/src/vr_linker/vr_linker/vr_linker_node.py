import socket

from interfaces.msg import Message, RobotData, VRData, VRHand, VRMode

import rclpy
from rclpy.node import Node

from .vr_linker import VRLinker


class VRLinkerNode(Node):

    def __init__(self):
        super().__init__("TCPSocketServer")

        print(self.__class__.__name__, "is running!")

        self.vr_linker = VRLinker(self)

        # subscribers
        self.sub_robot_data = self.create_subscription(
            RobotData, "robot_data", self.vr_linker.handle_robot_data, 1
        )

        # publishers
        self.pub_vr = self.create_publisher(VRData, "_vr_data", 1)
        self.pub_vr_hand = self.create_publisher(VRHand, "_vr_hand", 1)
        self.pub_vr_mode = self.create_publisher(VRMode, "_vr_mode", 1)
        self.pub_message = self.create_publisher(Message, "message", 1)

        self.host = "0.0.0.0"  # Listen on all network interfaces
        self.port = 8080

        # Create a TCP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the address and port
        self.server_socket.bind((self.host, self.port))

        # Listen for incoming connections
        self.server_socket.listen(1)
        print(f"Server is listening on {self.host}:{self.port}")

        self.listen()

    def listen(self) -> None:
        # Accept incoming connection
        self.client_socket, self.client_address = self.server_socket.accept()
        print(f"Connection established with {self.client_address}")

        try:
            while True:
                self.vr_linker.process_message()

        except BaseException as e:
            print(f"Error: {e}")
            msg = Message()
            msg.message = f"Error in VRLinkerNode, {e}. Closing connection..."
            msg.level = 50
            self.pub_message.publish(msg)

        self.cleanup()

    def cleanup(self):
        """Cleans up the node."""
        # Close the connection
        self.client_socket.close()
        self.server_socket.close()

        msg = Message()
        msg.message = "Socket connection to VR closed."
        msg.level = 50
        self.pub_message.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VRLinkerNode()
    rclpy.spin(node)
    rclpy.shutdown()
