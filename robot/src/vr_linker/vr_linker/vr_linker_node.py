import socket

from interfaces.msg import VRData, VRHand

import rclpy
from rclpy.node import Node

from .vr_linker import VRLinker


class VRLinkerNode(Node):

    def __init__(self):
        super().__init__("TCPSocketServer")

        print(self.__class__.__name__, "is running!")

        self.vr_linker = VRLinker(self)

        # publishers
        self.pub_vr = self.create_publisher(VRData, "_vr_data", 1)
        self.pub_vr_hand = self.create_publisher(VRHand, "_vr_hand", 1)

        self.host = "0.0.0.0"  # Listen on all network interfaces
        self.port = 8080

        # Create a TCP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the address and port
        self.server_socket.bind((self.host, self.port))

        # Listen for incoming connections
        self.server_socket.listen()
        print(f"Server is listening on {self.host}:{self.port}")

        # Accept incoming connection
        self.client_socket, self.client_address = self.server_socket.accept()
        print(f"Connection established with {self.client_address}")

        while True:
            self.vr_linker.process_message()

    def cleanup(self):
        """Cleans up the node."""
        # Close the connection
        self.client_socket.close()
        self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    tcp_server_node = VRLinkerNode()
    try:
        rclpy.spin(tcp_server_node)
    except BaseException:
        pass
    finally:
        tcp_server_node.cleanup()
        tcp_server_node.destroy_node()
        rclpy.shutdown()
