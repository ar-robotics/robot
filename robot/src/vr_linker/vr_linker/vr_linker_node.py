import json
import socket
import time

from interfaces.msg import VRData, VRHand

import rclpy
from rclpy.node import Node


class VRLinkerNode(Node):

    def __init__(self):
        super().__init__("TCPSocketServer")

        print(self.__class__.__name__, "is running!")

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
            # Receive data from client
            data = self.client_socket.recv(1024)

            if not data:
                break

            # Decode received data
            received_data = self.__to_json(data)
            print(f"{time.time()} - received: {received_data}")
            self.__publish_vr_data(received_data)

            # Echo the received data back to the client
            # self.client_socket.sendall(self.__to_json({"sucess":True}))
            self.send({"success": True})

    def __publish_vr_data(self, data: dict) -> None:
        """Publishes VRData messages.

        Args:
            data: VRData message to _vr_data topic
        """
        if not data:
            return

        if data.get("pinch"):
            vr_data = VRHand()
            vr_data.pinch = data["pinch"]
            vr_data.wrist = data["wrist"]
            self.pub_vr_hand.publish(vr_data)
            return

        vr_data = VRData()
        vr_data.x = data["x"]
        vr_data.y = data["y"]
        vr_data.speed = data["speed"]
        self.pub_vr.publish(vr_data)

    def send(self, data: dict) -> None:
        """Sends data to the client.

        Args:
            data: data to send
        """
        self.client_socket.sendall(self.__to_json(data))

    def __to_json(self, data: bytes | dict) -> dict | bytes:
        """Converts data to JSON.

        Args:
            data: data to convert

        Returns:
            JSON data as dict or bytes
        """
        try:
            if isinstance(data, bytes):
                return json.loads(data)
            return json.dumps(data).encode()
        except json.decoder.JSONDecodeError:
            print("Could not parse JSON data", data.decode())

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
    except Exception:
        pass
    finally:
        tcp_server_node.cleanup()
        tcp_server_node.destroy_node()
        rclpy.shutdown()
