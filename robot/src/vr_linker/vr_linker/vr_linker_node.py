import json
import socket
import time

from interfaces.msg import VRData

import rclpy
from rclpy.node import Node


class TCPSocket(Node):

    def __init__(self):
        super().__init__("tcp_socket")

        print(self.__class__.__name__, "is running!")

        # publishers
        self.pub_vr = self.create_publisher(VRData, "_vr_data", 1)

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
        if not data:
            return

        vr_data = VRData()
        vr_data.x = data["x"]
        vr_data.y = data["y"]
        vr_data.speed = data["speed"]
        self.pub_vr.publish(vr_data)

    def send(self, data: dict) -> None:
        self.client_socket.sendall(self.__to_json(data))

    def __to_json(self, data: bytes | dict) -> dict | bytes:
        try:
            if isinstance(data, bytes):
                return json.loads(data)
            return json.dumps(data).encode()
        except json.decoder.JSONDecodeError:
            print("Could not parse JSON data", data.decode())

    def cleanup(self):
        # Close the connection
        self.client_socket.close()
        self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    tcp_server_node = TCPSocket()
    try:
        rclpy.spin(tcp_server_node)
    except Exception:
        pass
    finally:
        tcp_server_node.cleanup()
        tcp_server_node.destroy_node()
        rclpy.shutdown()
