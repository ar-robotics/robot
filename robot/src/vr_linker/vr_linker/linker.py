import socket
import time

import rclpy
from rclpy.node import Node


class TCPSocket(Node):

    def __init__(self):
        super().__init__("tcp_socket")
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
            received_data = data.decode("utf-8")
            print(f"{time.time()} - received: {received_data}")

            # Echo the received data back to the client
            self.client_socket.sendall("hello meta quest".encode())

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
