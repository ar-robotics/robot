import rclpy
from rclpy.node import Node


class Controller(Node):
    """Interacts with the Expansion board"""

    def __init__(self):
        super().__init__("Controller")

        self.create_timer(1, self.print_loop)

    def print_loop(self) -> None:
        print("we are running...")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
