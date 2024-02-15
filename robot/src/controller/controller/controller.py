import rclpy
from rclpy.node import Node

from interfaces.msg import Testing

# from std_msgs.msg import String


class Controller(Node):
    """Interacts with the Expansion board."""

    def __init__(self):
        super().__init__("Controller")

        # subscribers
        self.sub_pos = self.create_subscription(Testing, "testing", self.sub, 1)

        # publishers
        self.create_timer(1, self.print_loop)

    def sub(self, msg) -> None:
        print("got message", msg)

    def print_loop(self) -> None:
        print("we are running...")


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
