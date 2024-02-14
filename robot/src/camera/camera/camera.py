import rclpy
from rclpy.node import Node
from v4l2py import Device


class Camera(Node):
    """Interacts with the Expansion board"""

    def __init__(self):
        super().__init__("Camera")

        self.cam = Device.from_id(0)
        self.cam.open()

        # since camera is 30 fps
        self.create_timer(1 / 30, self.take_picture)

    def take_picture(self) -> None:
        for frame in self.cam:
            print(f"frame data {frame.data}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
