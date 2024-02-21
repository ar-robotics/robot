from .direction import Direction
from .rosmaster import Rosmaster


class Robot:
    def __init__(self, testing: bool = False) -> None:
        if testing:
            self.ros_master = None
        else:
            self.ros_master = Rosmaster(com="/dev/ttyUSB0", debug=True)

        self.testing = testing
        self.speed = 50  # NOTE: cant send -values

    def get_direction(self, direction: Direction) -> list[int]:
        return [self.speed * i for i in direction.value]

    def set_speed(self, speed: int) -> None:
        self.speed = speed

    def drive(self, direction: Direction) -> None:
        if self.testing:
            print(f"Testing mode: would drive {direction}")
            return

        self.ros_master.set_motor(*self.get_direction(direction))

    def stop(self) -> None:
        self.drive(Direction.STOP)

    def forward(self) -> None:
        self.drive(Direction.FORWARD)

    def right(self) -> None:
        self.drive(Direction.RIGHT)

    def left(self) -> None:
        self.drive(Direction.LEFT)
