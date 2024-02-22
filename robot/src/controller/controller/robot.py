from .direction import Direction
from .rosmaster import Rosmaster


class Robot:
    def __init__(self, production: bool = True) -> None:
        if production:
            self.ros_master = Rosmaster(com="/dev/ttyUSB0", debug=True)
        else:
            self.ros_master = None

        self.production = production
        self.speed = 50  # NOTE: cant send negative values

    def get_direction(self, direction: Direction) -> list[int]:
        return [self.speed * i for i in direction.value]

    def set_speed(self, speed: int) -> None:
        self.speed = speed

    def drive(self, direction: Direction) -> None:
        print(f"Driving in {direction=}")

        if not self.production:
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
