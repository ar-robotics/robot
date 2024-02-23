from .direction import Direction
from .rosmaster import Rosmaster


class Robot:
    def __init__(self, production: bool = True) -> None:
        """Interacts with the Expansion board.

        Args:
            production: if True, the robot will be controlled by the Expansion board.
        """
        if production:
            self.ros_master = Rosmaster(com="/dev/ttyUSB0", debug=True)
        else:
            self.ros_master = None

        self.production = production
        self.speed = 50  # NOTE: cant send negative values

    def get_direction(self, direction: Direction) -> list[int]:
        """Returns the direction vector.

        Args:
            direction: direction

        Returns:
            direction vector


        Example:
            >>> from controller.robot import Robot, Direction
            >>> robot = Robot()
            >>> robot.set_speed(50)
            >>> robot.get_direction(Direction.FORWARD)
            [0, 50]
        """
        return [self.speed * i for i in direction.value]

    def set_speed(self, speed: int) -> None:
        """Sets the speed.

        Args:
            speed: speed
        """
        self.speed = speed

    def drive(self, direction: Direction) -> None:
        """Drives the robot in a direction.

        Args:
            direction: direction
        """
        print(f"Driving in {direction=}")

        if not self.production:
            return

        self.ros_master.set_motor(*self.get_direction(direction))

    def stop(self) -> None:
        """Stops the robot."""
        self.drive(Direction.STOP)

    def forward(self) -> None:
        """Drives the robot forward."""
        self.drive(Direction.FORWARD)

    # def right(self) -> None:
    #     """Drives the robot right."""
    #     self.drive(Direction.RIGHT)

    # def left(self) -> None:
    #     """Drives the robot left."""
    #     self.drive(Direction.LEFT)
