from .config import THRESHOLD
from .robot import Direction, Robot


class Controller:
    def __init__(self, production: bool = True) -> None:
        self.robot = Robot(production)
        # self.robot.drive(Direction.STOP)

    # TODO: add if not recevied command last 3 seconds, STOP

    @staticmethod
    def convert_coordinates_to_direction(x: float, y: float) -> Direction:
        upper_threshold = THRESHOLD
        lower_threshold = -THRESHOLD

        if x < lower_threshold:
            return Direction.TURN_LEFT

        if x > upper_threshold:
            return Direction.TURN_RIGHT

        if y > upper_threshold:
            return Direction.FORWARD

        return Direction.STOP

    def handle_vr_data(self, msg) -> None:
        x, y, speed = msg.x, msg.y, msg.speed
        print(f"got {x=} {y=} {speed=}")
        direction = self.convert_coordinates_to_direction(x, y)

        if speed not in range(0, 100 + 1):
            speed = 50

        if speed != self.robot.speed:
            self.robot.set_speed(speed)

        self.robot.drive(direction)
