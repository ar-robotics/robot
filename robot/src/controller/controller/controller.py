from .robot import Direction, Robot


class Controller:
    def __init__(self, testing: bool = False) -> None:
        self.robot = Robot(testing)
        # self.robot.drive(Direction.STOP)

    @staticmethod
    def convert_coordinates_to_direction(x: float, y: float) -> Direction:
        upper_threshold = 0.3
        lower_threshold = -upper_threshold

        if x < lower_threshold:
            return Direction.LEFT

        if x > upper_threshold:
            return Direction.RIGHT

        if y > upper_threshold:
            return Direction.FORWARD

        return Direction.STOP

    def handle_vr_data(self, msg) -> None:
        x, y, speed = msg.x, msg.y, msg.speed
        print(f"got {x=} {y=} {speed=}")
        direction = self.convert_coordinates_to_direction(x, y)

        print(direction)
        # self.robot.drive(direction)
