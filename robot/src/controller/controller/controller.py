from .robot import Direction, Robot
from .utils import get_threshold

THRESHOLD = get_threshold()


class Controller:

    def __init__(self, production: bool = True) -> None:
        """Interacts with the Expansion board.

        Args:
            production: if True, the robot will be controlled by the Expansion board.
        """
        self.robot = Robot(production)
        # self.robot.drive(Direction.STOP)

    # TODO: add if not recevied command last 3 seconds, STOP

    @staticmethod
    def convert_coordinates_to_direction(x: float, y: float) -> Direction:
        """Converts x, y coordinates to a direction.

        Args:
            x: x coordinate
            y: y coordinate
            threshold: positive threshold value

        Returns:
            direction
        """
        upper_threshold = THRESHOLD
        lower_threshold = -THRESHOLD

        if x < lower_threshold:
            return Direction.TURN_LEFT

        if x > upper_threshold:
            return Direction.TURN_RIGHT

        if y > upper_threshold:
            return Direction.FORWARD

        return Direction.STOP

    @staticmethod
    def convert_pinch_to_angle(pinch: int) -> int:
        return 0

    @staticmethod
    def convert_wrist_to_angle(wrist: int) -> int:
        return 0

    def handle_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        pinch, wrist = msg.pinch, msg.wrist
        print(f"got {pinch=} {wrist=}")

        if pinch not in range(45, 180 + 1):
            pinch = self.robot.UNPINCH_ANGLE

        if wrist not in range(0, 180 + 1):
            wrist = self.robot.WRIST_RESET_ANGLE

        return

        self.robot.set_wrist(wrist)
        self.robot.set_pinch(pinch)

    def _convert_x_to_angle_difference(self, x: float) -> int:
        return x * 90

    def handle_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        x, y, speed = msg.x, msg.y, msg.speed
        print(f"got {x=} {y=} {speed=}")
        direction = self.convert_coordinates_to_direction(x, y)

        if speed not in range(0, 100 + 1):
            speed = self.robot.DEFAULT_SPEED

        angle = self._convert_x_to_angle_difference(x)
        print(f"{angle=}")
        self.robot.set_arm_rotation_difference(angle)

        return

        if speed != self.robot.speed:
            self.robot.set_speed(speed)

        self.robot.drive(direction)
