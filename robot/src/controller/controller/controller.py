from .robot import Direction, Robot
from .utils import get_threshold


class Controller:
    try:
        THRESHOLD = get_threshold()
    except FileNotFoundError:
        # for building the documentation
        THRESHOLD = 0.3

    def __init__(self, production: bool = True) -> None:
        """Interacts with the Expansion board.

        Args:
            production: if True, the robot will be controlled by the Expansion board.
        """
        self.robot = Robot(production)
        # self.robot.drive(Direction.STOP)

    # TODO: add if not recevied command last 3 seconds, STOP

    def convert_coordinates_to_direction(self, x: float, y: float) -> Direction:
        """Converts x, y coordinates to a direction.

        Args:
            x: x coordinate
            y: y coordinate
            threshold: positive threshold value

        Returns:
            direction
        """
        upper_threshold = self.THRESHOLD
        lower_threshold = -self.THRESHOLD

        if x < lower_threshold:
            return Direction.TURN_LEFT

        if x > upper_threshold:
            return Direction.TURN_RIGHT

        if y > upper_threshold:
            return Direction.FORWARD

        return Direction.STOP

    @staticmethod
    def convert_x_to_angle_difference(x: float) -> int:
        return int(x * 90)

    @staticmethod
    def convert_y_to_angle_difference(y: float) -> int:
        y_abs = abs(y)

        if y_abs > 1:
            return 90

        return int(y_abs * 90)

    @staticmethod
    def convert_pinch_to_angle(pinch: float) -> int:
        """Converts pinch to angle.

        Args:
            pinch: pinch

        Returns:
            angle
        """
        angle = int(abs(pinch) * 180)

        # servo hangs if angle is less than 45
        if angle <= 45:
            angle = 45

        if angle >= 180:
            angle = 180

        return angle

    def handle_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        # pinch, wrist = msg.pinch, msg.wrist
        # print(f"got {pinch=} {wrist=}")

        # if pinch not in range(45, 180 + 1):
        #     pinch = self.robot.UNPINCH_ANGLE

        # if wrist not in range(0, 180 + 1):
        #     wrist = self.robot.WRIST_RESET_ANGLE
        #
        # self.robot.set_wrist(wrist)
        # self.robot.set_pinch(pinch)
        x, y, pinch, strength = msg.x, msg.y, msg.pinch, msg.strength

        print(f"got {x=} {y=} {pinch=} {strength=}")
        x_angle = self.convert_x_to_angle_difference(x)
        self.robot.set_arm_rotation_difference(x_angle)

        y_angle = self.convert_y_to_angle_difference(y)
        self.robot.set_arm_tilt(y_angle)

        pinch_angle = self.convert_pinch_to_angle(strength)
        self.robot.set_pinch(pinch_angle)

        print(f"- {x_angle=} {y_angle=} {pinch_angle=}")

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

        if speed != self.robot.speed:
            self.robot.set_speed(speed)

        self.robot.drive(direction)

    def set_mode_defaults(self, mode: int) -> None:
        """Sets the mode defaults.

        Args:
            mode: mode
        """
        self.robot.drive(Direction.STOP)

        if mode == 1:
            # beep once in drive mode
            self.robot.beep()

            # reset arm
            self.robot.reset_arm_rotation()
            self.robot.reset_arm_shoulder()
            self.robot.reset_arm_elbow()
            self.robot.reset_arm_tilt()
            self.robot.reset_wrist()
            self.robot.pinch()

        if mode == 2:
            # beep twice in arm mode
            self.robot.beep()
            self.robot.beep()

            self.robot.set_arm_tilt(90)
            self.robot.unpinch()

    def handle_vr_mode(self, msg) -> None:
        """Handles VRMode messages.

        Args:
            msg: VRMode message
        """
        print(f"got {msg.mode=}")

        # NOTE:
        # CHANGE CAMERA
        # RESET ARM?
        # STOP

        self.set_mode_defaults(msg.mode)
