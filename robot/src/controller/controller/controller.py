import time

from .robot import Direction, Robot
from .utils import get_threshold


class Controller:

    def __init__(self, production: bool = True) -> None:
        """Interacts with the Expansion board.

        Args:
            production: if True, the robot will be controlled by the Expansion board.
        """
        self.robot = Robot(production)
        self.THRESHOLD = get_threshold()
        self.last_command_received = time.time()
        self.is_sleeping = False
        self.last_mode = -1
        # self.robot.drive(Direction.STOP)

    def convert_coordinates_to_direction(self, x: float, y: float) -> Direction:
        """Converts x, y coordinates to a direction.

        Args:
            x: x coordinate
            y: y coordinate

        Returns:
            direction to drive in
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
        """Converts x to angle difference.

        Args:
            x: x

        Returns:
            angle difference
        """
        return int(x * 90)

    @staticmethod
    def convert_y_to_angle_difference(y: float) -> int:
        """Converts y to angle difference.

        Args:
            y: y

        Returns:
            angle difference
        """
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

    def set_mode_defaults(self, mode: int) -> None:
        """Sets the mode defaults.

        Args:
            mode: mode
        """
        self.robot.drive(Direction.STOP)

        if self.last_mode == mode:
            return

        if self.last_mode == -1:
            self.last_mode = mode

        if mode == 1:  # drive mode
            # beep once in drive mode
            self.robot.beep()

            # reset arm
            # rotate arm out
            self.robot.unpinch()
            self.robot.reset_wrist()
            self.robot.reset_arm_tilt()
            self.robot.set_arm_rotation(180)
            self.robot.reset_arm_elbow()
            self.robot.set_arm_shoulder(175)
            time.sleep(0.5)

            # rotate arm in
            self.robot.set_arm_rotation(100)
            time.sleep(0.5)

            self.robot.long_beep()

        if mode == 2:  # arm mode
            # beep twice in arm mode
            self.robot.beep()
            self.robot.beep()

            self.robot.set_arm_rotation(180)
            time.sleep(0.5)

            self.robot.set_arm_shoulder(90)
            time.sleep(0.25)
            self.robot.set_arm_rotation(90)
            time.sleep(0.5)

            self.robot.long_beep()

    def check_last_message_received(self) -> None:
        """Checks if the last message was received more than 5 seconds ago.

        If so, the robot will stop and sleep. Will long beep twice to indicate this.
        """
        if time.time() < self.last_command_received + 5:
            return

        # check only if first time going into sleep mode
        if self.is_sleeping:
            return

        self.robot.stop()
        self.robot.long_beep()
        self.robot.long_beep()
        self.is_sleeping = True

    def _set_last_message_received(self) -> None:
        """Sets the last message received time to the current time."""
        self.last_command_received = time.time()
        self.is_sleeping = False

    def handle_vr_hand(self, msg) -> None:
        """Handles VRHand messages.

        Args:
            msg: VRHand message
        """
        self._set_last_message_received()

        x, y, pinch, strength = msg.x, msg.y, msg.pinch, msg.strength

        print(f"got {x=} {y=} {pinch=} {strength=}")
        x_angle = self.convert_x_to_angle_difference(x)
        self.robot.set_arm_rotation_difference(x_angle)

        y_angle = self.convert_y_to_angle_difference(y)
        self.robot.set_arm_tilt(y_angle)

        pinch_angle = self.convert_pinch_to_angle(strength)
        self.robot.set_pinch(pinch_angle)

        print(f"- {x_angle=} {y_angle=} {pinch_angle=}")

    @staticmethod
    def _speed_out_of_range(speed: int) -> bool:
        """Checks if the speed is out of range.

        Args:
            speed: speed

        Returns:
            True if the speed is out of range
        """
        return speed not in range(0, 100 + 1)

    def handle_vr_data(self, msg) -> None:
        """Handles VRData messages.

        Args:
            msg: VRData message
        """
        self._set_last_message_received()

        x, y, speed = msg.x, msg.y, msg.speed
        print(f"got {x=} {y=} {speed=}")
        direction = self.convert_coordinates_to_direction(x, y)

        if self._speed_out_of_range(speed):
            speed = self.robot.DEFAULT_SPEED

        if speed != self.robot.speed:
            self.robot.set_speed(speed)

        self.robot.drive(direction)

    def handle_vr_mode(self, msg) -> None:
        """Handles VRMode messages.

        Args:
            msg: VRMode message
        """
        self._set_last_message_received()

        print(f"got {msg.mode=}")

        # NOTE:
        # CHANGE CAMERA
        # RESET ARM?
        # STOP

        self.set_mode_defaults(msg.mode)
