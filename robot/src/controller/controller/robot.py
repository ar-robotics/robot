import time

from .enums import Direction, Preset
from .rosmaster import Rosmaster
from .utils import in_production_mode


class Robot:
    def __init__(self, production: bool = True) -> None:
        """Interacts with the Expansion board.

        Args:
            production: if True, the robot will be controlled by the Expansion board.
        """
        self.ros_master = None
        self.production = production
        self.speed = Preset.SPEED  # NOTE: cant send negative values

        if production:
            self.ros_master = Rosmaster(com="/dev/expbrd", debug=True)
            # self.ros_master = Rosmaster(com="/dev/ttyUSB1", delay=0.08, debug=True)

            self.ros_master.create_receive_threading()

            self.reset()

        self.last_direction = Direction.STOP

    def _reset_arm(self) -> None:
        """Resets the arm to its default position."""
        if not self.production:
            return

        # TODO: test this

        # check if the arm is under the camera
        if self.get_arm_shoulder() > 90:
            self.set_arm_rotation(180)
            time.sleep(0.5)

        self.reset_arm_shoulder()
        time.sleep(0.5)

        self.reset_arm_rotation()
        self.reset_arm_elbow()
        self.reset_arm_tilt()
        self.reset_wrist()
        self.unpinch()

    def reset(self) -> None:
        """Resets the robot."""
        self.stop()
        self._reset_arm()

    def get_speed(self) -> int:
        """Returns the speed.

        Returns:
            speed
        """
        return self.speed

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

        self.last_direction = direction

        if not self.production:
            return

        self.ros_master.set_motor(*self.get_direction(direction))

    def stop(self) -> None:
        """Stops the robot."""
        self.drive(Direction.STOP)

    def forward(self) -> None:
        """Drives the robot forward."""
        self.drive(Direction.FORWARD)

    @in_production_mode
    def set_pinch(self, angle: int) -> None:
        """Sets the pinch angle.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(6, angle)

    def pinch(self) -> None:
        """Pinches the "fingers"."""
        self.set_pinch(Preset.PINCH_ANGLE)

    def unpinch(self) -> None:
        """Unpinches the "fingers"."""
        self.set_pinch(Preset.UNPINCH_ANGLE)

    @in_production_mode
    def get_wrist(self) -> int:
        """Returns the wrist angle.

        Returns:
            wrist angle
        """
        return self.ros_master.get_uart_servo_angle(5)

    @in_production_mode
    def set_wrist(self, angle: int) -> None:
        """Turns the wrist. Angle should be between 0 and 270.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(5, angle)

    def reset_wrist(self) -> None:
        """Resets the wrist to its default position."""
        self.set_wrist(Preset.WRIST_ANGLE)

    @in_production_mode
    def get_arm_rotation(self) -> int:
        """Returns the arm rotation angle.

        Returns:
            arm rotation angle
        """
        return self.ros_master.get_uart_servo_angle(1)

    @in_production_mode
    def set_arm_rotation(self, angle: int) -> None:
        """Sets the arm rotation angle.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(1, angle)

    def reset_arm_rotation(self) -> None:
        """Resets the arm rotation to its default position."""
        self.set_arm_rotation(Preset.ARM_ROTATION_ANGLE)

    @in_production_mode
    def get_arm_shoulder(self) -> int:
        """Returns the arm shoulder angle.

        Returns:
            arm shoulder angle
        """
        return self.ros_master.get_uart_servo_angle(2)

    @in_production_mode
    def set_arm_shoulder(self, angle: int) -> None:
        """Sets the arm shoulder angle.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(2, angle)

    def set_arm_rotation_difference(self, angle: int) -> None:
        """Sets the arm rotation angle difference.

        Args:
            angle: angle
        """
        self.set_arm_shoulder(Preset.ARM_ROTATION_ANGLE + angle)

    def reset_arm_shoulder(self) -> None:
        """Resets the arm shoulder to its default position."""
        self.set_arm_shoulder(Preset.ARM_SHOULDER_ANGLE)

    @in_production_mode
    def get_arm_elbow(self) -> int:
        """Returns the arm elbow angle.

        Returns:
            arm elbow angle
        """
        return self.ros_master.get_uart_servo_angle(3)

    @in_production_mode
    def set_arm_elbow(self, angle: int) -> None:
        """Sets the arm elbow angle.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(3, angle)

    def reset_arm_elbow(self) -> None:
        """Resets the arm elbow to its default position."""
        self.set_arm_elbow(Preset.ARM_ELBOW_ANGLE)

    @in_production_mode
    def get_arm_tilt(self) -> int:
        """Returns the arm tilt angle.

        Returns:
            arm tilt angle
        """
        return self.ros_master.get_uart_servo_angle(4)

    @in_production_mode
    def set_arm_tilt(self, angle: int) -> None:
        """Sets the arm tilt angle.

        Args:
            angle: angle
        """
        self.ros_master.set_uart_servo_angle(4, angle)

    def reset_arm_tilt(self) -> None:
        """Resets the arm tilt to its default position."""
        self.set_arm_tilt(Preset.ARM_TILT_ANGLE)

    @in_production_mode
    def beep(self) -> None:
        """Beeps the robot for 100ms, used for indicating mode change."""
        duration = 100  # in ms
        self.ros_master.set_beep(duration)
        time.sleep(duration / 1000)

    def long_beep(self) -> None:
        """Beeps the robot for 200ms, used for indicating robot is ready."""
        if not self.production:
            return

        duration = 200
        self.ros_master.set_beep(duration)
        time.sleep(duration / 1000)

    @in_production_mode
    def get_data(self) -> dict:
        return {
            "accelerometer": self.ros_master.get_accelerometer_data(),
            "gyroscope": self.ros_master.get_gyroscope_data(),
            "magnetometer": self.ros_master.get_magnetometer_data(),
            "motion": self.ros_master.get_motion_data(),
            "voltage": self.ros_master.get_battery_voltage(),
        }

    def __del__(self) -> None:
        """Cleans up the robot."""
        del self.ros_master
