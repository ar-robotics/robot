import time
from unittest import TestCase

from controller.controller import Controller
from controller.enums import Direction
from controller.utils import fill_vector_msg, get_sleep_mode_after


class TestController(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = Controller(production=False)

    def test_coordinates_direction(self):
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(0, 0), Direction.STOP
        )
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(1, 0),
            Direction.TURN_RIGHT,
        )
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(-1, 0),
            Direction.TURN_LEFT,
        )
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(0, 1),
            Direction.FORWARD,
        )
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(0.1, 1),
            Direction.FORWARD,
        )
        self.assertEqual(
            self.controller.convert_coordinates_to_direction(0.31, 1),
            Direction.TURN_RIGHT,
        )

    def test_angle_x_conversion(self):
        self.assertEqual(self.controller.convert_x_to_angle_difference(0), 0)
        self.assertEqual(self.controller.convert_x_to_angle_difference(1), 90)
        self.assertEqual(self.controller.convert_x_to_angle_difference(-1), -90)
        self.assertEqual(self.controller.convert_x_to_angle_difference(0.5), 45)
        self.assertEqual(self.controller.convert_x_to_angle_difference(-0.5), -45)

    def test_angle_y_conversion(self):
        self.assertEqual(self.controller.convert_y_to_angle_difference(0), 0)
        self.assertEqual(self.controller.convert_y_to_angle_difference(1), 90)
        self.assertEqual(self.controller.convert_y_to_angle_difference(-1), 90)
        self.assertEqual(self.controller.convert_y_to_angle_difference(0.5), 45)
        self.assertEqual(self.controller.convert_y_to_angle_difference(-0.5), 45)
        self.assertEqual(self.controller.convert_y_to_angle_difference(1.1), 90)

    def test_pinch_angle_conversion(self):
        self.assertEqual(self.controller.convert_pinch_to_angle(0), 45)
        self.assertEqual(self.controller.convert_pinch_to_angle(1), 180)
        self.assertEqual(self.controller.convert_pinch_to_angle(-1), 180)
        self.assertEqual(self.controller.convert_pinch_to_angle(0.5), 90)
        self.assertEqual(self.controller.convert_pinch_to_angle(1.1), 180)

    def test_vector_fill(self):
        class MockVector:
            def __init__(self) -> None:
                self.x = 0
                self.y = 0
                self.z = 0

        class MockRobotData:
            def __init__(self):
                self.accelerometer = MockVector()

        msg = MockRobotData()
        fill_vector_msg(msg, "accelerometer", [1, 2, 3])

        self.assertEqual(msg.accelerometer.x, 1)
        self.assertEqual(msg.accelerometer.y, 2)
        self.assertEqual(msg.accelerometer.z, 3)

    def test_auto_sleep(self):
        # self.controller._set_last_message_received()
        self.controller.check_last_message_received()

        self.assertFalse(self.controller.is_sleeping)

        self.controller.last_command_received = time.time() - (
            get_sleep_mode_after() + 0.1
        )
        self.controller.check_last_message_received()

        self.assertTrue(self.controller.is_sleeping)
        self.assertEqual(self.controller.robot.last_direction, Direction.STOP)

    def test_set_last_message(self):
        self.controller.last_command_received = 0
        self.controller.is_sleeping = True

        time_now = time.time()
        self.assertTrue(self.controller.is_sleeping)
        self.assertLess(self.controller.last_command_received, time_now)

        self.assertRaises(TypeError, self.controller.handle_vr_mode)

        self.assertFalse(self.controller.is_sleeping)
        self.assertGreater(self.controller.last_command_received, time_now)

    def test_speed_out_of_range(self):
        self.assertFalse(self.controller._speed_out_of_range(50))
        self.assertFalse(self.controller._speed_out_of_range(0))
        self.assertFalse(self.controller._speed_out_of_range(100))
        self.assertTrue(self.controller._speed_out_of_range(101))
        self.assertTrue(self.controller._speed_out_of_range(-20))
