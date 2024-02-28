from unittest import TestCase

from controller.controller import Controller
from controller.direction import Direction


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
