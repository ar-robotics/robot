from unittest import TestCase

from controller.controller import Controller
from controller.direction import Direction


class TestController(TestCase):
    def test_controller(self):
        controller = Controller(production=False)

        self.assertEqual(
            controller.convert_coordinates_to_direction(0, 0), Direction.STOP
        )
        self.assertEqual(
            controller.convert_coordinates_to_direction(1, 0), Direction.TURN_RIGHT
        )
        self.assertEqual(
            controller.convert_coordinates_to_direction(-1, 0), Direction.TURN_LEFT
        )
        self.assertEqual(
            controller.convert_coordinates_to_direction(0, 1), Direction.FORWARD
        )
        self.assertEqual(
            controller.convert_coordinates_to_direction(0.1, 1), Direction.FORWARD
        )
        self.assertEqual(
            controller.convert_coordinates_to_direction(0.31, 1), Direction.TURN_RIGHT
        )
