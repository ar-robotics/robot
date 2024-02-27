from unittest import TestCase

from controller.direction import Direction
from controller.robot import Robot


class TestDirections(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot(production=False)

    def test_directions(self):
        self.robot.set_speed(30)

        self.assertEqual(Direction.BACKWARD.name, "BACKWARD")
        self.assertEqual(self.robot.get_speed(), 30)
