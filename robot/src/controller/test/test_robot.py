from unittest import TestCase

from controller.direction import Direction
from controller.robot import Robot


class TestDirections(TestCase):
    def test_directions(self):
        robot = Robot(production=False)
        robot.set_speed(30)

        self.assertEqual(Direction.BACKWARD.name, "BACKWARD")
        self.assertEqual(robot.speed, 30)
