from unittest import TestCase

from controller.robot import Robot
from controller.direction import Direction


class TestDirections(TestCase):
    def test_directions(self):
        robot = Robot(testing=True)
        robot.set_speed(50)

        self.assertEqual(Direction.BACKWARD.name, "BACKWARD")
        self.assertEqual(
            robot.get_direction(Direction.FORWARD_LEFT), [50, 50, 25.0, 25.0]
        )
