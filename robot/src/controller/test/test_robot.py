from unittest import TestCase

from controller.enums import Direction, Preset
from controller.exceptions import NotInProductionMode
from controller.robot import Robot


class TestDirections(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot(production=False)

    def test_directions(self):
        self.robot.set_speed(30)

        self.assertEqual(Direction.BACKWARD.name, "BACKWARD")
        self.assertEqual(self.robot.get_speed(), 30)

    def test_defaults(self):
        self.assertEqual(Preset.SPEED, 50)
        self.assertEqual(Preset.WRIST_ANGLE, 135)
        self.assertEqual(Preset.ARM_ELBOW_ANGLE, 90)

    def test_production_mode(self):
        self.assertRaises(NotInProductionMode, self.robot.beep)
        self.assertRaises(NotInProductionMode, self.robot.reset_arm_rotation)

        self.robot.production = True

        self.assertRaises(AttributeError, self.robot.beep)
