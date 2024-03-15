from unittest import TestCase

from controller.rosmaster import Rosmaster


class TestRosmaster(TestCase):
    def setUp(cls) -> None:
        cls.r = Rosmaster(1, com="testing")

    def test_clamping(self):
        self.assertEqual(self.r._clamp_motor_value(40), 40)
        self.assertEqual(self.r._clamp_motor_value(-40), -40)
        self.assertEqual(self.r._clamp_motor_value(100), 100)
        self.assertEqual(self.r._clamp_motor_value(101), 100)
        self.assertEqual(self.r._clamp_motor_value(-100), -100)
        self.assertEqual(self.r._clamp_motor_value(-101), -100)

    def test_arm_convert_angle(self):
        self.assertEqual(self.r._arm_convert_angle(1, 0), 254)
        # self.assertEqual(self.r._arm_convert_angle(1, 0), 254)
        self.assertEqual(self.r._arm_convert_angle(7, 0), -1)
