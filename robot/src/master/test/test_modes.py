from unittest import TestCase

from master.modes import Mode


class TestModes(TestCase):
    def test_mode_change(self):
        mode = Mode.EMERGENCY
        self.assertEqual(mode, Mode(3))
        self.assertNotEqual(mode, Mode(1))
        self.assertEqual(Mode(3).name, "EMERGENCY")
