from unittest import TestCase

from vr_linker.vr_linker import VRLinker


class TestVRLinker(TestCase):
    # def setUp(cls) -> None:
    #     cls.vr_linker= VRLinker(None)

    def test_vr_linker(self):
        vr_linker = VRLinker(None)

        self.assertTrue(vr_linker._is_vr_data_message({"x": 0, "y": 0, "speed": 0}))
        self.assertFalse(vr_linker._is_vr_data_message({"x": 0}))

        self.assertTrue(vr_linker._is_vr_hand_message({"x": 0.6}))
        self.assertTrue(vr_linker._is_vr_hand_message({"x": 0, "wrist": 0}))
        self.assertFalse(vr_linker._is_vr_hand_message({"x": 0, "speed": 55}))

        self.assertEqual(vr_linker._to_json({"a": True}), b'{"a": true}')
        self.assertEqual(vr_linker._to_json(b'{"a": true}'), {"a": True})

    def test_msg_to_json(self):
        class MockVector:
            def __init__(self) -> None:
                self.x = 7
                self.y = 6
                self.z = 5

        class MockRobotData:
            def __init__(self):
                self.accelerometer = MockVector()

        msg = MockRobotData()
        msg = VRLinker._get_vector_data(msg, "accelerometer")
        self.assertEqual(msg, [7, 6, 5])
