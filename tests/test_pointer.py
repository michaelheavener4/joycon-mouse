import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import joycon_mouse as jm


class PointerTests(unittest.TestCase):
    def test_rest_holds_still(self):
        pointer = jm.Pointer()
        dx, dy = pointer.motion((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.1)
        self.assertEqual((dx, dy), (0, 0))

    def test_turn_left_moves_cursor_left(self):
        # Positive rotation around +Z (up) turns the shoulder end to the left.
        pointer = jm.Pointer(sens=20, deadzone=0)
        dx, dy = pointer.motion((0.0, 0.0, 10.0), (0.0, 0.0, 1.0), 0.05)
        self.assertEqual(dx, -10)
        self.assertEqual(dy, 0)

    def test_tip_down_moves_cursor_down(self):
        # Positive rotation around +Y tips the shoulder end down.
        pointer = jm.Pointer(sens=20, deadzone=0)
        dx, dy = pointer.motion((0.0, 10.0, 0.0), (0.0, 0.0, 1.0), 0.05)
        self.assertEqual(dx, 0)
        self.assertEqual(dy, 10)

    def test_deadzone_rejects_bias_sized_drift(self):
        pointer = jm.Pointer(sens=22, deadzone=1.2)
        dx, dy = pointer.motion((0.0, 0.5, -0.8), (0.08, -0.02, 1.01), 2.0)
        self.assertEqual((dx, dy), (0, 0))

    def test_shake_does_not_replace_gravity(self):
        pointer = jm.Pointer()
        pointer.update_gravity((0.0, 0.0, 1.0))
        pointer.update_gravity((4.0, 0.0, 0.2))
        self.assertAlmostEqual(pointer.gravity[2], 1.0, places=2)


class ReportTests(unittest.TestCase):
    def test_right_buttons_and_stick(self):
        packet = bytearray(49)
        packet[0] = 0x30
        packet[3] = jm.BTN_A | jm.BTN_ZR
        packet[4] = jm.BTN_PLUS >> 8
        # Stick X 2104 = 0x838, stick Y 2200 = 0x898, above a 2048-ish center.
        packet[9] = 0x38
        packet[10] = 0x88
        packet[11] = 0x89
        report = jm.parse_report(bytes(packet), right=True)
        self.assertIsNotNone(report)
        self.assertEqual(report.stick_x, 0x838)
        self.assertEqual(report.stick_y, 0x898)
        left, right, middle, pause, precision = jm.clicks(report.buttons, right=True)
        self.assertTrue(left)
        self.assertFalse(right)
        self.assertFalse(middle)
        self.assertTrue(pause)
        self.assertFalse(precision)

    def test_non_imu_report_ignored(self):
        self.assertIsNone(jm.parse_report(b"\x21" + bytes(20), right=True))

    def test_stick_up_scrolls_up(self):
        horizontal, vertical, acc_x, acc_y = jm.stick_notches(
            2100, 3600, 2100, 1850, 0.5, 0.0, 0.0
        )
        self.assertEqual(horizontal, 0)
        self.assertGreater(vertical, 0)
        self.assertGreaterEqual(acc_y, 0)

    def test_fractional_pixels_carry(self):
        pointer = jm.Pointer(sens=10, deadzone=0)
        first = pointer.motion((0.0, 0.0, 10.0), (0.0, 0.0, 1.0), 0.04)
        second = pointer.motion((0.0, 0.0, 10.0), (0.0, 0.0, 1.0), 0.04)
        # 10 dps * 0.04 s * 10 px/deg = 4 px per call, to the left.
        self.assertEqual(first, (-4, 0))
        self.assertEqual(second, (-4, 0))
        self.assertTrue(math.isclose(pointer.fx, 0.0, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()
